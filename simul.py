import argparse
import ctypes
import gc
import logging
import os
import select
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import can

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sil.decoder import decode_can_frame, decode_dxl_packet, dxl_goal_deg, dxl_read_ids, split_dxl_packets
from sil.encoder import (
    encode_dxl_ping,
    encode_dxl_read,
    encode_dxl_write,
    encode_maxon_sdo_ack,
    motor_feedback,
)
from sil.mapping import (
    CAN_BUS_MOTORS,
    DEFAULT_URDF_PATH,
    DXL_MOTORS,
    MAXON_SPEC,
    PRODUCTION_TO_URDF_JOINT,
    STARTUP_DXL_POSE_DEG,
    TMOTOR_SPEC,
    dxl_to_urdf_deg,
    production_to_urdf_deg,
)
from sil.motor_state import NmtState
from sil.pybullet_backend import PyBulletBackend
from sil.router import (
    MAX_DT,
    MAXON_FRICTION_TORQUE,
    MAXON_REFLECTED_INERTIA,
    MotorRouter,
    TORQUE_PHYSICS,
)
from sil.pybullet_backend import PHYSICS_TIMESTEP

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# [우회] SIL에서만 목이 팔보다 컨트롤러 lookahead(~1.12s)만큼 앞서 보이는 현상 보정.
# 컨트롤러/하드웨어는 그대로 두고, SIL에서 목(DXL) goal 적용을 이만큼 지연시켜 팔과 시작점을 맞춘다.
# 곡 무관 고정값으로 측정됨(TIM/BI/TY_short 모두 ~1.13s). 필요시 튜닝.
NECK_DELAY_S = 2.4


# 시뮬레이터 본체
class FrameSimulator:
    def __init__(
        self,
        can_buses,
        dxl_path: Path,
        urdf_path: Path,
        mode: str,
        feedback_hz: float,
    ):
        self.can_buses = list(can_buses)
        self.dxl_path = Path(dxl_path)
        self.feedback_dt = 1.0 / feedback_hz
        self.backend = PyBulletBackend(
            urdf_path=urdf_path,
            mode=mode,
            torque_physics=TORQUE_PHYSICS,
            reflected_inertia=MAXON_REFLECTED_INERTIA,
            friction_torque=MAXON_FRICTION_TORQUE,
        )
        self.router = MotorRouter()
        self.router_lock = threading.Lock()
        self.nmt_state = NmtState()
        self.bus_map: Dict[str, object] = {}
        self.motor_bus: Dict[str, str] = self._default_motor_bus()
        # TMotor는 vcan0/vcan1에만, Maxon은 vcan2/vcan3에만 있다.
        # TMotor 전용 버스는 별도 responder thread가 recv+echo를 맡고,
        # Maxon 버스는 main loop의 _poll_can이 계속 처리한다.
        self.tmotor_buses = [
            can_bus for can_bus, motors in CAN_BUS_MOTORS.items()
            if can_bus in self.can_buses and len(motors) > 0
            and all(motor in TMOTOR_SPEC for motor in motors)
        ]
        self.maxon_buses = [
            can_bus for can_bus in self.can_buses if can_bus not in self.tmotor_buses
        ]
        self.tmotor_stage: Dict[str, float] = {}
        self.tmotor_lock = threading.Lock()
        self.tmotor_stop = threading.Event()
        self.tmotor_thread: Optional[threading.Thread] = None
        self.dxl_feedback: Dict[int, float] = self._default_dxl_feedback()
        self.dxl_targets: Dict[str, float] = {}
        self.dxl_delay_q = []  # [우회] (apply_time, dxl_id, goal) 시간순 지연 큐
        self.dxl_lock = threading.Lock()
        self.dxl_stop = threading.Event()
        self.dxl_thread: Optional[threading.Thread] = None
        self.dxl_fd: Optional[int] = None
        self.dxl_buffer = b""
        self.last_feedback = 0.0
        self.last_motion = 0.0
        self.needs_step = False
        # torque 물리 모드: 벽시계와 고정 timestep을 맞추는 누산기
        self.last_step = 0.0
        self.step_accum = 0.0
        # [TIMING] 측정용 디버그 로그(neck/arm 적용 시각+각도). 재시작마다 새 파일. 측정 끝나면 제거.
        self.timing_file = None
        self.timing_t0 = 0.0
        self.timing_last: Dict[str, float] = {}

    # 생명주기
    def run(self) -> int:
        try:
            self.backend.start()
            self.backend.apply_targets(self.router.startup_targets())
            self.backend.step()
            self.last_motion = time.monotonic()
            self.last_step = self.last_motion
            self.timing_t0 = self.last_motion
            self.timing_file = self._open_timing_log()  # [TIMING] 재시작마다 새 파일
            # PyBullet 내부 thread는 RT가 아니어야 하므로 backend.start() 이후,
            # 우리 thread 생성 이전에 RT 정책을 건다(이후 만든 thread가 정책을 상속).
            self._apply_realtime()
            self._open_can_buses()
            self._open_dxl()
            self._start_dxl_thread()
            self._start_tmotor_thread()
            # warmup이 끝난 시점에 살아있는 객체를 freeze해 GC 스캔 부담을 줄인다.
            self._freeze_gc()

            while True:
                self._poll_can()
                self._release_delayed_dxl()
                self._apply_dxl_targets()
                self._apply_tmotor_targets()
                self._advance_motion()
                if TORQUE_PHYSICS:
                    self._step_torque_physics()
                elif self.needs_step:
                    self.backend.step()
                    self.needs_step = False
                time.sleep(0.0005)
        except KeyboardInterrupt:
            return 0
        finally:
            self.close()

    def close(self) -> None:
        if self.timing_file is not None:
            self.timing_file.close()
            self.timing_file = None
        self.tmotor_stop.set()
        if self.tmotor_thread is not None:
            self.tmotor_thread.join(timeout=0.5)
            self.tmotor_thread = None

        self.dxl_stop.set()
        if self.dxl_thread is not None:
            self.dxl_thread.join(timeout=0.5)
            self.dxl_thread = None

        if self.dxl_fd is not None:
            os.close(self.dxl_fd)
            self.dxl_fd = None

        for can_bus in self.bus_map.values():
            try:
                can_bus.shutdown()
            except Exception:
                pass

        self.bus_map.clear()
        self.backend.close()

    # 실시간 hardening: 비-RT Python에서 feedback 멈춤 원인을 최대한 줄인다.
    # 비-RT 커널에서는 deschedule을 완전히 없애진 못하므로 best-effort이며,
    # 권한이 없으면 각 항목을 조용히 건너뛴다. (SCHED_FIFO/mlockall은 root 필요)
    def _apply_realtime(self) -> None:
        # GIL: 전환 검사 주기를 5ms→0.5ms로 줄여, main이 순수 Python에 갇혀도
        # echo thread가 더 빨리 GIL을 넘겨받게 한다.
        sys.setswitchinterval(0.0005)

        applied = []
        # OS 스케줄링: SCHED_FIFO는 normal(SCHED_OTHER) 프로세스(brain/TTS 등)에 의한
        # deschedule을 막는다. 우리 loop는 매 iteration sleep으로 양보하므로 머신을 굶기지 않는다.
        try:
            param = os.sched_param(10)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            applied.append("SCHED_FIFO(10)")
        except (PermissionError, OSError, AttributeError):
            try:
                os.nice(-10)
                applied.append("nice(-10)")
            except (PermissionError, OSError):
                pass

        # 메모리 page를 RAM에 고정해 page fault/swap로 인한 멈춤을 차단한다.
        try:
            libc = ctypes.CDLL("libc.so.6", use_errno=True)
            MCL_CURRENT = 1
            MCL_FUTURE = 2
            if libc.mlockall(MCL_CURRENT | MCL_FUTURE) == 0:
                applied.append("mlockall")
        except Exception:
            pass

        if applied:
            logger.info("[SIL] realtime hardening: %s", ", ".join(applied))
        else:
            logger.info("[SIL] realtime hardening: none (권한 없음? sudo로 실행 필요)")

    # GC 멈춤 완화: warmup 후 살아있는 객체를 permanent gen으로 옮겨(freeze)
    # 이후 수집 스캔 대상에서 빼, 매 collection의 stop-the-world 시간을 줄인다.
    # collector 자체는 켜둔 채라 순환 참조 누수 위험은 없다.
    # (gc.disable()은 누수 위험 때문에 되돌렸다.)
    def _freeze_gc(self) -> None:
        gc.collect()
        gc.freeze()
        logger.info("[SIL] gc frozen after warmup (collector still on)")

    # 장치 초기화
    def _default_motor_bus(self) -> Dict[str, str]:
        motor_bus: Dict[str, str] = {}
        for can_bus, motors in CAN_BUS_MOTORS.items():
            if can_bus in self.can_buses:
                for motor in motors:
                    motor_bus[motor] = can_bus
        return motor_bus

    def _default_dxl_feedback(self) -> Dict[int, float]:
        feedback: Dict[int, float] = {}
        for dxl_id, motor in DXL_MOTORS.items():
            feedback[dxl_id] = STARTUP_DXL_POSE_DEG.get(motor, 0.0)
        return feedback

    def _open_can_buses(self) -> None:
        for can_bus in self.can_buses:
            self.bus_map[can_bus] = can.interface.Bus(
                channel=can_bus,
                interface="socketcan",
                receive_own_messages=False,
            )
            logger.info("[SIL] opened %s", can_bus)

    def _open_dxl(self) -> None:
        if not self.dxl_path.exists():
            logger.warning("[SIL] DXL PTY not found: %s", self.dxl_path)
            return

        self.dxl_fd = os.open(str(self.dxl_path), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        logger.info("[SIL] opened DXL PTY %s", self.dxl_path)

    def _start_dxl_thread(self) -> None:
        if self.dxl_fd is None:
            return

        self.dxl_thread = threading.Thread(target=self._dxl_loop, daemon=True)
        self.dxl_thread.start()

    def _dxl_loop(self) -> None:
        while not self.dxl_stop.is_set():
            self._poll_dxl()
            time.sleep(0.0002)

    # CAN 프레임 처리 반복 (Maxon 버스 전용; TMotor 버스는 _tmotor_loop가 맡는다)
    def _poll_can(self) -> None:
        for can_bus in self.maxon_buses:
            bus_obj = self.bus_map.get(can_bus)
            if bus_obj is None:
                continue

            while True:
                frame = bus_obj.recv(timeout=0.0)
                if frame is None:
                    break

                command = decode_can_frame(frame)
                if command is None:
                    continue

                if command.kind.startswith("nmt_"):
                    self.nmt_state.transition(command.kind, command.motor)
                    continue

                if command.kind == "maxon_sync":
                    self._send_maxon_sync_feedback(can_bus)
                    continue

                if command.motor:
                    self.motor_bus[command.motor] = can_bus

                if command.kind == "maxon_sdo":
                    reply = encode_maxon_sdo_ack(command.motor)
                    if reply is not None:
                        bus_obj.send(reply)
                    continue

                with self.router_lock:
                    targets = self.router.route_can(command)
                if targets:
                    self.backend.apply_targets(targets)
                    self.needs_step = True
                    self._log_apply("maxon", targets)  # 손목/발 등 position 경로

    # DXL 패킷 처리 반복
    def _poll_dxl(self) -> None:
        if self.dxl_fd is None:
            return

        for _ in range(16):
            readable, _, _ = select.select([self.dxl_fd], [], [], 0)
            if not readable:
                return

            try:
                data = os.read(self.dxl_fd, 4096)
            except BlockingIOError:
                return

            if not data:
                return

            self.dxl_buffer += data
            packets, self.dxl_buffer = split_dxl_packets(self.dxl_buffer)
            for packet in packets:
                commands = decode_dxl_packet(packet)
                for command in commands:
                    self._handle_dxl(command)

    def _handle_dxl(self, command) -> None:
        if self.dxl_fd is None:
            return

        if command.kind == "ping":
            response = encode_dxl_ping(command.dxl_id)
            if response is not None:
                self._write_dxl(response)
            return

        if command.kind == "write":
            response = encode_dxl_write(command.dxl_id)
            if response is not None:
                self._write_dxl(response)
            return

        if command.kind == "sync_read":
            # DXL 실제 장치 모델:
            # 주기 feedback은 만들지 않는다. SyncWrite로 받은 최신 goal을 보관해 두었다가
            # SyncRead packet을 받을 때만 status packet으로 즉시 echo한다.
            for dxl_id in dxl_read_ids(command):
                joint_deg = self.dxl_feedback.get(dxl_id, 0.0)
                response = encode_dxl_read(dxl_id, joint_deg)
                if response is not None:
                    self._write_dxl(response)
            return

        if command.kind == "sync_write":
            goal_deg = dxl_goal_deg(command.data)
            if goal_deg is not None:
                # [우회] 즉시 적용하지 않고 NECK_DELAY_S 뒤에 적용되도록 큐에 넣는다.
                with self.dxl_lock:
                    self.dxl_delay_q.append((time.monotonic() + NECK_DELAY_S, command.dxl_id, goal_deg))
            return

    # [우회] 지연 큐에서 도착시간 지난 목 goal을 꺼내 staging (FIFO/시간순이라 prefix만 꺼냄).
    def _release_delayed_dxl(self) -> None:
        now = time.monotonic()
        with self.dxl_lock:
            queue = self.dxl_delay_q
            count = 0
            while count < len(queue) and queue[count][0] <= now:
                count += 1
            ready = queue[:count]
            del queue[:count]
        for _, dxl_id, goal_deg in ready:
            self.dxl_feedback[dxl_id] = goal_deg
            self._stage_dxl_target(dxl_id, goal_deg)

    def _stage_dxl_target(self, dxl_id: int, goal_deg: float) -> None:
        motor = DXL_MOTORS.get(dxl_id)
        if motor is None:
            return

        targets = dxl_to_urdf_deg(motor, goal_deg)
        if not targets:
            return

        with self.dxl_lock:
            self.dxl_targets.update(targets)

    def _write_dxl(self, packet: bytes) -> None:
        if self.dxl_fd is None:
            return

        view = memoryview(packet)
        while view:
            try:
                sent = os.write(self.dxl_fd, view)
            except BlockingIOError:
                time.sleep(0.0001)
                continue
            if sent == 0:
                return
            view = view[sent:]

    # [TIMING] 재시작마다 apply_timing_N.csv 새로 연다(덮어쓰기 방지).
    def _open_timing_log(self):
        base = Path(__file__).resolve().parent
        for n in range(1, 1000):
            path = base / f"apply_timing_{n}.csv"
            if not path.exists():
                handle = open(path, "w")
                handle.write("elapsed_s,stream,joint,deg\n")
                logger.info("[TIMING] apply log -> %s", path)
                return handle
        return None

    # [TIMING] neck/arm 적용 시각+각도를 long-format으로 기록(둘 다 main loop라 락 불필요).
    # 변화 0.05deg 이상일 때만 기록(고빈도 flush가 타이밍 흔드는 것 방지).
    def _log_apply(self, stream: str, targets: Dict[str, float]) -> None:
        if self.timing_file is None:
            return
        elapsed = time.monotonic() - self.timing_t0
        wrote = False
        for joint, deg in targets.items():
            key = stream + "|" + joint
            last = self.timing_last.get(key)
            if last is not None and abs(deg - last) < 0.05:
                continue
            self.timing_last[key] = deg
            self.timing_file.write(f"{elapsed:.4f},{stream},{joint},{deg:.3f}\n")
            wrote = True
        if wrote:
            self.timing_file.flush()

    def _apply_dxl_targets(self) -> None:
        with self.dxl_lock:
            if not self.dxl_targets:
                return

            targets = dict(self.dxl_targets)
            self.dxl_targets.clear()

        self.backend.apply_targets(targets)
        self.needs_step = True
        self._log_apply("neck", targets)

    # 모션/피드백 반복
    def _advance_motion(self) -> None:
        now = time.monotonic()
        dt = now - self.last_motion
        self.last_motion = now

        with self.router_lock:
            targets = self.router.advance(dt)
        if targets:
            self.backend.apply_targets(targets)
            self.needs_step = True
            self._log_apply("arm", targets)  # velocity 적분으로 적용되는 팔(+허리)

    # torque 물리 모드: 흐른 벽시계만큼 고정 timestep으로 PyBullet을 적분한다.
    # 매 substep마다 출력단 토크를 다시 인가한다 (TORQUE_CONTROL은 step 단위라).
    def _step_torque_physics(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_step
        self.last_step = now

        # spiral of death 방지: 한 번에 따라잡을 시간을 MAX_DT로 제한한다.
        self.step_accum = min(self.step_accum + elapsed, MAX_DT)

        while self.step_accum >= PHYSICS_TIMESTEP:
            with self.router_lock:
                joint_torques = self.router.torque_targets()
            self.backend.apply_joint_torques(joint_torques)
            self.backend.step()
            self.step_accum -= PHYSICS_TIMESTEP

    # TMotor responder thread
    # PyBullet step에 막히는 main loop와 달리, TMotor 버스(vcan0/vcan1)의 recv+echo를
    # 전용 thread로 분리해 DrumRobot2의 current feedback이 step 지연에도 신선하게 유지되도록 한다.
    def _start_tmotor_thread(self) -> None:
        if not self.tmotor_buses:
            return

        self.tmotor_thread = threading.Thread(target=self._tmotor_loop, daemon=True)
        self.tmotor_thread.start()

    def _tmotor_loop(self) -> None:
        while not self.tmotor_stop.is_set():
            self._drain_tmotor_buses()
            self._emit_tmotor_feedback()
            time.sleep(0.0002)

    # TMotor 버스를 비우면서, 들어온 명령을 즉시 echo하고 PyBullet 반영용으로 staging한다.
    def _drain_tmotor_buses(self) -> None:
        staged: Dict[str, float] = {}
        for can_bus in self.tmotor_buses:
            bus_obj = self.bus_map.get(can_bus)
            if bus_obj is None:
                continue

            while True:
                frame = bus_obj.recv(timeout=0.0)
                if frame is None:
                    break

                command = decode_can_frame(frame)
                if command is None:
                    continue

                motor = command.motor
                if motor is None or motor not in TMOTOR_SPEC:
                    continue

                self.motor_bus[motor] = can_bus
                with self.router_lock:
                    targets = self.router.route_can(command)

                # position 명령만 즉시 echo한다. velocity 명령은 route가 None을 주고,
                # advance()가 적분한 motor_target를 _emit_tmotor_feedback이 echo한다.
                if targets:
                    staged.update(targets)
                    for urdf_deg in targets.values():
                        self._send_motor_feedback(motor, can_bus, urdf_deg)

        if staged:
            with self.tmotor_lock:
                self.tmotor_stage.update(staged)

    # TMotor 독립 heartbeat: 명령이 잠시 없거나 velocity 적분 중이어도 current를 유지한다.
    # source는 router.motor_target 하나로 통일한다(position/velocity/discovery 모두 커버).
    def _emit_tmotor_feedback(self) -> None:
        now = time.monotonic()
        if now - self.last_feedback < self.feedback_dt:
            return
        self.last_feedback = now

        for motor in TMOTOR_SPEC:
            can_bus = self.motor_bus.get(motor)
            if can_bus not in self.tmotor_buses:
                continue

            with self.router_lock:
                joint_deg = self.router.motor_target.get(motor)
            if joint_deg is None:
                continue

            urdf_deg = production_to_urdf_deg(motor, joint_deg)
            self._send_motor_feedback(motor, can_bus, urdf_deg)

    # TMotor thread가 staging한 target을 main thread에서 PyBullet에 반영한다.
    def _apply_tmotor_targets(self) -> None:
        with self.tmotor_lock:
            if not self.tmotor_stage:
                return

            targets = dict(self.tmotor_stage)
            self.tmotor_stage.clear()

        self.backend.apply_targets(targets)
        self.needs_step = True
        self._log_apply("arm", targets)  # position 모드 팔(있을 경우)

    def _send_maxon_sync_feedback(self, can_bus: str) -> None:
        # Maxon 실제 장치 모델:
        # 200Hz 주기 feedback을 따로 뿌리지 않고, CANopen SYNC(0x80)를 받을 때
        # Operational 상태인 노드의 TPDO 위치 feedback만 전송한다.
        state = self.backend.read_joint_states()
        for motor, motor_bus in self.motor_bus.items():
            if motor_bus != can_bus or motor not in MAXON_SPEC:
                continue

            joint_name = PRODUCTION_TO_URDF_JOINT.get(motor)
            if joint_name is None:
                continue

            urdf_deg = state.get(joint_name)
            if urdf_deg is not None:
                self._send_motor_feedback(motor, can_bus, urdf_deg)

    def _send_motor_feedback(
        self,
        motor: Optional[str],
        can_bus: str,
        urdf_deg: float,
    ) -> None:
        if motor is None:
            return

        if motor in MAXON_SPEC and not self.nmt_state.is_operational(motor):
            return

        bus_obj = self.bus_map.get(can_bus)
        if bus_obj is None:
            return

        frame = motor_feedback(motor, urdf_deg)
        if frame is not None:
            bus_obj.send(frame)


# 명령줄 진입점
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run frame-level DrumRobot SIL.")
    parser.add_argument("--mode", choices=["gui", "direct"], default="gui")
    parser.add_argument("--dxl", type=Path, default=Path("/tmp/ttyUSB0_sim"))
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument(
        "--feedback-hz",
        type=float,
        default=200.0,
        help="TMotor idle/discovery status rate.",
    )
    parser.add_argument("can_buses", nargs="*", default=["vcan0", "vcan1", "vcan2", "vcan3"])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    simulator = FrameSimulator(
        can_buses=args.can_buses,
        dxl_path=args.dxl,
        urdf_path=args.urdf,
        mode=args.mode,
        feedback_hz=args.feedback_hz,
    )
    return simulator.run()


if __name__ == "__main__":
    raise SystemExit(main())
