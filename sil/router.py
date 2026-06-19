from typing import Dict, Optional

from .decoder import CanCommand, DxlCommand, dxl_goal_deg
from .mapping import (
    DXL_MOTORS,
    MAXON_SPEC,
    PRODUCTION_TO_URDF_JOINT,
    STARTUP_CAN_POSE_DEG,
    STARTUP_DXL_POSE_DEG,
    TMOTOR_SPEC,
    dxl_to_urdf_deg,
    joint_load_inertia,
    motor_to_joint_deg,
    production_to_urdf_deg,
    production_to_urdf_torque,
)

# ── Maxon wrist drive 동특성 상수 ─────────────────────────────────────────
# 출처: docs/Maxon_wrist_motor.pdf
#   모터:   DCX22L GB KL 48V
#   기어:   GPX22HP 35:1 (2-stage High Power)
#   엔코더: ENX16 EASY 1024 IMP
# decoder는 torque 명령을 모터축 토크(mNm)로 준다. 반면 router는 출력단
# (기어 이후) 각도/각속도를 deg, deg/s로 추적한다. 그래서 토크/관성은 모두
# 출력단 기준으로 환산해서 적분한다. (foot도 같은 35:1 기어라 동일 모델 사용)
RAD_TO_DEG = 57.29577951308232

# 모터 datasheet 값
MAXON_KT = 45.2                # mNm/A,  토크 상수 Kt
MAXON_STALL_TORQUE = 294.0     # mNm,    정지 토크 (모터 토크 상한)
MAXON_IDLE_CURRENT = 0.0162    # A,      무부하 전류 (마찰 추정용)
MAXON_FREE_SPEED = 10100.0     # rpm,    무부하 속도
MAXON_ROTOR_INERTIA = 8.85e-7  # kg·m²,  로터 관성 (8.85 gcm²)

# 기어 datasheet 값 (GPX22HP 35:1)
MAXON_GEAR_RATIO = 35.0        # 감속비
MAXON_GEAR_EFFICIENCY = 0.75   # 최대 효율 75%
MAXON_PEAK_TORQUE = 3.0        # Nm,     출력단 순간 허용 토크
MAXON_GEAR_INERTIA = 1.31e-7   # kg·m²,  기어 관성 (1.31 gcm²)

# 위 datasheet 값에서 유도한 출력단 환산 상수
# 무부하 전류 × Kt = 모터 마찰 토크 → 기어로 출력단 환산
MAXON_FRICTION_TORQUE = (
    MAXON_KT * MAXON_IDLE_CURRENT / 1000.0 * MAXON_GEAR_RATIO * MAXON_GEAR_EFFICIENCY
)                              # Nm,     출력단 마찰 토크
# 무부하 속도를 출력단 deg/s로 환산 (rpm × 6 = deg/s)
MAXON_SPEED_LIMIT = MAXON_FREE_SPEED / MAXON_GEAR_RATIO * 6.0
# 출력단으로 반사된 (로터 + 기어) 관성 = (J_rotor + J_gear) × 기어비²
# 부하(스틱/링크) 관성은 여기에 더한다 → mapping.joint_load_inertia()가 URDF에서 읽는다.
MAXON_REFLECTED_INERTIA = (
    (MAXON_ROTOR_INERTIA + MAXON_GEAR_INERTIA) * MAXON_GEAR_RATIO * MAXON_GEAR_RATIO
)                              # kg·m²

MAX_DT = 0.02                  # s,      적분 timestep 상한 (수치 안정용, 모터 스펙 아님)

# torque 모드 처리 방식 선택
#   False: router가 1D로 손적분한다 (결정론적, step 타이밍과 무관, 현행 기본).
#   True:  PyBullet이 실제 동역학으로 적분한다 (datasheet 관성/토크가 물리 입력).
#          이 경우 router는 torque 관절을 적분하지 않고 출력단 토크만 만들어 준다.
TORQUE_PHYSICS = True


class MotorRouter:
    # 내부 상태
    def __init__(self):
        self.motor_target: Dict[str, float] = dict(STARTUP_CAN_POSE_DEG)
        self.dxl_target: Dict[str, float] = dict(STARTUP_DXL_POSE_DEG)
        self.motor_mode: Dict[str, str] = {}
        self.joint_velocity: Dict[str, float] = {}
        self.motor_torque: Dict[str, float] = {}

    # 시작 시 분배
    def startup_targets(self) -> Dict[str, float]:
        targets: Dict[str, float] = {}
        for motor, joint_deg in self.motor_target.items():
            targets.update(self._can_target(motor, joint_deg))

        for motor, dxl_deg in self.dxl_target.items():
            dxl_target = dxl_to_urdf_deg(motor, dxl_deg)
            if dxl_target is not None:
                targets.update(dxl_target)

        return targets

    # CAN 명령 분배
    def route_can(self, command: CanCommand) -> Optional[Dict[str, float]]:
        motor = command.motor

        if command.kind in {"tmotor_position", "maxon_position"} and command.position is not None:
            joint_deg = motor_to_joint_deg(motor, command.position)
            self.motor_target[motor] = joint_deg
            self.motor_mode[motor] = "position"
            self.joint_velocity[motor] = 0.0
            return self._can_target(motor, joint_deg)

        if command.kind == "tmotor_velocity" and command.velocity is not None:
            self.motor_mode[motor] = "velocity"
            self.joint_velocity[motor] = self._tmotor_velocity(motor, command.velocity)
            return None

        if command.kind == "maxon_velocity" and command.velocity is not None:
            self.motor_mode[motor] = "velocity"
            self.joint_velocity[motor] = self._maxon_velocity(motor, command.velocity)
            return None

        if command.kind == "maxon_torque" and command.torque_mnm is not None:
            self.motor_mode[motor] = "torque"
            self.motor_torque[motor] = command.torque_mnm
            return None

        return None

    # CAN 모션 적분
    def advance(self, dt: float) -> Dict[str, float]:
        dt = max(0.0, min(dt, MAX_DT))
        if dt == 0.0:
            return {}

        targets: Dict[str, float] = {}
        for motor, mode in self.motor_mode.items():
            if mode == "velocity":
                joint_deg = self._advance_velocity(motor, dt)
            elif mode == "torque" and not TORQUE_PHYSICS:
                joint_deg = self._advance_torque(motor, dt)
            else:
                # TORQUE_PHYSICS=True면 torque 관절은 PyBullet이 적분하므로 건너뛴다.
                continue

            self.motor_target[motor] = joint_deg
            targets.update(self._can_target(motor, joint_deg))

        return targets

    # physics 모드에서 torque 관절에 인가할 출력단 토크(Nm)
    def torque_targets(self) -> Dict[str, float]:
        commands: Dict[str, float] = {}
        for motor, mode in self.motor_mode.items():
            if mode != "torque":
                continue

            joint_name = PRODUCTION_TO_URDF_JOINT.get(motor)
            if joint_name is None:
                continue

            # 모터축 토크를 stall로 제한 → 기어×효율로 출력단 환산 → 기어 순간 토크로 제한
            motor_torque = self.motor_torque.get(motor, 0.0)
            motor_torque = max(-MAXON_STALL_TORQUE, min(MAXON_STALL_TORQUE, motor_torque))
            output_torque = motor_torque / 1000.0 * MAXON_GEAR_RATIO * MAXON_GEAR_EFFICIENCY
            output_torque = max(-MAXON_PEAK_TORQUE, min(MAXON_PEAK_TORQUE, output_torque))

            commands[joint_name] = production_to_urdf_torque(motor, output_torque)

        return commands

    def _can_target(self, motor: str, joint_deg: float) -> Dict[str, float]:
        joint_name = PRODUCTION_TO_URDF_JOINT.get(motor)
        if joint_name is None:
            return {}
        return {joint_name: production_to_urdf_deg(motor, joint_deg)}

    def _advance_velocity(self, motor: str, dt: float) -> float:
        joint_deg = self.motor_target.get(motor, STARTUP_CAN_POSE_DEG.get(motor, 0.0))
        velocity = self.joint_velocity.get(motor, 0.0)
        return joint_deg + velocity * dt

    def _advance_torque(self, motor: str, dt: float) -> float:
        joint_deg = self.motor_target.get(motor, STARTUP_CAN_POSE_DEG.get(motor, 0.0))
        velocity = self.joint_velocity.get(motor, 0.0)
        torque_mnm = self.motor_torque.get(motor, 0.0)

        # 모터축 토크 명령을 stall torque로 제한한다
        motor_torque = max(-MAXON_STALL_TORQUE, min(MAXON_STALL_TORQUE, torque_mnm))

        # 기어로 증폭해 출력단 토크(Nm)로 바꾸고, 기어 순간 허용 토크로 제한한다
        output_torque = motor_torque / 1000.0 * MAXON_GEAR_RATIO * MAXON_GEAR_EFFICIENCY
        output_torque = max(-MAXON_PEAK_TORQUE, min(MAXON_PEAK_TORQUE, output_torque))

        # 무부하 전류 기반 마찰을 운동 반대 방향으로 뺀다
        if velocity > 0.0:
            output_torque -= MAXON_FRICTION_TORQUE
        elif velocity < 0.0:
            output_torque += MAXON_FRICTION_TORQUE

        # 출력단 각가속도(rad/s²) = 순 토크 / 총 관성, deg/s²로 환산해 적분한다
        total_inertia = joint_load_inertia(motor) + MAXON_REFLECTED_INERTIA
        accel = output_torque / total_inertia * RAD_TO_DEG

        velocity += accel * dt
        velocity = max(-MAXON_SPEED_LIMIT, min(MAXON_SPEED_LIMIT, velocity))
        self.joint_velocity[motor] = velocity

        return joint_deg + velocity * dt

    def _tmotor_velocity(self, motor: str, velocity_erpm: float) -> float:
        spec = TMOTOR_SPEC.get(motor)
        if spec is None:
            return 0.0

        pole = spec["pole"]
        gear_ratio = spec["gear_ratio"]
        joint_rad_s = velocity_erpm * 2.0 * 3.141592653589793 / (pole * gear_ratio * 60.0)
        return joint_rad_s * spec["cw_dir"] * 180.0 / 3.141592653589793

    def _maxon_velocity(self, motor: str, velocity_enc: float) -> float:
        spec = MAXON_SPEC.get(motor)
        if spec is None:
            return 0.0

        gear_ratio = spec["gear_ratio"]
        motor_deg_s = velocity_enc * 360.0 / (gear_ratio * 4096.0)
        return motor_deg_s * spec["cw_dir"]

    # DXL 명령 분배
    def route_dxl(self, command: DxlCommand) -> Optional[Dict[str, float]]:
        motor = DXL_MOTORS.get(command.dxl_id)
        if motor is None:
            return None

        if command.kind == "sync_write" and command.address == 108:
            goal_deg = dxl_goal_deg(command.data)
            if goal_deg is None:
                return None
            self.dxl_target[motor] = goal_deg
            return dxl_to_urdf_deg(motor, goal_deg)

        return None
