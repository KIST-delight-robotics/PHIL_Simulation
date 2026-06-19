import atexit
import math
from pathlib import Path
import shutil
import tempfile
from typing import Dict, Optional, Set

import pybullet as p

from .mapping import PEDAL_JOINTS
from .urdf_tools import build_runtime_urdf
from .visuals import (
    add_drum_pads,
    apply_robot_theme,
    create_pedals,
    setup_world,
    tilt_pedal,
)

PHYSICS_TIMESTEP = 1.0 / 240.0  # s, torque 물리 모드 고정 적분 timestep


class PyBulletBackend:
    def __init__(
        self,
        urdf_path: Path,
        mode: str = "gui",
        torque_physics: bool = False,
        reflected_inertia: float = 0.0,
        friction_torque: float = 0.0,
    ):
        self._source_urdf_path = urdf_path.resolve()
        self._mode = mode
        self._client_id: Optional[int] = None
        self._robot_id: Optional[int] = None
        self._patched_dir: Optional[str] = None
        self._joint_index_by_name: Dict[str, int] = {}
        self._pedal_ids: Dict[str, int] = {}
        # torque 물리 모드 설정
        self._torque_physics = torque_physics
        self._reflected_inertia = reflected_inertia
        self._friction_torque = friction_torque
        self._torque_joints: Set[str] = set()

    # Lifecycle
    def start(self) -> None:
        client_id = p.connect(p.GUI if self._mode == "gui" else p.DIRECT)
        setup_world(client_id, self._mode)

        patched_urdf_path = self._build_runtime_urdf()
        robot_id = self._load_robot(client_id, patched_urdf_path)

        self._client_id = client_id
        self._robot_id = robot_id
        self._joint_index_by_name = self._read_joint_indices()
        self._place_robot_on_ground()
        self._pedal_ids = create_pedals(client_id)
        apply_robot_theme(client_id, robot_id)
        add_drum_pads(client_id)

        if self._torque_physics:
            # 물리 적분은 고정 timestep이 안정적이다. 실시간 보조는 simul loop가 맞춘다.
            p.setTimeStep(PHYSICS_TIMESTEP, physicsClientId=client_id)

    def close(self) -> None:
        if self._client_id is not None and p.isConnected(self._client_id):
            p.disconnect(physicsClientId=self._client_id)

        if self._patched_dir:
            shutil.rmtree(self._patched_dir, ignore_errors=True)

    def step(self) -> None:
        if self._client_id is None:
            return

        p.stepSimulation(physicsClientId=self._client_id)

    # Joint IO
    def read_joint_states(self) -> Dict[str, float]:
        if self._robot_id is None or self._client_id is None:
            return {}

        result: Dict[str, float] = {}
        for joint_name, joint_index in self._joint_index_by_name.items():
            state = p.getJointState(
                self._robot_id,
                joint_index,
                physicsClientId=self._client_id,
            )
            result[joint_name] = math.degrees(state[0])
        return result

    def apply_targets(self, joint_targets_deg: Dict[str, float]) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        for joint_name, target_deg in joint_targets_deg.items():
            side = PEDAL_JOINTS.get(joint_name)
            if side is not None:
                tilt_pedal(self._client_id, self._pedal_ids.get(side), side, target_deg)
                continue

            joint_index = self._joint_index_by_name.get(joint_name)
            if joint_index is not None:
                self._set_joint_position_deg(joint_index, target_deg)

    # Robot loading
    def _load_robot(self, client_id: int, urdf_path: Path) -> int:
        base_orn = p.getQuaternionFromEuler((-math.pi / 2.0, 0.0, 0.0))
        return p.loadURDF(
            str(urdf_path),
            baseOrientation=base_orn,
            useFixedBase=True,
            physicsClientId=client_id,
        )

    def _build_runtime_urdf(self) -> Path:
        patched_dir = tempfile.mkdtemp(prefix="drum_intheloop_urdf_")
        atexit.register(lambda: shutil.rmtree(patched_dir, ignore_errors=True))

        self._patched_dir = patched_dir
        return build_runtime_urdf(self._source_urdf_path, Path(patched_dir))

    def _read_joint_indices(self) -> Dict[str, int]:
        if self._robot_id is None or self._client_id is None:
            return {}

        indices: Dict[str, int] = {}
        joint_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for joint_index in range(joint_count):
            joint_info = p.getJointInfo(self._robot_id, joint_index, physicsClientId=self._client_id)
            joint_name = joint_info[1].decode("utf-8")
            indices[joint_name] = joint_index

        return indices

    def _place_robot_on_ground(self, clearance: float = 0.02) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        mins = [float("inf"), float("inf"), float("inf")]
        link_count = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for link_index in range(-1, link_count):
            aabb_min, _ = p.getAABB(self._robot_id, link_index, physicsClientId=self._client_id)
            for axis in range(3):
                mins[axis] = min(mins[axis], aabb_min[axis])

        base_pos, base_orn = p.getBasePositionAndOrientation(
            self._robot_id,
            physicsClientId=self._client_id,
        )
        pos = [
            base_pos[0],
            base_pos[1],
            base_pos[2] + clearance - mins[2],
        ]
        p.resetBasePositionAndOrientation(
            self._robot_id,
            pos,
            base_orn,
            physicsClientId=self._client_id,
        )

    # Torque control (physics 모드)
    def apply_joint_torques(self, joint_torques: Dict[str, float]) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        for joint_name, torque in joint_torques.items():
            joint_index = self._joint_index_by_name.get(joint_name)
            if joint_index is None:
                continue

            if joint_name not in self._torque_joints:
                self._enable_torque_joint(joint_name, joint_index)

            net_torque = self._apply_friction(joint_index, torque)
            p.setJointMotorControl2(
                self._robot_id,
                joint_index,
                controlMode=p.TORQUE_CONTROL,
                force=net_torque,
                physicsClientId=self._client_id,
            )

    def _enable_torque_joint(self, joint_name: str, joint_index: int) -> None:
        # 기본 속도 모터를 꺼야 외부 토크가 관절에 실제로 먹는다.
        p.setJointMotorControl2(
            self._robot_id,
            joint_index,
            controlMode=p.VELOCITY_CONTROL,
            force=0.0,
            physicsClientId=self._client_id,
        )
        # Bullet엔 모터 armature 칸이 없어, datasheet 반사 관성을 link 관성에 더한다.
        # 이 URDF의 torque 관절(wrist)은 회전축이 link Z라 Izz(인덱스 2)에 더한다.
        info = p.getDynamicsInfo(self._robot_id, joint_index, physicsClientId=self._client_id)
        inertia = list(info[2])
        inertia[2] = inertia[2] + self._reflected_inertia
        p.changeDynamics(
            self._robot_id,
            joint_index,
            localInertiaDiagonal=inertia,
            physicsClientId=self._client_id,
        )
        self._torque_joints.add(joint_name)
        print(
            f"[SIL] torque physics ENABLED on {joint_name} "
            f"(reflected_inertia={self._reflected_inertia:.3e} kg·m²)",
            flush=True,
        )

    def _apply_friction(self, joint_index: int, torque: float) -> float:
        # 무부하 전류 기반 Coulomb 마찰을 측정 속도 반대 방향으로 뺀다.
        if self._friction_torque <= 0.0:
            return torque

        state = p.getJointState(self._robot_id, joint_index, physicsClientId=self._client_id)
        velocity = state[1]
        if velocity > 0.0:
            return torque - self._friction_torque
        if velocity < 0.0:
            return torque + self._friction_torque
        return torque

    # Joint writers
    def _set_joint_position_deg(self, joint_index: int, target_deg: float) -> None:
        if self._robot_id is None or self._client_id is None:
            return

        p.resetJointState(
            self._robot_id,
            jointIndex=joint_index,
            targetValue=math.radians(target_deg),
            targetVelocity=0.0,
            physicsClientId=self._client_id,
        )
