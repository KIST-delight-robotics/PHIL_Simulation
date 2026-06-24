# Change Log

## 2026-06-23
- 11:27 KST (UTC+9) — [조사 종료/revert] "SIL에서만 목이 팔보다 먼저 까닥" 장기 조사 마무리, 실험 코드 전부 되돌림
  - 수정 파일: (revert) `simul.py`, `sil/decoder.py` — git checkout으로 HEAD 복원
  - 메모: 결론 = 목 선행은 **명령 스트림에 내재**. `genDxlTrajectory`가 박자/intensity 기반 head nod를 타격 여부와 무관하게 생성해, 인트로에 팔은 ready 유지하는데 목만 먼저 끄덕임. 같은 sim clock 실측에서 목이 팔보다 ~1.1s 선행(팔은 31.8s엔 −89.9° 고정→32.9s부터 시작), 팔은 자기 명령 τ≈0 추종 → 컨트롤러가 그 구간 팔을 붙들고 목을 끄덕인 것(=명령). 앞선 가설들은 모두 실측 반박: (1) syncRead blocking(빼도 동일), (2) velocity 적분 lag(팔 τ≈0), (3) DXL teleport(teleport→프로파일 보간으로 바꿔도 선행 그대로). 단 **하드웨어에선 인트로에 목이 안 까닥임** — 같은 명령인데 다른 이유 미해결(추정: 실서보 부하/데드밴드로 미세 nod가 안 보임). 깔끔한 해결책 없어 보류. 그래서 이번 세션 실험(타임스탬프 디버그 로그 + DXL 모션 모델: `dxl_profile_s`/`dxl_chase`/`dxl_trapezoid`/`_advance_dxl`)을 전부 revert. (`apply_timing.csv` 산출물도 삭제.) 컨트롤러 `syncRead` 변경도 앞서 revert됨.
  - 향후 옵션(빠른 우회): 연주 시작 시 목을 N초 홀드해 팔과 시작점 맞추기. 인트로 길이가 곡마다 달라 근사이며, 필요해지면 컨트롤러(`genDxlTrajectory`/play 시작부)에서 인트로 구간 DXL을 rest로 덮는 식으로 추가.

## 2026-06-22
- 10:42 KST (UTC+9) — head_tilt 매핑 부호 반전 되돌림 (06-19 변경 revert)
  - 수정 파일: `sil/mapping.py`
  - 메모: 06-19에 tilt를 `dxl_deg - 90.0`으로 뒤집었으나, SIL 매핑은 연주/상호작용 두 경로에 공통 적용되므로 잘못된 레이어였다. 연주(ground truth)가 반대로 깨지고 상호작용만 맞아 보이던 현상의 원인. SIL은 `90.0 - dxl_deg`(연주/하드웨어 관례)로 원복하고, 실제 불일치는 controller 쪽 제스처 하드코딩 숫자에서 잡았다(`DrumRobot2/src/AgentAction.cpp`). `dxl_to_urdf_deg`/`urdf_to_dxl_deg` 모두 `90.0 - x`로 환원.

## 2026-06-19
- 14:52 KST (UTC+9) — torque 모드에 PyBullet 실제 동역학 경로 추가 (플래그로 선택, 기본 off)
  - 수정 파일: `sil/router.py`, `sil/mapping.py`, `sil/pybullet_backend.py`, `simul.py`
  - 메모: `router.TORQUE_PHYSICS` 플래그 추가. False(기본)면 기존 `_advance_torque` 1D 손적분 + resetJointState 그대로라 동작 불변. True면 router는 torque 관절을 적분하지 않고 `torque_targets()`로 출력단 토크(Nm, stall→기어×효율→peak clamp, URDF 부호 적용)만 만들고, `PyBulletBackend.apply_joint_torques()`가 `TORQUE_CONTROL`로 인가→`stepSimulation`이 물리 적분. backend는 첫 torque 명령 때 `_enable_torque_joint`로 기본 속도모터 끄고(force=0) datasheet 반사 관성(`MAXON_REFLECTED_INERTIA`)을 link Izz에 더함(Bullet엔 armature 칸 없음). URDF link 관성은 PyBullet이 이미 보유하므로 부하 관성은 따로 안 넣음. 무부하 전류 기반 마찰은 측정 속도 반대로 차감. 피드백은 기존 `getJointState`(SYNC TPDO) 경로 그대로라 자동 반영. 물리 모드는 `simul._step_torque_physics`가 벽시계 누산기로 고정 timestep(1/240) substep을 돌려 실시간에 맞춤(누산 상한 MAX_DT).
  - 주의: 물리 모드 미검증. wrist는 Maxon이라 feedback이 main loop step 타이밍에 묶임 → 풀 부하에서 safety trip 재발 가능. 라이브로 timestep/스텝 cadence 튜닝 필요. 반사 관성 Izz 합산은 wrist 회전축이 link Z라는 가정의 근사.
- 14:16 KST (UTC+9) — Maxon torque 모드 동특성을 datasheet 기반으로 재작성
  - 수정 파일: `sil/router.py`, `sil/mapping.py`
  - 메모: 기존 `_advance_torque`는 `MAXON_TORQUE_GAIN=3.0`/`MAXON_TORQUE_DAMPING=8.0`/`MAXON_VELOCITY_LIMIT=720` 같은 근거 없는 임의 상수로 1차 적분하던 가짜 모델이었다. `docs/Maxon_wrist_motor.pdf`(DCX22L GB KL 48V + GPX22HP 35:1 + ENX16 1024)에서 토크 상수 45.2 mNm/A, stall 294 mNm, 무부하 전류 16.2 mA, 무부하 속도 10100 rpm, 로터 관성 8.85 gcm², 기어 효율 75%, 기어 순간 토크 3 Nm, 기어 관성 1.31 gcm²를 뽑아 router 상수로 대체. 모델: 모터축 토크를 stall로 clamp → 기어비×효율로 출력단 토크 환산 후 3 Nm로 clamp → 무부하 전류 기반 마찰 차감 → `accel = T_net / J_total`로 적분, 속도는 무부하 속도 출력단 환산값(1731 deg/s)으로 clamp. 부하 관성은 datasheet에 없으므로 `mapping.joint_load_inertia()`가 URDF link inertia를 평행축 정리로 관절축 기준으로 환산해 가져온다(wrist=1.444e-3 kg·m², URDF). pedal joint는 이 URDF에 없어 foot는 `DEFAULT_LOAD_INERTIA` fallback. `MAX_DT`는 적분 안정용이라 유지.
- 11:44 KST (UTC+9) — `gc.disable()` 되돌림(freeze는 유지)
  - 수정 파일: `simul.py`
  - 메모: RT hardening + responder thread로도 SIL safety 트립이 완전히 사라지지 않아(잔여는 비-RT 커널 deschedule + echo의 구조적 ≥1 step 지연), 메모리 누수 위험만 있고 효과가 불확실한 `gc.disable()`을 제거. `gc.collect()`+`gc.freeze()`는 누수 위험 없이 GC 스캔 부담만 줄이므로 유지. `sys.setswitchinterval`/`SCHED_FIFO`/`mlockall`은 무해/저위험이라 유지. 트립 자체의 최종 종결은 컨트롤러측 vcan 게이트 완화(B)로 예정.

## 2026-06-18
- 17:29 KST (UTC+9) — SIL feedback 멈춤 원인(GIL/GC/deschedule)에 best-effort 실시간 hardening 추가
  - 수정 파일: `simul.py`
  - 메모: responder thread만으로 못 막는 잔여 트립(프로세스 deschedule·GC·GIL 경합)을 줄이기 위해 `run()` warmup 직후 `_apply_realtime()`, loop 진입 전 `_freeze_gc()` 추가. GIL: `sys.setswitchinterval(0.0005)`(5ms→0.5ms)로 echo thread가 GIL을 더 빨리 넘겨받게. 우선순위: `os.sched_setscheduler(SCHED_FIFO, 10)`(실패 시 `nice(-10)` 폴백) + `mlockall`로 deschedule/page fault 멈춤 차단. RT 정책은 PyBullet 내부 thread 생성 이후·우리 thread 생성 이전에 걸어 echo/dxl thread만 정책을 상속. GC: warmup 후 `gc.freeze()`+`gc.disable()`로 stop-the-world 제거(ref counting 유지). 모두 best-effort라 권한 없으면 조용히 skip. SCHED_FIFO/mlockall은 root 필요 → simul.py를 sudo로 실행해야 실제 적용됨.
  - 주의: `gc.disable()`로 순환 참조 누수 가능 → 장시간 실행 시 메모리 증가 관찰 필요. 비-RT 커널이라 deschedule을 완전히 없애진 못함(트립이 남으면 컨트롤러측 (A)/(B) 게이트 완화로 마무리).
- 17:05 KST (UTC+9) — TMotor recv+echo를 전용 responder thread로 분리해 SIL safety current 신선도 개선
  - 수정 파일: `simul.py`
  - 메모: `safetyCheckSendT`가 SIL에서 간헐 트립하던 원인은 main loop가 PyBullet `stepSimulation`에 막혀 TMotor echo feedback이 지연→DrumRobot2 current가 stale해지는 것. TMotor 전용 버스(vcan0/vcan1)의 recv+echo를 step에 안 막히는 별도 thread(`_tmotor_loop`)로 분리. 명령은 즉시 echo하고 PyBullet 반영은 staging→main loop에서 처리. heartbeat feedback source는 `router.motor_target` 하나로 통일(position/velocity/discovery 공통). Maxon 버스(vcan2/vcan3)와 SYNC TPDO는 PyBullet 읽기 때문에 main loop에 그대로 둠. 라우터가 stateful이라 `route_can`/`advance`/`motor_target` 접근에 `router_lock`을 둠(GIL 위 sub-µs 임계구역이라 지연 영향 무시 가능, dict iterate 중 mutate 크래시 방지가 목적). 기존 `_send_tmotor_idle_feedback`/`last_tmotor_command` 제거.
  - 후속: brain까지 붙인 풀 부하에서 R_arm 계열 트립이 사라지는지 확인. 안 되면 컨트롤러측 (A)/(B) 게이트 완화 검토.

## 2026-06-16
- 15:00 KST (UTC+9) — 레포 분할 준비: 인터페이스 계약 단일 소스와 분리-레포 헤더 추가
  - 수정 파일: 신규 `CONTRACTS.md` / 수정 `AGENTS.md`
  - 메모: 이 레포(`phil-sil`)의 독립 로그를 0에서 시작. 이전 통합 로그는 옮기지 않는다.
