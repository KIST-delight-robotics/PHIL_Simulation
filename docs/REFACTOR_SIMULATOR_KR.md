# `simul.py` 분할 설계 (FrameSimulator 책임 분리)

> 상태: **설계 단계** (코드 미변경). 실제 리팩터링 전 합의용 문서.
> 대상: `Drum_intheloop/simul.py`의 `FrameSimulator`(약 572줄).

## 1. 왜 손대려는가

`FrameSimulator` 한 클래스가 아래를 전부 들고 있어 **스레드와 역할이 코드에서 안 보인다.**

- 오케스트레이션 (생명주기, RT/GC hardening, apply/step 루프)
- 장치 transport (vcan 버스 open/close, DXL PTY open/close/read/write)
- 3개 장치 responder (TMotor / Maxon / DXL) 프로토콜·feedback 로직
- motion advance + physics step

특히 **한 장치의 책임이 두 스레드에 쪼개져 있다.** 예: TMotor는 자기 스레드에서
recv/echo하지만, 받은 target의 backend 적용은 main 스레드(`_apply_tmotor_targets`)가 한다.
그래서 "TMotor 담당이 어디냐"가 한 곳에 모이지 않는다.

## 2. 현재 스레드 모델 (있는 그대로)

스레드는 3개다.

| 스레드 | 책임 | 현재 메서드 |
|---|---|---|
| **main loop** (`run`) | Maxon CAN 처리 · staged target을 backend 적용 · motion advance · physics step | `_poll_can`, `_apply_dxl_targets`, `_apply_tmotor_targets`, `_advance_motion`, `_step_torque_physics`, `_send_maxon_sync_feedback` |
| **DXL thread** (`_dxl_loop`) | PTY recv → ping/write/read 응답 → goal staging | `_poll_dxl`, `_handle_dxl`, `_stage_dxl_target`, `_write_dxl` |
| **TMotor thread** (`_tmotor_loop`) | vcan0/1 recv → 즉시 echo → 200Hz heartbeat → target staging | `_drain_tmotor_buses`, `_emit_tmotor_feedback` |

```text
                         ┌──────────────── main loop (run) ────────────────┐
   vcan2/3 (Maxon) ────▶ │ _poll_can ─ NMT/SDO/SYNC 처리, SYNC→TPDO feedback │
                         │ _apply_dxl_targets   ◀── (dxl_lock) dxl_targets   │
                         │ _apply_tmotor_targets◀── (tmotor_lock) stage      │ ──▶ backend.apply/step
                         │ _advance_motion      ── (router_lock) router      │
                         │ _step_torque_physics                              │
                         └───────────────────────────────────────────────────┘
                                  ▲ stage                    ▲ stage
   vcan0/1 (TMotor) ──▶ TMotor thread (recv+echo+heartbeat)  │
   /dev/ttyUSB0 PTY ──▶ DXL thread (recv+ping/write/read echo)
```

## 3. 숨어 있는 핵심 불변식 (문서화되지 않음)

코드를 추적하면 이미 지켜지지만 **어디에도 적혀 있지 않은** 규칙이 있다.

> **`backend`(`apply_targets` / `step` / `read_joint_states`)는 오직 main 스레드만 건드린다.**
> worker 스레드(DXL/TMotor)는 자기 bus의 I/O(recv/echo)와 decode/encode만 하고,
> 결과 target을 lock으로 보호된 dict에 **staging만** 한다. backend는 절대 직접 안 만진다.

이 불변식이 전체 스레드 안전성의 토대다. 지금은 `router_lock` / `dxl_lock` /
`tmotor_lock` 3개의 ad-hoc lock+dict 쌍으로만 암묵적으로 표현돼 있다.
이걸 1급 객체(`TargetSink`)로 만들면 규칙이 타입으로 드러난다.

추가로 feedback 경로의 비대칭도 명시해 둘 가치가 있다.

- TMotor: command echo + 200Hz heartbeat (worker 스레드, backend 안 읽음 — `router.motor_target` 사용)
- Maxon: SYNC(0x80) 받을 때만 TPDO (main 스레드, `backend.read_joint_states()` 사용)
- DXL: syncRead 받을 때만 status (worker 스레드, backend 안 읽음 — `dxl_feedback` dict 사용)

→ backend state를 읽는 feedback은 **Maxon 하나뿐**이고, 그래서 main 스레드에 있다.
나머지 둘은 자체 보관 값으로 응답하므로 worker 스레드에서 즉답이 가능하다.

## 4. 제안 구조

장치 responder 축으로 자르고, staging 경계를 명시 타입으로 만든다.

```text
sil/
  decoder.py / encoder.py / router.py / mapping.py    # (기존 유지)
  pybullet_backend.py / visuals.py / urdf_tools.py    # (기존 유지)
  motor_state.py                                      # (기존 유지)

  target_sink.py    # [신규] responder가 push, main이 drain하는 thread-safe 경계.
                    #        lock+dict 3쌍을 대체. 불변식을 코드로 강제.
  runtime.py        # [신규] RT/GC hardening (_apply_realtime + _freeze_gc)

  transport/
    can_bus.py      # [신규] vcan 버스 open/close, bus_map, motor↔bus 바인딩
    dxl_pty.py      # [신규] PTY open/close, framed read/write (_write_dxl, split_dxl_packets)

  responders/
    base.py         # [신규] Responder 인터페이스: 스레드 1개 소유, start/stop/join
    tmotor.py       # [신규] TMotorResponder: drain+echo+heartbeat → sink push
    maxon.py        # [신규] MaxonResponder: NMT/SDO ack, SYNC→TPDO (backend read 필요)
    dxl.py          # [신규] DxlResponder: ping/write/syncRead echo → sink push

simul.py            # [얇게] args 파싱 → transport/responder/backend 와이어링
                    #        → "sink drain → backend apply → step" 루프만
```

설계 원칙: **파일 1개 = responder 1개 = 스레드 1개.** 스레드 생성/join은 `base.py`에 모은다.

### Maxon의 위치 결정 포인트

Maxon만 feedback에 `backend.read_joint_states()`가 필요하다(불변식상 main 전용 자원).
두 선택지:

- **(a) Maxon은 main 루프가 직접 호출** (현행 유지): `MaxonResponder`는 순수 로직만 갖고
  스레드를 안 만든다. main이 `_poll_can` 자리에서 `maxon.poll(bus)`를 호출.
- **(b) Maxon도 스레드 소유**: backend 읽기를 sink처럼 "요청 큐"로 빼서 main이 대신 읽어줌.

→ 권장은 **(a)**. Maxon feedback이 step 타이밍에 묶이는 건 실제 Maxon(SYNC 동기)과
   같은 성질이고, 괜히 backend 읽기 채널을 추가하면 불변식이 복잡해진다.
   "responder = 스레드"는 TMotor/DXL에만 적용하고 Maxon은 main-driven으로 남긴다.

## 5. 현재 → 제안 매핑

| 현재 (simul.py) | 이동 위치 |
|---|---|
| `run`, `close` 루프 골격 | `simul.py` (얇게) |
| `_apply_realtime`, `_freeze_gc` | `sil/runtime.py` |
| `_open_can_buses`, `bus_map`, `motor_bus` | `sil/transport/can_bus.py` |
| `_open_dxl`, `_write_dxl`, PTY 버퍼 | `sil/transport/dxl_pty.py` |
| `_drain_tmotor_buses`, `_emit_tmotor_feedback` | `sil/responders/tmotor.py` |
| `_poll_can`(Maxon 부분), `_send_maxon_sync_feedback`, `nmt_state` | `sil/responders/maxon.py` |
| `_dxl_loop`, `_poll_dxl`, `_handle_dxl`, `_stage_dxl_target` | `sil/responders/dxl.py` |
| `dxl_targets`/`tmotor_stage` + 각 lock | `sil/target_sink.py` (통합) |
| `_apply_dxl_targets`, `_apply_tmotor_targets`, `_advance_motion`, `_step_torque_physics` | `simul.py` 루프 (sink drain + step) |
| `_send_motor_feedback` | responder가 공유 (transport 통해 send) |

## 6. 제안 스레드 모델 (분할 후, 동작 동일)

```text
TMotorResponder.thread ─ recv+echo+heartbeat ─▶ TargetSink.push()
DxlResponder.thread    ─ recv+ping/write/read ─▶ TargetSink.push()
                                                      │ drain
simul main loop:  sink.drain() ─▶ backend.apply ─▶ maxon.poll() ─▶ router.advance ─▶ step
```

스레드 개수·역할·timing은 그대로 두고 **소유권만 명확히** 한다(동작 불변 리팩터).

## 7. 제안 이행 순서 (작게, 동작 보존)

각 단계 후 `python3 -m compileall`과 라이브 SIL로 동작 동일성 확인.

1. `runtime.py` 추출 (RT/GC) — 가장 독립적, 위험 최소.
2. `transport/can_bus.py`, `transport/dxl_pty.py` 추출 — I/O 경계.
3. `target_sink.py` 도입 — lock+dict 3쌍을 1개 타입으로. 불변식 코드화.
4. `responders/{tmotor,dxl}.py` 추출 — 스레드 소유권 이동.
5. `responders/maxon.py` 추출 (main-driven 유지).
6. `simul.py`를 와이어링 + 루프만 남기게 정리.

## 8. 비목표 (이번 분할에서 안 건드림)

- 프로토콜 wire format (`decoder`/`encoder`)·CAN ID·각도 변환 — **Contract B**, 손대지 않음.
- feedback timing 정책 변경 (TMotor echo/heartbeat, Maxon SYNC, DXL syncRead 모델 유지).
- torque physics 모델 (`router.TORQUE_PHYSICS` 경로 그대로).
- 동작 변경은 없음. 순수 구조 리팩터.
