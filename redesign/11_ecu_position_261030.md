# 11 — ECU_V3 Position Control 구체안 (2026-10-30)

> 출처: `research/modeling/sim/planning/User/261030_SW_수정계획.md` + 2026-10-30 대화 합의.
> 범위: **`feature/ecu_ctr` 브랜치, `stm_ws/ECU_V3` 만.** Orin·WEB 은 §9 "Orin 계약" 만 적고
> 상세는 후속 문서(12 Orin / 13 WEB)에서 다룬다.
> ⚠ STM 빌드는 이 머신에서 불가 — 구현 후 STM32CubeIDE 빌드 검증 필요.

---

## 0. 목표와 확정 결정

**목표**: Orin 은 200Hz 로 "지금 위치 cmd" 만 보낸다. AK 피드백 범위(±3200°)를 넘는
연속 이동은 **ECU 가 스스로 원점을 옮기고(set origin) 명령을 보정**해 루프가 한 tick 도 끊기지 않게 한다.

| # | 결정 | 근거 |
|---|------|------|
| D1 | 전용 inst `PACKET_INST_POS (0x05)` — cmd_position write + **고정 FB 블록** 응답 | 기존 RW 와 같은 1-트랜잭션, read 세그 협상 불필요 |
| D2 | FB 블록은 **addr 256 이후 신규 페이지(PAGE1)** — 기존 0~255 주소 불변 | 기존 Orin 프리셋·도구 무손상, 1회 memcpy = 원자 스냅샷 |
| D3 | 원점은 **ECU 자율**: \|fb\| ≥ 3000° 에서 `pre_origin_pose` 래치 + SET_ORIGIN(temporary) | Orin 은 set origin 시점을 알 수 없다 |
| D4 | old-frame 판정 기준은 **fb 가 아니라 `last_tx`** (ECU 가 직전에 모터로 보낸 cmd) | fb 는 CAN 피드백 주기만큼 늦다 → 레이스 |
| D5 | Orin 의 offset 반영 트리거는 **`origin_seq` 변화** (flag 아님) | flag 는 응답 타이밍상 2회 이상 보일 수 있다 → 이중 적용 방지 |
| D6 | S(원점 이동량) = **fb_old** (1단계). 진단 3종을 FB 블록에 실어 실측 후 2단계(외삽) 판단 | 합의 (2026-10-30) |
| D7 | 보정 flag 는 **Data[0] 에 넣지 않는다** — FB 블록 `origin_flag` | Data[0]≠0 은 Orin 이 write 거부로 세어 50 tick 후 LOCKED |
| D8 | I2C(AS5600) 는 `#if RD_USE_I2C_ENCODER` 로 **컴파일 제외 가능** | 합의. 기본값 1 (project 모드는 사용) |
| D9 | realtime_tick·로드셀 FB 포함 | 합의 |
| D11 | 원점 FSM 은 **INST_POS 수신 중에만** 동작 — RW 로 보내는 기존 POSITION(`auto_mode=5`)·`OP_SET_ORIGIN` 은 지금과 동일 | 기존 경로는 `origin_seq` 를 읽지 않아 자동 원점을 따라갈 수 없다 (2026-11-05) |
| D10 | linear encoder 는 **MUX ch3(`enc[3]`, 4번 채널)의 커스텀 보드** — `'3'` = x·y int32 8B, `'i'` = 0 초기화 (§6.1) | 2026-11-04 패킷 확정 |

---

## 1. 변경 파일 요약

| 파일 | 변경 |
|------|------|
| `Core/Inc/rd_define.h` | `RD_USE_I2C_ENCODER` 스위치 |
| `Core/Inc/rd_register_ecu.h` | PAGE1 구조체 `PAGE_POS_FB_t` + offset 상수, `REG_TOTAL_SIZE` 256→344, `REGISTER_t` 확장, `lin_zero_req`(addr 193) — CMD_SYSTEM 13→14B, RSVD1 31→30B |
| `Core/Inc/rd_comm_rls.h` / `Src/rd_comm_rls.c` | `RLS_CUST` 모델 + `RLS_CMD_CUST_e` + `RLS_ReadCust` 파서 |
| `Core/Inc/rd_comm_ecu.h` / `Src/rd_comm_ecu.c` | `PACKET_INST_POS` 핸들러 |
| `Core/Src/rd_map_ecu.c` | LUT 에 PAGE1 영역, MARSHAL_PUBLISH 에 PAGE1 발행 |
| **`Core/Inc/rd_origin.h` / `Src/rd_origin.c` (신규)** | 원점 FSM — controlTask 소유 |
| `Core/Src/rd_can_motor.c` / `.h` | `RD_CAN_MOTOR_TRANSMIT(cmd, mask, origin_mask)` — SET_ORIGIN 선행 프레임 |
| `Core/Src/rd_peripheral_ecu.c` / `.h` | `RD_PERIPHERAL_WRITE` 에 origin_mask 전달, I2C init ifdef |
| `Core/Src/rd_system.c` | controlTask 에 `RD_ORIGIN_UPDATE` 삽입, RLS 채널 혼합 INIT, UART4 Checker/Recovery, I2C ifdef |
| `Core/Inc/rd_mux_rls.h` / `Src/rd_mux_rls.c` | 채널별 모델·명령, `RLS_NUM` 2→4, 무응답 시 `error=1` 표시, ch3 `'i'` 1회 송신 |
| `ECU_V3.ioc` (+ 재생성 `main.c`) | **CAN1 Transmit FIFO Priority = ENABLE** (§5.4) |

---

## 2. 레지스터 — PAGE1 `PAGE_POS_FB_t` (addr 256~345, 90B, R/O)

> 2026-11-08: ch2(θ₃)를 Orbis → AksIM 으로 교체하며 `rls_theta3` 을 u16 → u32 로 넓혔다. 326 이후 필드가 2B 씩 밀렸고 총 크기 344 → 346.

전 필드 little-endian, packed. 원자 스냅샷: MARSHAL_PUBLISH 의 단일 CRIT 안에서 채우고
INST_POS 응답은 DISPATCH_READ 1회(CRIT 1회)로 복사한다.

| addr | 타입     | 필드                 | 단위/의미                                                                                      | 출처                    |
| ---: | ------ | ------------------ | ------------------------------------------------------------------------------------------ | --------------------- |
|  256 | u32    | `realtime_tick`    | ×0.1ms, TIM5 (= reg.sys 와 같은 값)                                                            | PUBLISH 의 now         |
|  260 | u8     | `rs485_proc_delta` | ×0.1ms (시계 동기용, 다음 응답에 실림 — 기존 규약 동일)                                                      | rs485Task             |
|  261 | u8     | `sys_state`        | SYSTEM_STATE_e                                                                             | robot_state           |
|  262 | u8     | `hw_reset`         | HARDWARE_STATUS_t bitfield                                                                 | hw.reset              |
|  263 | u8     | `origin_flag`      | bit0~3 = ORIGIN_PEND(M1~4, ECU 보정 중) / bit4~7 = FRAME_LOST(M1~4)                           | rd_origin             |
|  264 | u8[8]  | `degraded_cnt`     | 0~100%. idx0 uart1 / 1 uart2 / 2 uart6 / 3 can1 / 4 i2c1 / 5 로드셀 예약 / **6 uart4(RLS, 신규)** / 7 RSVD | 기존 deg[] + idx6       |
|  272 | i16[4] | `mtr_position`     | ×0.1° — **모터 현재 프레임(원점 이후 좌표)**                                                            | ECU_AK[i].state       |
|  280 | i16[4] | `mtr_velocity`     | ×10 ERPM (AK raw 그대로)                                                                      | 〃                     |
|  288 | i16[4] | `mtr_current`      | ×0.01A                                                                                     | 〃                     |
|  296 | i16[4] | `pre_origin_pose`  | ×0.1° — 모터별 **마지막 확정 S**                                                                   | rd_origin             |
|  304 | u8[4]  | `origin_seq`       | 모터별 확정 원점 이벤트 카운터 (부팅 0, mod 256)                                                          | rd_origin             |
|  308 | u16[2] | `loadcell_avg`     | raw 0~4095. **ch0 = 지면 총 접선력 / ch1 = 윗벽 y힘** (물리변환은 Orin)                                  | loadcell.raw_avg      |
|  312 | u8     | `loadcell_state`   | STATE_t                                                                                    | loadcell.state        |
|  313 | u8     | `motor_state`      | STATE_t (4모터 worst)                                                                        | err.can.state         |
|  314 | u8     | `rls_state`        | STATE_t (UART4 채널)                                                                         | ECU_uart4.error.state |
|  315 | u8     | `rls_ew`           | 2bit×4 (b1=Error, b0=Warning, active-high 로 저장), LSB=ch0. ch3 은 Error 만 사용 (통신 실패)         | enc[i].error/warning  |
|  316 | u32    | `rls_psi1`         | AksIM raw (18bit, 0..2¹⁸−1)                                                                | enc[0].position       |
|  320 | u32    | `rls_psi2`         | AksIM raw (18bit)                                                                          | enc[1].position       |
|  324 | u32    | `rls_theta3`       | AksIM raw (18bit) — 2026-11-08 Orbis u16 에서 교체                                                                        | enc[2].position       |
|  328 | u8     | `lin_zero_result`  | 마지막 `'i'` 결과: 0=없음 / 1=성공 / 2=보드 실패 / 3=무응답 / 4=형식 불일치 / 5=잘못된 요청 (§6.1)                 | enc[3].zero_result    |
|  329 | u8     | `lin_zero_seq`     | `'i'` 완료 카운터 (부팅 0, mod 256) — 값이 바뀌면 `lin_zero_result` 가 새 결과                            | enc[3].zero_seq       |
|  330 | i32    | `lin_x`            | linear encoder x [count, 물리 환산은 Orin]. 실패 시 직전 값 유지 + rls_ew ch3 Error=1                 | enc[3].lin_x          |
|  334 | i32    | `lin_y`            | linear encoder y [count]. 〃                                                                 | enc[3].lin_y          |
|  338 | u8     | `odiag_motor`      | 마지막 원점 이벤트 모터 idx (0~3, 0xFF=없음)                                                           | rd_origin             |
|  339 | u8     | `odiag_dt`         | ×0.1ms: 원점 TX 시각 − fb_old 취득 시각 (sat 255)                                                  | 〃                     |
|  340 | i16    | `odiag_e_track`    | ×0.1°: `last_tx_old − fb_old` (추종 오차)                                                      | 〃                     |
|  342 | i16    | `odiag_residual`   | ×0.1°: 사후 검증 오차 (§5.5)                                                                     | 〃                     |
|  344 | i16    | `odiag_vel`        | ×10 ERPM: 원점 시점 속도 raw                                                                     | 〃                     |

### 2.1 규칙
- **미판독 ≠ 0** (CLAUDE_ECU 규칙): RLS 채널 무응답·파싱 실패면 `rls_ew` 해당 ch Error=1 + 값 유지.
  부팅 후 첫 성공 전까지도 Error=1 이다 (linear 의 0 은 "초기화된 위치"일 수 있어 값으로 구분 불가).
  delta_tick 은 PAGE1 에 넣지 않는다 (합의: tick 불필요).
- 기존 base map 은 **주소·의미 불변**. 단 `reg.sys.degraded_cnt[6]` 에도 uart4 를 채운다(같은 배열).
- `REG_TOTAL_SIZE` 346 (2026-11-08, 종전 344). `check_region_range` 의 상한·LUT 오름차순·빈틈 없음 가정은 PAGE1 을
  마지막 엔트리로 붙이면 그대로 성립. 기존 READ inst 로도 PAGE1 디버그 판독 가능.
- 응답 크기: Data[0] 1B + 90B = 91B ≤ `PACKET_DATA_BUF_SIZE`(256). 와이어 ~97B ≈ 1.05ms @921600.

### 2.2 base map 추가 — `lin_zero_req` (addr 193, u8, R/W)
- linear 0 초기화 요청. Orin 이 1 을 쓰면 encoderTask 가 다음 스윕에서 CRIT 로 읽고 0 으로 지운 뒤 `'i'` 를 **1회만** 보낸다.
  **재시도하지 않는다** — 엣지 명령을 재시도하면 초기화가 두 번 실행될 수 있다 (rd_control.hpp 펄스 규약과 같은 이유).
- 그릇은 RSVD1 앞 1B 인출 (addr 192 `motor_mask` 때와 같은 방식): `REG_CMD_SYSTEM_SIZE` 13→14, `REG_RSVD1_OFFSET` 193→194, `REG_RSVD1_SIZE` 31→30.
- `cmd_write_tick` 워치독 범위(132~187) 밖이고 모터 lock 대상도 아니다 → MANUAL 에서도 초기화 가능.
- Orin 미러(`orin_ws/.../core/rd_register_ecu.hpp`)도 같이 바꿔야 한다 (12 문서).

---

## 3. `PACKET_INST_POS (0x05)` 포맷

```
요청 Param = [AddrL|H] + cmd_position data (4B × n)
             addr ∈ {132,136,140,144}, n ≥ 1, addr + 4n ≤ 148     (cmd_position[] 부분집합)
응답 Data  = [0] err = (write_err << 4) | read_err   ← RW 와 동일 니블 규약
             [1..88] PAGE1 전체 (read_err == NONE 일 때)
```

처리 순서 (`RD_PACKET_HANDLE` case):
1. **write 검증** — 위 범위·정렬 위반 → `PACKET_ERR_DATA_LEN`. `reg.cmd_system.auto_mode != 5(POSITION)`
   → `PACKET_ERR_ACCESS` (POSITION 이 아닌데 위치를 쓰면 조용히 무시되는 것을 막는다).
2. `RD_MAP_DISPATCH_WRITE(addr, 4n, data, lock)` — 기존 lock(AUTO 만 허용)·`cmd_write_tick` 워치독 그대로.
3. **read 는 write 결과와 무관하게 항상** `RD_MAP_DISPATCH_READ(256, 88, &Data[1])`.
4. 활성 모터 수(2/4)는 FW 가 모른다 — Orin 이 addr/n 으로 정한다.
5. write 가 성공하면 `pos_inst_tick = osKernelGetTickCount()` 기록 → 원점 FSM 활성 판정에 쓴다 (D11).

`reg.cmd_motor.cmd_position` 에는 **Orin 이 보낸 값 그대로** 저장한다 (보정은 controlTask 가
`cmd_mtr` 사본에서만 한다 → reg 는 "Orin 이 보낸 것", `last_tx` 는 "모터로 나간 것").

---

## 4. 태스크·소유권

| 데이터 | 생산(쓰기) | 소비 |
|--------|-----------|------|
| `reg.cmd_motor.cmd_position` | rs485Task (INST_POS write) | controlTask CONSUME |
| `ORIGIN[i]` (rd_origin 내부 상태) | **controlTask 단독** | MARSHAL_PUBLISH (CRIT 스냅샷) |
| SET_ORIGIN / POSITION CAN 프레임 | controlTask → canTxQueue | can1Task |
| PAGE1 | rs485Task MARSHAL_PUBLISH | INST_POS / READ |
| RLS enc[] (linear 포함) | encoderTask (2ms) | PUBLISH, systemTask Checker |
| `reg.cmd_system.lin_zero_req` | rs485Task (set) / encoderTask (clear) | encoderTask |

controlTask 파이프라인 (변경 후):
```
CONSUME → CONTROL_UPDATE(LPF) → RD_ORIGIN_UPDATE(&cmd_mtr, tx_mask, &origin_mask) → PERIPHERAL_WRITE(mask, origin_mask)
```

---

## 5. 원점 FSM (`rd_origin.c`, controlTask 200Hz)

### 5.1 상수 (`rd_origin.h`)
| 상수 | 값 | 의미 |
|------|----|------|
| `ORIGIN_TH_RAW` | 30000 | \|fb\| ≥ 3000° 에서 원점 트리거 (AK fb 한계 3200° 까지 200° 여유) |
| `ORIGIN_JUMP_TH_DEG` | 1000.0f | old-frame / 점프 판정 (S ≥ 3000° 이므로 충분히 분리) |
| `ORIGIN_CONFIRM_MARGIN` | 20 (×0.1ms) | 원점 TX 후 이 시간 이후 수신한 fb 만 "새 프레임" 으로 인정 |
| `ORIGIN_CONFIRM_TO_MS` | 50 | 이 안에 새 프레임 fb 미확인 → 원점 실패 |
| `ORIGIN_SHIFT_OK_RAW` | 15000 | 확인 fb 의 \|fb_new\| < 1500° 이면 원점 적용됨 |
| `ORIGIN_INST_TO_MS` | 100 | 마지막 INST_POS 후 이 시간 안일 때만 FSM 활성 (`AUTO_TIMEOUT` 과 같은 값) |
| `ORIGIN_POS_GAP` | 200 (×0.1ms) | SET_ORIGIN 후 POSITION 을 쉬는 시간 20ms (§5.7) |
| `POS_OVERSPEED_RAW` / `_TICKS` / `_BRAKE_A` | 1008 / 8 / 5.0 | POSITION 과속 제동: 360°/s 가 40ms 이어지면 FRAME_LOST + 5A 제동 |
| `AK_VEL_RAW_TO_DEGPS` | 10·6/168 **잠정 (T1 확정)** | velocity raw → °/s. fb 위치가 출력축 기준이라는 가정. residual 진단에만 쓰고 제어에는 안 쓴다 |
| `SET_ORIGIN_TEMPORARY` | 0 | AK set_origin_mode — **temporary 만 사용** (permanent=flash 기록) |

### 5.2 모터별 상태
```c
typedef struct {
    uint8_t  armed;        /* last_tx 유효 (POSITION 활성 중) */
    float    last_tx;      /* 직전 모터로 나간 cmd [deg, 현재 모터 프레임] */
    uint8_t  pending;      /* ORIGIN_PEND: ECU 가 old-frame cmd 를 보정 중 */
    uint8_t  confirming;   /* 원점 TX 후 새 프레임 fb 대기 */
    uint8_t  frame_lost;   /* FRAME_LOST 래치 */
    uint8_t  shifted;      /* 이 전원 세션에서 temporary 원점이 1회 이상 적용됨 */
    int16_t  S_raw;        /* 진행 중 이벤트의 S (×0.1°) */
    int16_t  S_pub;        /* 확정·발행된 S → pre_origin_pose */
    uint8_t  seq;          /* 확정 카운터 → origin_seq */
    uint32_t t_origin;     /* 원점 TX 시각 [TIM5] */
    uint32_t t_fb_old;     /* fb_old 취득 시각 [TIM5] */
    int16_t  fb_old, vel_old;
} ORIGIN_t;
```

### 5.3 tick 알고리즘 (모터 i)
```
inst   = (now_ms − pos_inst_tick) <= ORIGIN_INST_TO_MS            ← D11: RW 경로에서는 항상 0
         && hcan1.Init.TransmitFifoPriority == ENABLE              ← §5.4 방어선: FIFO 설정이 아니면 원점 FSM 자체를 안 켠다
active = (cmd.ctr_mode[i]==MODE_POSITION) && inst && motor_on && !ESTOP_override && (tx_mask & bit i)
snap   = CRIT{ fb=ECU_AK[i].state.position, vel=.velocity, ts=ECU_AK[i].ts_fb }

[0] 확인 (confirming 중) — active 와 무관하게 수행. SET_ORIGIN 이 이미 나갔으므로 중간에 ESTOP·INST_POS 중단이 와도 결과를 확정해야 한다:
        if (int32)(ts − (t_origin + MARGIN)) > 0 :                  ← 원점 이후 fb
            if |fb| < SHIFT_OK : 확정 → S_pub=S_raw, seq++, shifted=1, confirming=0
                                 residual 계산 (§5.5), odiag_* 래치
            else               : 실패 → frame_lost=1 (Orin 은 seq 미변화로 offset 안 바꿈)
        else if 경과 > CONFIRM_TO : 실패 → frame_lost=1

[1] FRAME_LOST
    if ctr_mode[i] != MODE_POSITION : frame_lost=0, shifted=0 (Orin 재무장 = POSITION 이탈 후 재진입)
    if shifted && 활성 mask && !(ready_mask & bit i) : frame_lost=1  ← 모터 통신 상실 = 전원 재기동 가능성
    if frame_lost : cmd = CURRENT_BRAKE(BREAK_CURRENT_SW), armed=0, pending=0, confirming=0, return
                    ← TX skip 이 아닌 제동. AK 는 마지막 위치 목표를 계속 따라가므로 TX 만 끊으면 멈추지 않는다

[2] 재진입 / 비활성:
    if confirming && !armed && POSITION : cmd.ctr_mode[i]=MODE_ESTOP (이 모터 TX 만 수 ms 쉼), return
                    ← 확인 중 재진입이면 fb·cmd 가 어느 좌표인지 모른다
    if !active : armed=0, pending=0 (seq·S_pub 유지), return        ← cmd 는 손대지 않음 (RW 경로 그대로 통과)

[3] 시드: if !armed :
        c = cmd.cmd_position[i]
        if |c − fb·0.1| > JUMP_TH : c = fb·0.1, jump_rej++          ← 진입 첫 cmd 점프 방지
        last_tx = c, armed = 1 → [5] 로

[4] 보정/점프 가드:
        c = cmd.cmd_position[i]
        if pending && |c − last_tx| > JUMP_TH : c −= S_raw·0.1      ← old-frame cmd → 현재 프레임
        else if pending                      : pending = 0           ← Orin 이 새 프레임으로 넘어옴
        if |c − last_tx| > JUMP_TH           : c = last_tx, jump_rej++  ← 보정 후에도 점프면 hold

[5] 트리거: if !pending && !confirming && |fb| ≥ ORIGIN_TH_RAW && fb 가 마지막 원점 이후 값 :
        S_raw = fb, fb_old = fb, vel_old = vel, t_fb_old = ts, t_origin = now
        e_track = c − fb·0.1
        pending = 1, confirming = 1, gap = 1, origin_mask |= bit i
        c −= S_raw·0.1                                              ← 이번 tick cmd 부터 새 프레임

[6] cmd.cmd_position[i] = c ; last_tx = c
    if gap : (now − t_origin) < ORIGIN_POS_GAP 이면 ctr_mode[i] = MODE_ESTOP (POSITION 만 쉼), 아니면 gap = 0
```

### 5.4 CAN 송신 순서 — ⚠ 현재 설정이면 순서가 뒤집힌다
- AK ExtId = `(mode << 8) | id` → POSITION(4) = 0x40x, SET_ORIGIN(5) = 0x50x.
- 현재 `hcan1.Init.TransmitFifoPriority = DISABLE` = **식별자 우선순위**. 두 프레임이 메일박스에
  동시에 있으면 **POSITION(cmd−S) 이 SET_ORIGIN 보다 먼저 나간다** → 원점 전 프레임에서 −S(≈−3000°)
  로 가라는 명령 = 큰 점프.
- **조치: `.ioc` 에서 CAN1 Transmit FIFO Priority = ENABLE (시간순 송신)** 후 재생성.
  기존 동작에는 영향 없음 (모터별 1프레임/tick, 순서 무관했음).
- `RD_CAN_MOTOR_TRANSMIT(cmd, mask, origin_mask)`: origin_mask 모터에 `MODE_SET_ORIGIN(set_origin_mode=0)` 을 먼저 큐잉한다. 그 뒤 POSITION 은 `ORIGIN_POS_GAP` 동안 나가지 않는다 (§5.7).
- 큐 stale 폐기(`CAN_TX_STALE_MS`)로 SET_ORIGIN 만 빠지는 경우는 [0] 확인 실패 → FRAME_LOST 가 잡는다.
- **방어선**: `hcan1.Init.TransmitFifoPriority != ENABLE` 이면 FSM 을 켜지 않는다. CubeMX 재생성으로 설정이 되돌아가도 원점 이동이 위험하게 돌지 않고, INST_POS 는 자동 원점 없이 통과한다 (±3200° 한계 그대로).

### 5.5 S 와 진단 (1단계: S = fb_old)
- 원점은 "AK 가 SET_ORIGIN 을 처리한 순간의 실제 위치 p_true" 를 0 으로 잡는다.
  S 오차 `e = S − p_true` 는 ① 원점 직후 목표 1회 점프 e, ② Orin 절대좌표 bias e (이벤트마다 누적).
- **ECU 보정에 쓰는 S 와 발행하는 S 는 같은 양자화 값(S_raw·0.1)** → ECU·Orin 좌표계 상호 정합은 항상 보장.
- 진단 (PAGE1 336~343, 마지막 이벤트):
  - `odiag_e_track = 원점 tick 의 cmd(옛 좌표) − fb_old` — 추종 오차. S 후보(cmd vs fb) 차이. 작으면 모터가 cmd 를 잘 따라감.
  - `odiag_dt = t_origin − t_fb_old` — fb 지연. `v·dt` 가 fb_old 방식의 오차 상한.
  - `odiag_residual = v·(t_fb_new − t_fb_old) − fb_new·0.1` [°] (S=fb_old 일 때 `p_true_est − S` 와 같다)
    — **실측 S 오차**. v = vel_old × `AK_VEL_RAW_TO_DEGPS`.
- 2단계(필요 시): `S = fb_old + v·dt` 외삽 — [5] 의 `S_raw` 한 줄만 바뀐다. T5 결과로 판단.

### 5.6 시퀀스 (정상)
| 시점 | ECU | 응답 PAGE1 | Orin |
|------|-----|-----------|------|
| k | \|fb\|≥3000° → S 래치, SET_ORIGIN (POSITION 20ms 쉼) | flag PEND=1, seq 불변 | old-frame cmd 계속 |
| k+1… | old-frame cmd → −S 보정, 새 프레임 fb 대기 | PEND=1, seq 불변 | old-frame cmd 계속 (ECU 가 보정) |
| k+m (≤~10ms) | 새 프레임 fb 확인 → S_pub=S, seq++ | PEND=1, **seq 변화**, pre_origin=S | `offset += S` |
| k+m+1 | 새 프레임 cmd 수신 → PEND=0 | PEND=0 | `cmd_rel = cmd_abs − offset` |

200Hz 루프는 한 번도 멈추지 않고, Orin 동작은 "seq 가 바뀌면 offset += pre_origin_pose" 뿐이다.

### 5.7 안전 규칙 요약
| 위험 | 대응 |
|------|------|
| fb 지연으로 old-frame cmd 를 못 알아봄 | 판정 기준 `last_tx` (D4) |
| POSITION 프레임이 SET_ORIGIN 보다 먼저 나감 | TXFP=ENABLE (§5.4) |
| SET_ORIGIN 직후 POSITION 에 AK 가 26~31A 로 달림 (실기: 약 30°/s 에서 ~900°/s, 이유 미확인) | `ORIGIN_POS_GAP` 20ms 동안 POSITION 을 쉰다. 실기 M1 모터 180°/s 통과. 쉬는 동안에도 속도는 약 25ms 안에 거의 0 으로 떨어지고 약 75ms 뒤 회복하므로, 고속 통과는 충격이 있어 운용에서 3000° 도달을 피한다 |
| SET_ORIGIN 유실·미적용 | 확인 단계 [0] → FRAME_LOST + 제동, seq 미발행. ⚠ 판정까지(최대 ~12ms, fb 100Hz) 모터는 옛 좌표에서 `c−S` 쪽으로 움직일 수 있다 |
| 원점 확인 중 ESTOP·INST_POS 중단 | 확인 단계는 active 와 무관하게 수행 → 프레임 변화가 Orin 에 항상 통보됨 |
| 기존 RW POSITION 이 자동 원점·점프 가드에 걸림 | FSM 은 INST_POS 수신 중에만 활성 (D11) |
| Orin offset 이중 적용 | seq 기반 (D5) |
| 같은 모터 연속 원점으로 S 덮어쓰기 | pending/confirming 중 트리거 금지 + "원점 이후 fb" 조건 |
| 모터 전원 재기동 → temporary 원점 소실 | shifted && 통신상실 → FRAME_LOST, POSITION 이탈 전까지 제동 |
| 비정상 cmd 점프 (Orin 버그)·NaN | 점프 가드 → hold last_tx, `origin_jump_rej` 카운트 (DIAG addr 228) |
| ECU 리부트 | seq 0 리셋 + realtime_tick 역행 → Orin 이 offset 무효화 (§9) |

---

## 6. RLS (UART4 MUX) 레지스터 연결

- `RLS_NUM` 2→**4**: ch0 ψ₁ AksIM 18bit / ch1 ψ₂ AksIM 18bit / ch2 θ₃ AksIM 18bit (2026-11-08, 종전 Orbis 14bit) / **ch3 linear 커스텀 보드 (4번 채널, Y3)**.
- `RD_MUX_RLS_INIT` 후 채널별 재초기화: `RD_RLS_INIT(&enc[2], RLS_ORBIS)`, `RD_RLS_INIT(&enc[3], RLS_CUST)`.
  폴링 명령은 네 채널 모두 `'3'`(0x33) 이다. `RD_MUX_RLS_SWEEP(mux, cmd)` 시그니처는 유지하되
  `RLS_CUST` 채널은 `cmd` 를 무시하고 §6.1 규칙으로 명령을 고른다 — `ENC_COMMAND` 로 `'d'` 등을 디버그해도 보드에 잘못된 요청이 가지 않게.
- **무응답·파싱 실패 표시**: 현재 `enc[i].error` 는 파싱 성공 때만 갱신된다. `RD_MUX_RLS_TRANSFER` 의
  skip/miss 경로에서 `error=1` 을 세우고, `RD_RLS_INIT` 도 `error=1` 로 시작한다 (첫 성공이 지운다).
- ⚠ **하드웨어 확인**: Orbis 파트넘버의 UART 보레이트가 UART4 설정(1Mbps, `main.c:543`)과 같은지.
- **Checker/Recovery (systemTask 100Hz)**: uart6(IMU) 와 같은 `RD_UART_CHECKER` + `RD_UART_RECOVERY` 패턴. HAL 에러 재무장도 여기서 된다.
  FAULT 로 번지지 않는 텔레메트리 채널이고, 첫 수신 전에는 READY 대기라 RLS 를 안 꽂은 구성에서는 아무 일도 없다.
  채널별 상태는 `rls_ew` 로 본다 (한 채널만 빠져도 UART 채널 전체는 정상일 수 있음).
  `hw.error/fatal/reset` 비트는 `uart4` = **bit6**. bit5 는 진단 인덱스 5(로드셀, Orin 03 §3.1) 와 맞추려고 비워 둔다.
- 주기: encoderTask 2ms 유지. 4채널 순회 ≈ 0.7~0.8ms 추정 (실측 0.18ms/채널 기준 + linear 8B). 실측은 `sweep_tick`.

### 6.1 linear encoder 커스텀 패킷 (`RLS_CUST`)

요청은 기존 RLS 와 같은 ASCII 1바이트, 응답은 **헤더 없이 항상 8B**.

| 요청 | 응답 8B | 의미 |
|------|---------|------|
| `'3'` (0x33) `CUST_POS` | int32 x + int32 y | 현재 위치 |
| `'i'` (0x69) `CUST_ZERO` | 전부 `0x00` | 초기화 성공 |
| `'i'` (0x69) | 전부 `0x11` | 초기화 실패 |
| 그 외 잘못된 요청 | int32 `0x7FFFFFFF` × 2 (와이어 `FF FF FF 7F FF FF FF 7F`) | 거부 (NAK) — 위치로 나올 수 없는 값 |

**바이트 순서**: little-endian (2026-11-04 확정). 거부 패턴은 2026-11-05 결정 (`CUST_BAD_REQ_VALUE`, 추후 변경 가능. 보드 펌웨어와 맞출 것).

`rd_comm_rls.h` 추가:
```c
typedef enum { RLS_ORBIS = 0, RLS_AKSIM, RLS_CUST } RLS_Model_e;
typedef enum { CUST_POS = 0x33, CUST_ZERO = 0x69 } RLS_CMD_CUST_e;
typedef enum { CUST_ZERO_NONE = 0, CUST_ZERO_OK, CUST_ZERO_REJECT,
               CUST_ZERO_NO_RESP, CUST_ZERO_BAD_FRAME, CUST_ZERO_BAD_REQ } RLS_CustZero_e;
/* RLS_comm_t 추가 필드 */
int32_t lin_x, lin_y;        /* '3' 위치 [count] — 실패 시 직전 값 유지 */
uint8_t zero_result;         /* RLS_CustZero_e → lin_zero_result */
uint8_t zero_seq;            /* 'i' 완료 카운터 → lin_zero_seq   */
```

`RLS_ReadCust(o, b, len, cmd)` 판정:
```
len != 8                 : cmd=='i' 이면 zero 결과 BAD_FRAME. RET_NOK (framing 통보)

cmd == '3':
    x=y=0x7FFFFFFF : RET_NOK                ← 거부. 값 유지, rls_ew Error=1
    else           : lin_x, lin_y 갱신, error=0, RET_OK
cmd == 'i':
    전부 0x00 : lin_x=lin_y=0, zero_result=OK, zero_seq++, RET_OK
    전부 0x11 : zero_result=REJECT, zero_seq++, RET_OK     ← 통신은 정상. degraded 에 넣지 않음. x·y 유지
    x=y=0x7FFFFFFF : zero_result=BAD_REQ, zero_seq++, RET_OK  ← 요청이 깨졌다. 초기화 안 됨. x·y 유지
    else     : zero_result=BAD_FRAME, zero_seq++, RET_NOK
```

**`'i'` 흐름 (encoderTask)**: 채널 3 차례에서 `lin_zero_req` 가 1 이면 지우고 `CUST_ZERO` 를, 아니면 `CUST_POS` 를 보낸다.
`'i'` 를 보낸 차례에 타임아웃이면 `zero_result=NO_RESP, zero_seq++`. 그 스윕에 위치 판독은 없고 다음 스윕(2ms 뒤)부터 새 원점 값이 온다.
⚠ `NO_RESP` 는 "초기화 안 됨"이 아니라 **불확정**이다 (응답만 유실됐을 수 있다).

**보드 쪽 요구사항** (ECU 수신 구조에서 나오는 조건):
- 1 Mbps 8N1.
- 8B 를 끊김 없이 연속 송신. ECU 는 UART IDLE 로 프레임을 자르므로 바이트 사이 간격이 1문자 시간(약 10µs)을 넘으면 프레임이 쪼개져 형식 불일치가 된다.
- 요청 후 **1ms 안에 응답 완료** (`RLS_RX_TIMEOUT_TICK`=2 는 최소 1ms 만 보장). `'i'` 도 마찬가지 — 초기화 처리가 길면 먼저 응답하거나 이 상수를 키워야 한다.
- 커스텀 패킷엔 유효성 비트가 없다. 엔코더 자체 이상을 보드가 ECU 에 알릴 방법은 없고, `rls_ew` ch3 는 **통신 실패만** 나타낸다.

---

## 7. I2C 엔코더 `#if RD_USE_I2C_ENCODER`

`rd_define.h`: `#define RD_USE_I2C_ENCODER 1` (기본 ON — project 모드 사용. position 실험 빌드에서 0).

OFF(0) 일 때:

| 위치 | 처리 |
|------|------|
| `RD_TASK_I2C1` | 루프 대신 `osThreadSuspend(osThreadGetId())` (CubeMX 생성부 `osThreadNew` 는 손대지 않음 — 재생성 안전) |
| `RD_PERIPHERAL_INIT` | `RD_I2C_ENCODER_INIT` 생략, `err.i2c.state = LS_OFFLINE / HC_OK` |
| `RD_PERIPHERAL_I2C`, `RD_I2C_ENCODER_RECOVERY` 호출부 (systemTask req.bit.i2c 포함) | 컴파일 제외 |
| `RD_SYSTEM_EVALUATE_STATE` | i2c 비트 평가 제외 (OFFLINE 이 fatal 로 번지지 않게) |
| MARSHAL_PUBLISH | `reg.encoder` = 0, `delta_tick[]` = 0xFF, `state = LS_OFFLINE` 고정 → "꺼짐" 이 정상 0 과 구분됨 |

레지스터 영역(70~85)은 **판독 가능하게 둔다** — 기존 Orin 프리셋 `{48,80}` 등이 이 영역을 가로질러
읽으므로 ACCESS 에러로 막으면 세그먼트 전체가 실패한다.

---

## 8. 실기 검증 항목 (구현 후, 저속·무부하부터)

| # | 항목 | 통과 기준 / 산출 |
|---|------|------------------|
| T0 | CubeIDE 빌드 + 기존 기능 회귀 (velocity/current AUTO, MANUAL) | 기존 Orin 프리셋 정상 |
| T1 | 위치 단위: POSITION 360° 지령 → 출력축 회전수 / velocity raw 스케일 | `AK_VEL_RAW_TO_DEGPS` 확정 |
| T2 | (참고) fb 가 ±3200° 를 넘을 때 거동 (포화/랩) | 기록만 |
| T3 | 정지 상태 SET_ORIGIN (POSITION 유지 중) | fb→≈0, 모터 튐 없음, confirm 시간 |
| T4 | 저속(출력 ~10rpm) 연속 이동으로 3000° 통과 | 튐 없음, seq 1 증가, `odiag_*` 수집 |
| T5 | 등반 속도 범위 반복 | residual 분포 → 2단계(외삽) 필요 여부 결정 |
| T6 | POSITION 중 모터 전원 차단·복구 | FRAME_LOST → 제동, 재무장 전 위치 명령 안 나감 |
| T7 | Orin 점프 cmd 주입 | hold + jump_rej 증가 |
| T8 | RLS 4채널 혼합 판독 / 1채널 분리 시 rls_ew·degraded, `sweep_tick` | 값·상태 정상, 순회 < 2ms |
| T8b | linear: `'3'` x·y 부호·방향 확인 (바이트 순서 검증), `lin_zero_req` → seq 1 증가·값 0, 보드 거부·무응답 시 result 2/3 | x·y 정상, 초기화 1회만 실행 |
| T9 | `RD_USE_I2C_ENCODER 0` 빌드 | 70~85 OFFLINE/0xFF, FAULT 없음 |
| T10 | INST_POS 200Hz 장시간 | 응답 ~1.05ms, proc_delta·LOCKED 무발생 |
| T11 | 기존 경로 회귀: RW POSITION 으로 3000° 통과·1000° 초과 스텝, `OP_SET_ORIGIN` | 자동 원점·점프 가드 미발동, 기존과 동일 |

---

## 9. Orin 계약 (ECU 가 전제하는 것 — 상세는 12 문서)

1. 진입: `auto_mode=5` WRITE 후 INST_POS 사용. 진입 첫 cmd 는 현재 fb 근처 (bumpless — 기존 IDLE 규칙).
   자동 원점은 INST_POS 를 보낼 때만 동작한다. RW 로 POSITION 을 보내면 기존과 같고 ±3200° 한계도 그대로다.
   **INST_POS 로 원점이 옮겨진 뒤 RW 로 넘어가면 모터 프레임은 옮겨진 상태 그대로다** — 전환 전에 Orin 이 offset 을 반영해야 한다.
2. 좌표: `abs_fb = offset_i + mtr_position·0.1`, `cmd_rel = cmd_abs − offset_i`.
   **PEND=1 이고 `origin_seq` 가 아직 안 바뀐 응답의 `mtr_position` 은 쓰지 않는다** (직전 abs 유지).
   그 구간에는 새 좌표 fb 가 offset 반영보다 먼저 보일 수 있어서 abs 가 한 샘플 −3000° 로 튄다. seq 가 바뀐 응답부터는 fb·offset 이 일치한다.
3. **`origin_seq[i]` 가 바뀐 응답에서만 `offset_i += pre_origin_pose[i]·0.1`.** 차이가 2 이상 → 이벤트
   유실 = 좌표 불확정 → 정지·재호밍. `origin_flag` PEND 는 표시용.
4. `FRAME_LOST` / ECU 리부트(realtime_tick 역행 또는 seq 리셋) / 브리지 재시작 → offset 무효,
   POSITION 이탈 후 재호밍.
5. Data[0] 은 기존 규약 그대로(에러 전용) — 원점은 에러가 아니다.
6. linear 초기화: `lin_zero_req`(addr 193) 에 1 WRITE → `lin_zero_seq` 변화로 완료 판단 후 `lin_zero_result` 확인. Orin 이 재전송하지 않는다.
   `lin_x/y` 는 `rls_ew` ch3 Error=0 일 때만 유효.

---

## 10. 미결 / 후속

- linear encoder 스케일 (사용자 확인, 2026-11-08): `lin_x` = 10 um/count, `lin_y` = 20 um/count. 방향은 각 축 방향대로 읽힌다.
- **구현 상태 (2026-11-05)**: §2~§7 전부 코드 반영 완료, 빌드·실기 미검증. `.ioc` CAN FIFO 는 사용자가 CubeMX 로 수정 예정 — 그 전까지는 §5.4 방어선 때문에 자동 원점이 켜지지 않는다.
- Orin 미러 갱신 전까지 `test_stm_mirror` 는 깨진다 (stm_ws 가 있는 환경에서만 빌드되는 테스트). 12 문서에서 처리.
- 2단계 S 외삽 여부 (T5 후).
- `ORIGIN_TH_RAW` 여유(200°)가 최고 속도에서 충분한지 — 5ms 에 200° = 출력 ~6700°/s 이상이어야 위험 → 등반에선 무관.
- 12 Orin 구체안: INST_POS 슬롯·OriginManager·POSITION 프로파일 절대좌표화·텔레메트리.
- 13 WEB 구체안.
