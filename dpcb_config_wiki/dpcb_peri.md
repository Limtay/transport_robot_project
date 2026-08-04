# DPC_B 페리페럴 구조 및 통신 채널

> 최종 갱신: 2026-08-04 (실기 구동 검증 — EXIO 초기화 순서 §4, SW1 mode 토글)  
> 상위: [dpcb_overview.md](dpcb_overview.md)

---

## 1. 통신 채널 일람

| 채널 | 속도 | 대상 | 방향 제어 | 전담 태스크 |
|------|------|------|----------|------------|
| USART2 | 921600 | Orin RS485 | DIR=PC3, TC→RX 자동 복귀 | `rs485Task` |
| UART4 | - | DPC-A 4-byte 패킷 | 단방향 TX+RX | `dpcaTask` |
| USART6 | 57600 | Dynamixel × 3 | DIR=PB15, 동일 패턴 | `periTask` |
| I2C1 | 400 kHz | MCP23017 패널 | - | `i2cTask` |

- RS485 방향 제어: TX 시 DIR SET → TC 인터럽트로 RX 자동 복귀
- USART2/USART6 동일 패턴, DIR 핀만 상이 (PC3 / PB15)

---

## 2. Dynamixel XM430-W350 (윈치 × 3)

- ID: MOT[0]=2, MOT[1]=3, MOT[2]=4
- 제어 모드: `DYN_MODE_CUR_POSITION` (전류 기반 위치 제어)
- 초기 전류 제한: 750 (×2.69 mA ≈ 2.0 A)

**구조체 계층:**

```
PERIPHERAL_t  DPCB_PERIPHERAL
  └─ PERIPHERAL_MOT_t  MOT[3]
       ├─ int32_t   CTL_SPEED / INIT_POS / TARGET_POS
       ├─ int16_t   LPF_CURRENT
       ├─ uint8_t   DYN_IDS  (2 / 3 / 4)
       └─ DYN_Ctrl_t  dyn_ctrl
            ├─ ram.cmd.goal_position / goal_current / goal_velocity
            └─ ram.state.present_position / velocity / current / temperature / hardware_error_status
```

**periTask 루프 패턴 (모터 1개당 5회 RD_DYN_LOOP):**

```c
RD_DYN_UPDATE_STATE(dyn)    → RD_DYN_LOOP(...)   // present 상태 READ
RD_DYN_UPDATE_HWERROR(dyn)  → RD_DYN_LOOP(...)   // HW 에러 READ
RD_DYN_UPDATE_CMD(dyn, mode)→ RD_DYN_LOOP(...)   // goal 명령 WRITE
RD_DYN_OPERATE_ON(dyn, mode)→ RD_DYN_LOOP(...)   // 모드 설정
RD_DYN_TORQUE_ON(dyn, 1)    → RD_DYN_LOOP(...)   // 토크 ON
```

- `RD_DYN_LOOP` 내부: `osThreadFlagsWait(0x0001, ..., 2ms)` — USART6 IDLE ISR → periTask 깨우기

---

## 3. DPC-A (집게 / 그리퍼)

- UART4, 4-byte simple packet (`PACKET_s_t`)
- 송신: `lock_en`(집게 잠금/해제), `boot_en`(부가 기능 — 주목적과 무관, 현재 미고려)
- 수신: `prox_contact`(지면 근접 3bit), `lock_contact`(DPC-A 측 집게 접점 4bit)

> **lock_contact 구분 주의**
> - DPC-A `lock_contact`(수신): 집게 자체의 접점 상태
> - DPC-B `lock_contact`(GPIO, 섹션 5): 집게가 크레인(DPC-B) 끝단에 도달·**고정**됐는지 확인하는 센서
> - auto FSM 상승 완료 판정 = **DPC-B 측** `lock_contact` 사용

```
PACKET_comm_t  DPCA_PACKET  →  tx(Header + Data[2] + Checksum) / rx

PERIPHERAL_t  DPCB_PERIPHERAL
  ├─ A_CON_DATA   (DPC-A 수신: 집게 잠금 접점 4bit)
  ├─ A_PROX_DATA  (DPC-A 수신: 근접센서 3bit)
  ├─ A_EN_ALL / A_EN_BOOT  (DPC-A 송신)
```

- dpcaTask 흐름: `RD_PACKET_WRITE → 폴링 → RD_PACKET_READ → RD_DPCA_UPDATE`

---

## 4. MCP23017 패널 (I2C)

| 포트 | 항목 |
|------|------|
| GPIOA | LED1(GPA0), LED2(GPA1), SW3B~SW6B(GPA2~5) |
| GPIOB | SW6A~SW1(GPB2~7) |

**스위치 값 정의**
- SPST (SW1, SW2): `0=OFF / 1=ON`
- SPDT (SW3~SW6): `0=IDLE(mid) / 1=UP / 2=DOWN`

**스위치 ↔ 기능 매핑** (매번 코드 확인 불필요하도록 명시)

| 스위치 | 타입 | reg 필드 | 기능 |
|--------|------|----------|------|
| SW1 | SPST | `ex_sw[0]` | **mode 토글** (짧게 눌러 MANUAL↔AUTO, 실기 검증 완료 2026-08-04) |
| SW2 | SPST | `ex_sw[1]` | 패널 제어 시 **locker 작동** |
| SW3 | SPDT | `ex_sw[2]` | **MOT[2]** 상하 운동 제어 (1=상승 / 2=하강) |
| SW4 | SPDT | `ex_sw[3]` | **MOT[1]** 상하 운동 제어 (1=상승 / 2=하강) |
| SW5 | SPDT | `ex_sw[4]` | **MOT[0]** 상하 운동 제어 (1=상승 / 2=하강) |
| SW6 | SPDT | `ex_sw[5]` | **servo lock/unlock** 제어 → `SERVO_EN` (0=듀티 0 / 1=lock / 2=unlock) |

> 물리 스위치 `SWn` ↔ 레지스터 `ex_sw[n-1]` (0-index). SPDT 3개는 `SW5/SW4/SW3` → `MOT[0]/MOT[1]/MOT[2]` **역순** 대응에 주의.
> 패널 제어 구동 로직 위치: `rd_control.c` MANUAL(CASE-0) 액션 (SW5=`:290` / SW4=`:307` / SW3=`:324` / SW6=`:272`).
> **SW6 servo 동작**: `SW6_state` → `SERVO_EN` 그대로 전달 — `0`=servo 입력 듀티 0, `1`=구조적 lock 수행 듀티, `2`=구조적 unlock 수행 듀티. (`SERVO_CMD_IDLE/LOCK/UNLOCK` 값과 동일 체계)

- i2cTask: `RD_EXIO_UPDATE(&DPCB_PERIPHERAL.PANEL)` 10ms 주기 — **read(SW)와 write(LED1/LED2)를 한 함수에서 순차 수행** (동일 태스크·동일 I2C, read↔write 경합 없음)

> **⚠ EXIO 초기화 순서 (2026-08-04 실기 버그, `RD_EXIO_INIT`)**: MCP23017 **RST 토글은 방향설정(`EXIO_Set_OUTPUT/INPUT`)보다 먼저** 수행해야 함. RST 가 뒤에 오면 리셋이 IODIR 을 전원기본값(전핀 input)으로 되돌려 **LED(GPA0/1) 출력이 죽음** — 이때 switch read 는 input 기본이라 정상 동작하여 "LED_state 값은 변하는데 LED 만 안 켜짐, 통신은 정상" 증상으로 나타남. 순서를 reset→configure 로 고쳐 해결. 상세: [history.md](history.md) §Session 22
> **인디케이팅 LED 정책**: LED1=MODE(MANUAL blink/AUTO solid), LED2=컨텍스트(MANUAL=locker EN/AUTO=STATE). 산출=`RD_TASK_DEFAULT`(100ms) 단독 소유, flush=`RD_EXIO_UPDATE`(10ms). 동작표: [dpcb_task.md](dpcb_task.md) §5-1

---

## 5. GPIO / 솔레노이드

```
PERIPHERAL_t  DPCB_PERIPHERAL
  ├─ EN_ALL / EN_BOOT / CON_DATA (4bit) / SERVO_EN / LIGHT_EN
  └─ PERIPHERAL_IO_ALL_t  IO  (BOOT_IO, EN_IO, CON_A/B/C/D_IO, SERVO_IO, LIGHT_IO)
```

- **DPC-B `lock_contact` (CON_DATA)**: 집게(DPC-A)가 크레인 끝단 도달·**고정** 확인 접점 센서(4bit). auto FSM descend_1→descend_2, ascend_2→finish 전이 판정에 사용
- **lock_en (EN_ALL)**: 위 고정 해제 솔레노이드. descend_1 진입 시 동작
- periTask 루프 끝에서 `RD_PERIPHERAL_READ` + `RD_PERIPHERAL_WRITE` 호출
