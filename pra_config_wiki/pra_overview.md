# PRA 시스템 오버뷰

> 최종 갱신: 2026-10-04 (Step7 완료 + 실보드 ADC/EXIO 검증 통과. 다음=Step8 BMS CAN)
> 상위 인덱스 문서. 세부는 각 분할 문서 참조.

---

## 1. 시스템 목적

- PRA: **배터리 입력 감시 + 대용량 세이프티/스위칭 보드** (STM32F446RET6). **Orin/외부 관점에서는 `PCU`** 로 지칭 (§6)
- 역할: 배터리 입력을 감시하고 메인파워·보조전원을 스위칭하는 세이프티 계층. Orin(마스터)의 RS485 명령 + 로컬 스위치 입력으로 전원 온오프 제어
- 기존 검증 완료된 **ECU / DPC_B 와 동일 모듈 구조 공유** — RS485 modbus 레지스터맵 통신·`rd_*` 드라이버 계층을 그대로 재사용

### 러프 기능

1. **메인파워 (100A급)**: 제어용 P/N 릴레이 + 솔리드스테이트 프리차저 → **전압 측정 기반 순차 제어**
2. **서브파워 (5A급)**: 보조전원 공급, 2채널 (AUX1=MCU / AUX2=ORIN)
3. **BMS CAN 통신**: 배터리 전압/전류/SOC/SOH/온도/상태 수집
4. **스위치 입력**: 로컬 패널 스위치(MCP23017)로 AUX·메인파워 온오프 제어
5. **Orin 통신**: RS485 modbus (레지스터맵 기반)

---

## 2. 하드웨어 구성도

```
        [BMS]                         [Jetson Orin AGX]
          │ CAN1 (250 kbps)              │ RS485 modbus (USART6, 921600 8N1)
          ▼                              ▼
      ┌──────────────── [PRA : STM32F446RET6] ────────────────┐
      │                                                        │
   ADC1 (DMA 6ch)     GPIO 파워단          I2C1            USART1 (EX RS485)
   NTC×3/V×2/I×1   릴레이·프리차저·AUX     MCP23017          미사용(확장용)
      │                  │                  │
   배터리 감시     P/N릴레이·PC·AUX1/2    패널 SW+LED
```

> USART1(EX RS485)은 통신형 BMS(RS485/CAN 2종) 확장 대비로 MX 에서 열어뒀으나 **현 시스템 미사용** — BMS 는 CAN1 사용. [pra_peri.md](pra_peri.md) §1.

---

## 3. 분할 문서 인덱스

| 문서 | 내용 |
|------|------|
| [pra_overview.md](pra_overview.md) | 시스템 목적·구성·파일 구조·진행 상태 (본 문서) |
| [pra_peri.md](pra_peri.md) | 페리페럴 구조 + 통신 채널 (ADC/CAN/RS485×2/I2C/GPIO) |
| [pra_register.md](pra_register.md) | 레지스터 맵 구조·필드 (CSV 초안 기반) |
| [pra_opmode.md](pra_opmode.md) | sys_state + 전원 제어 시퀀스(메인파워 순차·프리차저) |
| [pra_task.md](pra_task.md) | FreeRTOS 태스크 구조 + 전역 객체 |
| [pra_checker.md](pra_checker.md) | Checker / failsafe 정책 (USART6 구현, ECU 참조) |
| [BMS_protocol_can.md](BMS_protocol_can.md) | BMS CAN 프로토콜·PCAN 실측 디코드·레지스터 매핑 (규격 전용) |
| [pra_bms_can.md](pra_bms_can.md) | BMS CAN 구현 계획·Step·작업 로그 (ECU CAN 참조, Step8 실작업) |
| [plan.md](plan.md) | 구현 단계(Step) 및 진행 상태 |
| [history.md](history.md) | 세션별 작업 히스토리 |

---

## 4. 소스 파일 구조 (`stm_ws/PRA`)

```
stm_ws/PRA/
  PRA.ioc                  CubeMX 프로젝트 (핀맵/페리페럴 정의)
  Core/
    Inc/
      main.h               CubeMX 생성 핀 라벨 매크로
      rd_register_pra.h    256B REGISTER_t 레지스터 맵 (Step2·3 완료)
      rd_common.h          RD_RET + STATE_t/LS_*/HC_* (Step4 이식 — DPC_B 상위호환본)
      rd_comm_orin.h       ORIN_COMM_t, RD_ORIN_* (Orin RS485 패킷, 노드 0xA1) (Step4)
      rd_map_pra.h         REGISTER_t reg, DISPATCH/MARSHAL, ORIN_Error_e (Step5)
      rd_system.h          전역객체 extern, SYSTEM_STATE_e, HW_ERROR_FLAG_t, RD_SYSTEM_INIT/RD_TASK_RS485/RD_TASK_SYSTEM (Step4·6)
      rd_uart.h            UART_Ring_t(STATE_t error/wake_task), RS485_t — ECU/DPC_B advanced (Step6 교체)
      rd_peripheral.h      GPIO_IO_t 만 정의 (골격)
      rd_define.h          빈 파일 (골격)
      FreeRTOSConfig.h / stm32f4xx_hal_conf.h / stm32f4xx_it.h
    Src/
      main.c               CubeMX 생성 — MX_*_Init, 태스크 함수(RD_TASK_RS485 호출), ADC ConvCplt,
                           NTC(Steinhart-Hart)/전류·전압 변환 헬퍼
      rd_comm_orin.c       Orin RS485 패킷 빌더/파서 (CRC-16, 멀티세그 READ) (Step4)
      rd_map_pra.c         reg 정의, 영역 LUT, DISPATCH R/W, RD_MAP_INIT, MARSHAL 스텁 (Step5)
      rd_system.c          전역객체, RD_SYSTEM_INIT, RD_TASK_RS485, RD_TASK_SYSTEM(Checker), HAL_UART_ErrorCallback (Step4·6)
      rd_uart.c            UART/RS485 드라이버 — ECU/DPC_B advanced (Checker/Recovery/degraded) (Step6 교체)
      freertos.c           CMSIS-RTOS v2 훅 (USER CODE 비어있음)
      stm32f4xx_it.c       ISR (Step4: USART6 TC/IDLE 배선)
      (그 외 CubeMX 생성 파일)
```

> **미생성 (계획)**: `rd_peripheral_pra` 본체·연동(Step7, 스켈레톤 有), `rd_comm_bms.*`(CAN, Step8), controlTask 전원 FSM(Step9), MARSHAL SENSOR/CONSUME(Step10). CAN1/I2C1 Checker 는 해당 페리페럴 추가 후. 상세: [plan.md](plan.md).

---

## 5. 현재 진행 상태 (2026-09-18)

| 항목                                                    | 상태                                                                                        |
| ----------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| CubeMX 페리페럴 초기화 (ADC/CAN/USART1/USART6/I2C1/GPIO/DMA) | **완료** (`.ioc` + `MX_*_Init`)                                                             |
| ADC DMA + ConvCplt 스냅샷 + NTC/전류·전압 변환 헬퍼              | **완료** (`main.c`)                                                                         |
| `rd_uart` 드라이버                                        | **완료** — Step6 에 ECU/DPC_B **advanced 판 교체**(STATE_t/degraded/lifecycle/Checker/Recovery) |
| 레지스터 맵 (`rd_register_pra.h`)                          | **완료** (Step2·3 검증)                                                                       |
| RS485 modbus 계층 (`rd_comm_orin`/`rd_map_pra`)         | **완료** (Step4·5) — Orin READ/WRITE/PING/REBOOT                                            |
| `rd_system` (INIT + RS485 태스크 + Checker/systemTask)   | **완료** (Step4·6)                                                                          |
| Checker (USART6/Orin 채널)                              | **완료** (Step6) — CAN1/I2C1 은 페리페럴 추가 후                                                    |
| 페리페럴 본체 (`rd_peripheral_pra` 연동 + EXIO)          | **완료 + 실보드 검증(2026-10-04)** — ADC(NTC/Vin/Vout/전류)·MCP23017 EXIO I2C 정상. 출력단 구동 Step9 전 확인 |
| MARSHAL PUBLISH — SYSTEM 영역                           | **완료** (Step6) / SENSOR·CONSUME 본체 **Step10**                                             |
| BMS CAN 파서                                            | **미생성** (Step8)                                                                           |
| 전원 제어 로직 (릴레이/프리차저 FSM)                               | **미생성** (Step9)                                                                           |
| FreeRTOS 태스크 본체                                       | **rs485Task·systemTask 실동작** / defaultTask(LED) / control·can1 빈 루프                       |

> 요약: **Step7 까지 완료 + 실보드 ADC/EXIO 검증 통과(2026-10-04)** (레지스터맵·Orin RS485·USART6 Checker·페리페럴 계층). **▶ 다음 = Step8 BMS CAN** → Step9 전원제어 → Step10 MARSHAL(완료).

---

## 6. Orin 연동 — PRA = PCU (확정)

> **명칭 규약 (2026-09-17 확정)**: 본 보드는 **STM 개발문서(pra_config_wiki·`stm_ws/PRA`)에서는 `PRA`**, **Orin 및 외부 관점에서는 `PCU`** 로 지칭. 동일 노드의 두 이름.

- Orin 브리지(`orin_ws/src/orin_firmware_bridge`)의 **`PCU` 노드(PacketID `0xA1`)** = **PRA** (`rd_comm.hpp`, `rd_register_pcu.hpp`). 현재 Orin 측은 "레지스터 상당 부분 미확정", `enable_pcu_read` 기본 off.
- PRA 의 `ORIN_MY_ID` = **`0xA1`** (PCU). Orin `PacketID::PCU` 와 반드시 일치 (불일치 시 ID 필터에서 조용히 폐기되는 "무응답" 증상 — DPC_B history §20 교훈).
- ⚠ **양측 레지스터맵 현재 불일치**: Orin PCU 맵은 `DEFINE(0:16)/SYS(46:16)/POWER(96:8)/CMD(128:4)/DIAG(224:32)`(placeholder), PRA 맵은 `SYSTEM(16~)/SENSOR(38~60)/cmd(61~63)/DIAG(80~99)`. **레지스터맵 정합 필수**.
- **TODO**: ① 레지스터맵 Orin↔PRA 정합(기준 방향 결정), ② Orin `enable_pcu_read` 활성화 시점. 상세: [plan.md](plan.md) §4-1, [pra_register.md](pra_register.md) §3.
