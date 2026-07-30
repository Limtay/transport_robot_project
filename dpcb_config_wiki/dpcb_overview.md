# DPC_B 시스템 오버뷰

> 최종 갱신: 2026-07-04  
> 상위 인덱스 문서. 세부는 각 분할 문서 참조.

---

## 1. 시스템 목적

- DPC_B: **3축 전동 윈치 크레인** 제어 보드 (STM32F446RET6)
- 역할: Orin(마스터)의 RS485 명령 수신 → Dynamixel 윈치 3개 구동 + 집게 장치(DPC-A) 연동 → 화물 자동 전개
- 윈치 = Dynamixel 3개 / 집게(크레인 고리) = DPC-A / 패널 입력 = MCP23017

---

## 2. 하드웨어 구성도

```
[Jetson Orin AGX]
       │  RS485 / Dynamixel 2.0-like 패킷 (USART2, 921600 bps)
       ▼
   [DPC_B : STM32F446RET6]
       │                    │                    │
  UART4 (4-byte pkt)   USART6 (RS485)        I2C1
       │                    │                    │
   [DPC_A]          [Dynamixel × 3]          [MCP23017]
   집게/그리퍼         윈치 모터 3개           패널 스위치
```

---

## 3. 분할 문서 인덱스

| 문서 | 내용 |
|------|------|
| [dpcb_overview.md](dpcb_overview.md) | 시스템 목적·구성·파일 구조 (본 문서) |
| [dpcb_opmode.md](dpcb_opmode.md) | 운용 모드(mode)와 sys_state (2모드+sys_state 통합 구조) |
| [dpcb_peri.md](dpcb_peri.md) | 페리페럴 구조 + 통신 채널 |
| [dpcb_task.md](dpcb_task.md) | FreeRTOS 태스크 구조 + 전역 객체 |
| [dpcb_register.md](dpcb_register.md) | 레지스터 맵 구조·필드·MARSHAL 매핑 |
| [dpcb_checker.md](dpcb_checker.md) | Checker / failsafe 조건, 디버깅 참조 |
| [plan.md](plan.md) | 구현 단계(Step) 및 진행 상태 |
| [history.md](history.md) | 세션별 작업 히스토리 |

---

## 4. 소스 파일 구조

```
stm_ws/DPC_B/Core/
  Inc/
    rd_system.h          전역 객체 extern, 타입 정의, 태스크 프로토타입
    rd_register_dpcb.h   레지스터 맵 구조체
    rd_uart.h            UART_Ring_t (wake_task 포함), RS485_t
    rd_peripheral_dpcb.h PERIPHERAL_t, PERIPHERAL_MOT_t
    rd_map_dyn.h         DYN_Ctrl_t, RD_DYN_LOOP
    rd_map_dpcb.h        REGISTER_t reg, DISPATCH/MARSHAL 프로토타입
    rd_comm_orin.h       ORIN_COMM_t, RD_ORIN_* (Orin RS485 패킷 레이어)
    rd_comm_dpcb.h       PACKET_s_t, PACKET_comm_t (DPC-A 4-byte)
    rd_control.h         CONTROL_DPC_t, RD_CONTROL_LOOP
    DYN_xm430_w350.h     Dynamixel 컨트롤 테이블 주소/크기 상수

  Src/
    rd_system.c          전역 객체, RD_SYSTEM_INIT, RD_TASK_* 구현
    rd_map_dpcb.c        DISPATCH_WRITE/READ, MARSHAL_PUBLISH/CONSUME
    rd_uart.c            UART 드라이버 (CHECKER / RECOVERY 포함)
    rd_peripheral_dpcb.c RD_PERIPHERAL_INIT/READ/WRITE, RD_DPCA_UPDATE
    rd_map_dyn.c         Dynamixel 루프/패킷 레이어
    rd_control.c         RD_CONTROL_LOOP, deploy FSM
    stm32f4xx_it.c       ISR (USART2/4/6 TC+IDLE 핸들러)
    main.c               CubeMX 생성, 태스크 생성
```
