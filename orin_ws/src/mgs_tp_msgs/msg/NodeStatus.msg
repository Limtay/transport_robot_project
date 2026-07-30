# ECU/DPC/PCU 공용 노드 상태 (redesign/03 §5.3, 20Hz)
#
# 구 토픽 8개를 대체한다: carrier/status, carrier/{ecu,dpc,pcu}/connected,
#   carrier/ecu/{fsm,alive_time}, carrier/ecu/error 계열, carrier/ecu/{lc,hs} 계열
#
# ⚠ 부분 발행 금지: degraded_cnt(16~23)/hw_error(24)/hw_fatal(25) 은 {16,17} 을 읽어야만
#   얻어진다. 그 구간을 읽지 않은 트랜잭션에서는 **아예 발행하지 않는다** —
#   "안 읽음" 과 "정상" 을 구분하는 문제를 메시지를 안 보내는 것으로 해결한다.
#   소비자는 header.stamp 의 나이로 stale 을 판정한다 (fields_valid 비트마스크는 기각).
#   2026-07-27 전 프리셋이 {16,17} 로 시작하도록 통일돼 이 제약은 실질적으로 해소됐다.
std_msgs/Header header

bool     connected          # 브리지가 판정한 링크 상태
uint8    fsm                # 노드 FSM (ECU sys_state: 0=INIT 1=MANUAL 2=AUTO
                            #   3=ESTOP_SW 4=ESTOP_HW 5=FAULT)
float32  alive_time         # [s] 노드 기동 후 경과

# ── 진단 배열 — 인덱스 규약 (03 §3.1 공통) ──────────────────
#   0=uart1/RC  1=uart2/RS485  2=uart6/IMU  3=can1/모터  4=i2c1/엔코더
#   5=adc/로드셀  6,7=RSVD
uint8[8] lc                 # lifecycle nibble
uint8[8] hs                 # health nibble
uint8[8] degraded_cnt       # 통신 오염도 [%]
uint8    hw_error           # 비트필드 (같은 인덱스)
uint8    hw_fatal
uint8    hw_reset
