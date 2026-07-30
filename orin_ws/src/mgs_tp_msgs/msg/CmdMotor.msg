# 스트림 명령 입구 — 웹 슬라이더 + MPC 공용 (redesign/03 §5.3)
#
# **arm=on 일 때만 소비한다** (01 §6.1.3 — 웹 run/stop 버튼).
# arm=off 면 구독은 하되 버린다. 그래야 노드 기동 직후 남아 있던 발행자(웹 탭, 죽다 만 MPC)가
# 조작자 모르게 모터를 움직이는 일이 없다.
# 스테일 > timeout -> IDLE + arm 자동 off. LOCKED 는 비정상 경로 전용이다.
std_msgs/Header header      # 명령 생성 시각 — 스테일 판정 기준
uint8[4]   ctr_mode         # ⚠ **예약 — 브리지는 읽지 않는다 (2026-07-28 확인).**
                            # 종전 주석은 "auto_mode=DIRECT 에서만 유효" 라고 적었으나
                            # 실제로는 StreamCmd_t 에 담기기만 하고 아무도 안 읽는다.
                            #
                            # 의도적으로 그렇게 둔다: ctr_mode 변경은 `ControlConfig`
                            # OP_SET_CTR_MODE 로만 하며 그 경로는 **IDLE 전용**이다.
                            # 200Hz 스트림 메시지가 제어 모드를 바꿀 수 있으면 회전 중에
                            # 모드가 바뀌고, safe_stop 게이트(01 §6.2)가 무의미해진다.
                            # DIRECT 의 단위는 config 로 정해 둔 모터별 ctr_mode 가 정한다.
float32[4] position         # [deg]
float32[4] velocity         # [RPM]
float32[4] current          # [A]
