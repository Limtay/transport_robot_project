# 통신 지연 분해 원자료 — 디버그 전용, 기본 off (redesign/03 §5.3, 5Hz)
#
# comm_diag_enable:=true 일 때만 발행한다. 클럭 추정기 자체는 항상 돌지만
# 원자료 발행은 디버그 상황에서만 한다 — 상시 필요한 시각 정보는
# ControlFeedback 이 이미 갖고 있다.
std_msgs/Header header
float64 t_req               # 요청 write 직전 [s, epoch]
float64 t_resp              # 응답 수신 완료 [s, epoch]
float32 rtt
float32 wire_up             # 계산치 (요청 bytes x 10.85us)
float32 wire_down
float32 proc_delta          # STM 처리시간 (직전 트랜잭션, -1=stale)
float32 residual            # rtt - wire_up - wire_down - proc_delta = Orin USB 지연
float64 clock_offset        # b
float32 tick_period         # a [s/tick] — 드리프트 반영된 실효 tick 주기
float32 drift_ppm
float32 quality             # 이번 샘플 불확도 (ub-lb) [s]
uint32  sample_cnt          # 추정기 누적 샘플
uint32  drop_cnt
