# 모터 4채널 진단 (redesign/03 §5.3, 20Hz)
#
# 구 토픽 10개를 대체한다: carrier/ecu/motor/{current/raw, current/filtered,
#   pose, speed, temp, error} + carrier/ecu/motor/comm_err/{m0..m3}
# 모터 데이터는 프로젝트 계약이 아니라 진단이므로 이쪽 네임스페이스로 옮긴다.
#
# ⚠ 엔코더(링크 각도)는 여기 없다 — 모터 데이터가 아니고 project 프리셋이 읽지도 않는다.
#   ControlFeedback.link_angle 로만 나간다 (03 §4.1).
std_msgs/Header header
uint8      motor_mask       # 활성 모터 비트필드 (bit0~3 = M1~M4)
float32[4] position         # [deg]
float32[4] velocity         # [RPM]
float32[4] current          # [A]
float32[4] current_filtered # [A] EMA (alpha=0.05)
int8[4]    temp             # [°C]
uint16     error_code       # 4bit x 4 packed (LSB=M1) — AK 에러코드
uint8      comm_err         # 2bit x 4 packed (LSB=M1): bit0=RX err, bit1=TX err
uint8      state            # STATE_t raw (4모터 중 worst)
