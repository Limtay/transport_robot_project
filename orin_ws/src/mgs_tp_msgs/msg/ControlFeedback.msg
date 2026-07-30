# control 모드 200Hz 피드백 — 이번 재설계의 중심 메시지 (redesign/03 §5.3)
#
# **1 RW 트랜잭션 = 1 메시지.** 보간·합성 금지 — 없는 값을 지어내지 않는다.
# 구 TestbedFeedback + CommLatency 를 흡수한다 (control 은 토픽 1개 계약, 03 B3).
std_msgs/Header header
  # stamp = **ECU 취득 기준시각을 Orin 시간축으로 변환한 값** (03 §2.4)
  #   = a * realtime_tick + b     (Orin 수신 시각이 아니다)
  # stamp_valid==false 일 때만 Orin 수신 시각으로 fallback

# ── 시간축 (03 B1) ─────────────────────────────────────────
uint32  ecu_tick            # realtime_tick raw [x0.1ms] — 분석의 권위 소스, 완전 복원용
float32 stamp_quality       # header.stamp 불확도 +- [s]  (0.0005 = +-0.5ms)
bool    stamp_valid         # 클럭 추정기 수렴 여부
float32 drift_ppm           # 현재 드리프트 추정 (참고. 실측 ~= -19600)

# 센서별 취득 지연 [s] — 절대 취득시각 = header.stamp - dt_*
#   미판독 프리셋 / delta_tick==0xFF(stale) -> NaN
float32    dt_imu
float32    dt_loadcell
float32[5] dt_encoder
float32[4] dt_motor         # CAN 피드백 수신 지연
float32[4] dt_motor_cmd     # CAN 명령 송출 지연

# ── 진단 (2026-07-27 추가) ─────────────────────────────────
# 인덱스 규약 = 03 §3.1 (0=uart1/RC 1=uart2/RS485 2=uart6/IMU 3=can1/모터 4=i2c1/엔코더 5=로드셀)
#   프리셋이 그 채널을 읽지 않으면 **255(0xFF) = 미판독**. OK(0) 와 반드시 구분한다.
uint8[8] lc                 # lifecycle nibble
uint8[8] hs                 # health nibble
uint8[8] degraded_cnt       # 통신 오염도 [%]
uint8   hw_error            # 현재 활성 에러 비트필드
uint8   hw_fatal            # 재초기화 필요 치명 에러
uint8   hw_reset            # 소프트 리셋 요구 비트필드

# ── 상태 ───────────────────────────────────────────────────
uint8   ecu_state           # ECU sys_state
uint8   control_state       # 0=INIT 1=IDLE 2=RUNNING 3=STREAM 4=LOCKED
                            #   (TestbedState 그대로. 종전 주석은 STREAM 이 빠져 있어
                            #    소비자가 STREAM(3) 을 LOCKED 로 읽게 돼 있었다)
uint8   write_source        # 0=NONE 1=CMD_VEL 2=PROFILE 3=STREAM
uint8   auto_mode           # 현재 유효 auto_mode (write 범위의 근거)
uint8   motor_mask
uint32  goal_id             # 활성 프로파일 (0=없음) — 분석 자동 분할 키
float32 profile_time        # [s]
uint16  segment_index

# ── 명령·피드백 ────────────────────────────────────────────
float32[4] cmd              # 명령값. 단위는 auto_mode 가 결정 (A / RPM / deg)
float32[4] fb_current       # [A]
float32[4] fb_velocity      # [RPM]
float32[4] fb_position      # [deg]
int32[2]   loadcell_raw     # raw (캘리브레이션은 오프라인)
float32[4] imu_quat         # x,y,z,w — control 프리셋만, 없으면 NaN
float32[3] imu_gyro         # [rad/s]
float32[3] imu_acc          # [m/s^2]
float32[5] link_angle       # [deg] linkage encoder — 링크가 몸체에 대해 기울어진 각.
                            #   AS5600 12bit raw x 360/4096. control 프리셋만, 없으면 NaN.
                            #   IMU/GPS 와 함께 global position 추정(MPC)의 입력 (03 §8.2)

# ── RW 트랜잭션 결과 ───────────────────────────────────────
uint8   rw_err              # read_err | write_err<<4 (STM 응답 Data[0])
uint32  drop_cnt            # 발행 큐 누적 드롭 (02 §6.3 Q3) — 0 이면 한 번도 안 막혔다
uint32  irregular_tick_cnt  # out-of-span 등으로 정규 RW 가 대체된 tick 누적
uint32  late_tick_cnt       # 주기(5ms)를 넘겨 위상이 리셋된 tick 누적. tick 번호는 안 건너뛰고
                            #   시간축이 밀린다 — 늘어난 시간의 양 (RunProfile.action 참조)
