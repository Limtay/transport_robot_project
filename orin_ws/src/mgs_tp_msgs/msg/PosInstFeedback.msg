# position_inst 기동의 유일한 200Hz 토픽 — /carrier/control/pos_feedback (redesign/14)
#
# ECU POS_FB 페이지(addr 256~345, 90B) 전체 + 브리지가 보낸 위치 명령 + 브리지 상태.
# 응답 1건 = 메시지 1건. position_inst 에서는 ControlFeedback·상태 토픽을 내지 않는다.
# PosFeedback(구 정의)은 옛 bag 을 읽기 위해 그대로 둔다 — 이 타입은 2026-11-08 이후 bag 이다.
std_msgs/Header header        # ControlFeedback 과 같은 규칙: ECU 취득 시각의 Orin 시간축 변환값
uint32     ecu_tick           # POS_FB realtime_tick [x0.1ms]
bool       stamp_valid        # 시계 추정기 수렴 여부 (false 면 header.stamp = Orin 수신 시각)

# ── 브리지 상태 ──
uint8      control_state      # 0=INIT 1=IDLE 2=RUNNING 3=STREAM 4=LOCKED
uint8      write_source       # 0=NONE 1=CMD_VEL 2=PROFILE 3=STREAM
uint8      auto_mode
uint8      motor_mask
uint32     goal_id            # 0 = 재생 없음
float32    profile_time       # [s]
uint16     segment_index

# ── 위치 명령 ──
float32[4] cmd                # [deg] ECU 에 쓴 cmd_position (모터 좌표, 브리지가 마지막으로 보낸 값)
float64[4] cmd_abs            # [deg] 절대 위치 명령. 좌표 무효면 NaN
float64[4] fb_abs             # [deg] offset + fb. PEND 구간은 직전 값
float64[4] offset             # [deg] 누적 원점 이동량
bool       fb_abs_held        # 이번 샘플이 PEND 로 유지된 값인가

# ── POS_FB (addr 256~345) — 원값 ──
uint8      rs485_proc_delta   # 260 [x0.1ms] 직전 트랜잭션 처리시간
uint8      ecu_state          # 261 sys_state (SYSTEM_STATE_e)
uint8      hw_reset           # 262
uint8      origin_flag        # 263 bit0~3 PEND / bit4~7 FRAME_LOST
uint8[8]   degraded_cnt       # 264 [%]
int16[4]   mtr_position       # 272 [x0.1 deg] 모터 좌표
int16[4]   mtr_velocity       # 280 AK raw (x10 ERPM)
int16[4]   mtr_current        # 288 [x0.01 A]
int16[4]   pre_origin_pose    # 296 [x0.1 deg]
uint8[4]   origin_seq         # 304
int32[2]   loadcell_raw       # 308 raw 0~4095 (uint16 → int32. 미판독 sentinel 없음 — POS_FB 는 항상 채워진다)
uint8      loadcell_state     # 312 STATE_t
uint8      motor_state        # 313 STATE_t (4 모터 worst)
uint8      rls_state          # 314 STATE_t
uint8      rls_ew             # 315 2bit x4 (b1=Error, b0=Warning), LSB=ch0
uint32[3]  rls_raw            # 316 psi1, psi2, theta3 — 모두 AksIM 18bit (theta3 은 2026-11-08 Orbis 에서 교체)
uint8      lin_zero_result    # 328 0=없음 1=성공 2=보드 실패 3=무응답 4=형식 불일치 5=잘못된 요청
uint8      lin_zero_seq       # 329
int32[2]   lin_xy             # 330 [count] x 10 um, y 20 um. 채널 Error 면 INT32_MIN
uint8      odiag_motor        # 338 마지막 원점 이벤트 모터 idx (0xFF=없음)
uint8      odiag_dt           # 339 [x0.1ms] 원점 TX - fb_old 취득
int16      odiag_e_track      # 340 [x0.1 deg]
int16      odiag_residual     # 342 [x0.1 deg]
int16      odiag_vel          # 344 AK raw

# ── POS_FB 환산값 (브리지 계산) ──
float32[4] fb_position        # [deg]  mtr_position x 0.1
float32[4] fb_velocity        # [RPM]  mtr_velocity x 10
float32[4] fb_current         # [A]    mtr_current x 0.01
float32[3] rls_deg            # [deg]  해당 채널 Error 면 NaN

# ── RW 트랜잭션 결과 ──
uint8      rw_err             # read_err | write_err<<4
uint32     drop_cnt           # 발행 큐 누적 드롭
uint32     irregular_tick_cnt
uint32     late_tick_cnt
