# INST_POS 런 피드백 (redesign/12 §6) — /carrier/control/pos_feedback, 200Hz
#
# position_inst 기동에서만 나간다. 응답 1건 = 메시지 1건이고, 같은 tick 의 ControlFeedback 과
# header.stamp·ecu_tick 이 같다. ControlFeedback 은 모터 좌표 그대로이고 절대값은 여기 있다.
# ControlFeedback 정의를 늘리지 않는 이유: 옛 bag 분석이 메시지 정의로 디코드한다 (12 Q3).
std_msgs/Header header
uint32     ecu_tick           # x0.1ms
uint32     goal_id            # 0 = 재생 없음
float32    profile_time       # [s]

float64[4] cmd_abs            # [deg] 절대 위치 명령 (IDLE·LOCKED 는 유지 목표). 좌표 무효면 NaN
float64[4] fb_abs             # [deg] offset + fb. PEND 구간은 직전 값
float64[4] offset             # [deg] 누적 원점 이동량
bool       fb_abs_held        # 이번 샘플이 PEND 로 유지된 값인가

uint8[4]   origin_seq
int16[4]   pre_origin_pose    # x0.1 deg
uint8      origin_flag        # bit0~3 PEND / bit4~7 FRAME_LOST

uint32[3]  rls_raw            # psi1, psi2 (18bit) / theta3 (14bit)
float32[3] rls_deg            # [deg] 해당 채널 Error 면 NaN
uint8      rls_ew             # 2bit x4 (b1=Error, b0=Warning), LSB=ch0
uint8      rls_state          # STATE_t

int32[2]   lin_xy             # [count] 채널 Error 면 INT32_MIN (0 은 유효값이다)
uint8      lin_zero_result    # 0=없음 1=성공 2=보드 실패 3=무응답 4=형식 불일치 5=잘못된 요청
uint8      lin_zero_seq

uint8      odiag_motor        # 마지막 원점 이벤트 모터 idx (0xFF=없음)
uint8      odiag_dt           # x0.1ms 원점 TX - fb_old 취득
int16      odiag_e_track      # x0.1 deg
int16      odiag_residual     # x0.1 deg
int16      odiag_vel          # AK raw
