/*
 * rd_define.h
 *
 *  Created on: Aug 12, 2025
 *      Author: abc01
 */

#ifndef INC_RD_DEFINE_H_
#define INC_RD_DEFINE_H_

/* AS5600 I2C 엔코더 사용 여부. 0 이면 I2C 를 전혀 읽지 않는다 (position 실험 빌드).
 * project 모드는 엔코더가 필요하므로 1 로 빌드할 것. */
#define RD_USE_I2C_ENCODER 1

/* 자율 원점 이동(SET_ORIGIN) 사용 여부. 0 이면 SET_ORIGIN 을 보내지 않는다 — 점프 가드·과속 제동은 유지.
 * 2026-11-05 실기: 이동 중 SET_ORIGIN 직후 AK 가 위치 명령을 따르지 않고 ~900°/s 로 달렸다.
 * 원인 확정 전까지 0 으로 둔다 (redesign/11 §8 T2·T3). */
#define RD_USE_AUTO_ORIGIN 1

/* SET_ORIGIN 타이밍 시험 빌드 (2026-11-05). 1 이면 원점 기준을 50° 로 낮추고 cmd 보정 전후 값을
 * POS_FB.odiag_* 로 내보낸다. 실패해도 약 50° 만 벗어나게 하려는 것이다. 시험 후 0 으로 되돌릴 것. */
#define RD_ORIGIN_TEST 1

#endif /* INC_RD_DEFINE_H_ */
