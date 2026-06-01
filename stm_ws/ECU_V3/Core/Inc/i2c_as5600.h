/**
 ******************************************************************************
 * @file    i2c_as5600.h
 * @author  Kyeongtae
 * @date    2026-02-14
 * @brief   AS5600 12bit 절대각 자기 엔코더 + TCA9548A I2C MUX 드라이버.
 *
 *  - 다채널 엔코더(예: 5개)를 한 I2C 버스에서 MUX로 분기하여 폴링.
 *  - 핸들 1개 = 엔코더 1개 + 매달린 MUX 채널 번호. I2C 핸들은 의존성 주입.
 *  - 데이터는 raw(0 ~ ENC_RAW_MAX-1)만 노출. 물리량(deg) 변환은 상위 레이어.
 *
 *  에러 처리 정책
 *  -----------------------------------------------------------------------
 *    - MUX 통신 실패 : 즉시 AS5600_ERR_MUX 반환 (모듈 카운터 갱신 없음).
 *                      → 상위 레이어가 MUX 단절을 판정하고 재초기화 등 결정.
 *    - ENC 통신 실패 : err_cnt 1 증가 (0xFFFF에서 saturate) 후 AS5600_ERR_ENC.
 *                      → 상위 레이어는 err_cnt >= ENC_ERR_THRESHOLD 로 fault 판정.
 *    - 성공          : raw_angle 갱신, err_cnt = 0, AS5600_OK 반환.
 *
 *  사용 순서
 *  -----------------------------------------------------------------------
 *    1) AS5600_INIT(&Enc[i], &hi2c1, mux_ch);  // 각 채널마다 1회
 *    2) 주기 태스크(10ms 등)에서:
 *         uint8_t any_mux = 0;
 *         for (i ...) {
 *             if (AS5600_UPDATE(&Enc[i]) == AS5600_ERR_MUX) any_mux = 1;
 *         }
 *         // any_mux, Enc[i].err_cnt 를 상위에서 활용
 ******************************************************************************
 */

#ifndef INC_I2C_AS5600_H_
#define INC_I2C_AS5600_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
/* --- I2C 주소 (7bit << 1, HAL 형식) --------------------------------------- */
#define AS5600_I2C_ADDR    (0x36 << 1)  /**< AS5600 고정 주소                  */
#define I2C_MUX_ADDR       (0x70 << 1)  /**< TCA9548A 기본 주소                */

/* --- AS5600 레지스터 (datasheet) ------------------------------------------ */
#define AS5600_REG_RAW_ANGLE  0x0C  /**< 필터링 안 된 12bit 각도               */
#define AS5600_REG_ANGLE      0x0E  /**< 필터링/히스테리시스 적용된 12bit 각도 */

/* --- 데이터 한계 / 에러 임계 ---------------------------------------------- */
#define ENC_RAW_MAX        4096     /**< AS5600 12bit → 0 ~ 4095               */
#define ENC_ERR_THRESHOLD  5        /**< err_cnt 이 값 이상이면 fault 판정     */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief AS5600_UPDATE 결과 코드.
 *
 *  AS5600_ERR_MUX 는 항상 caller가 처리해야 함 (모듈은 카운터를 안 올림).
 *  AS5600_ERR_ENC 는 모듈이 err_cnt 를 자동 증가시킴 (caller는 무시 가능).
 */
typedef enum {
    AS5600_OK = 0,    /**< 정상 — raw_angle 갱신됨                            */
    AS5600_ERR_ENC,   /**< 엔코더 read 실패 — err_cnt 증가됨, 일시적 에러일 수 있음 */
    AS5600_ERR_MUX    /**< MUX 채널 선택 실패 — 상위 레이어 처리 필요         */
} AS5600_Status_e;

/**
 * @brief 엔코더 1채널 핸들. 배열로 N개 선언해 다채널 사용.
 *
 *  volatile: 주기 태스크와 다른 컨텍스트(예: 통신 응답)에서 raw_angle을
 *  읽을 수 있으므로 일관성 확보용.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;         /**< 사용할 I2C 핸들 (의존성 주입)         */
    uint8_t            mux_channel;  /**< MUX 채널 번호 (0 ~ 7)                 */
    volatile uint16_t  raw_angle;    /**< 0 ~ ENC_RAW_MAX-1 (12bit raw)         */
    volatile uint16_t  err_cnt;      /**< 엔코더 연속 read 실패 횟수 (saturate) */
} AS5600_Handle_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  엔코더 핸들 초기화. 통신은 하지 않음 (단순 필드 세팅).
 * @param  pEncoder      대상 핸들
 * @param  hi2c          이 엔코더가 매달릴 I2C 핸들
 * @param  mux_channel   MUX 채널 번호 (0~7)
 */
void AS5600_INIT(AS5600_Handle_t *pEncoder, I2C_HandleTypeDef *hi2c, uint8_t mux_channel);

/**
 * @brief  MUX 채널을 선택하고 AS5600에서 각도 1개를 읽어 raw_angle 갱신.
 *         주기 태스크에서 채널별로 호출.
 * @param  pEncoder  대상 핸들
 * @retval AS5600_OK       성공 (raw_angle 갱신, err_cnt 리셋)
 * @retval AS5600_ERR_ENC  엔코더 read 실패 (err_cnt 증가됨)
 * @retval AS5600_ERR_MUX  MUX 선택 실패 (상위 레이어에서 처리 필요)
 */
AS5600_Status_e AS5600_UPDATE(AS5600_Handle_t *pEncoder);

#endif /* INC_I2C_AS5600_H_ */
