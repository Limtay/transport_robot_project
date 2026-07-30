/*
 * rd_paxini_sc.c
 *
 *  Created on: Feb 25, 2026
 *      Author: abc01
 */


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_paxini_sc.h"
#include <string.h>

/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
static uint8_t s_fm_spi_buff[SPI_BUFF_SIZE];

static const unsigned char auchCRCLo[] = { //crc table
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static const float DP_M2826_POS[127][3] = { //sensor node position data
    {-12.313811f, -3.889191f, 3.134789f}, 	{-12.456434f, -3.616474f, -0.301605f}, 	{-12.597844f, -0.081198f, 3.505195f},
    {-12.755421f, 0.258216f, 0.030674f}, 	{-12.659332f, 2.979382f, 3.780670f}, 	{-12.826631f, 3.366201f, 0.299772f},
    {-12.460141f, 6.815793f, 4.087254f}, 	{-12.642313f, 7.246576f, 0.645080f}, 	{-11.917985f, -3.975822f, 5.688293f},
    {-12.157673f, -0.252295f, 6.095944f}, 	{-12.186177f, 2.738893f, 6.378841f}, 	{-11.955834f, 6.489783f, 6.664497f},
    {-12.017659f, 9.866876f, 4.306283f}, 	{-12.227310f, 10.325090f, 0.936300f},	{-11.502394f, 9.474796f, 6.826042f},
    {-11.042854f, 13.603986f, 4.520883f},	{-11.302493f, 14.094957f, 1.300573f},   {-10.522602f, -3.642387f, 8.794919f},
    {-10.615047f, -0.185012f, 9.195008f},	{-10.551207f, 2.589759f, 9.432227f},    {-10.264641f, 6.054439f, 9.619334f},
    {-10.541630f, 13.138115f, 6.917870f},	{-9.836953f, 8.806199f, 9.677464f},    	{-9.898257f, 16.472438f, 1.661365f},
    {-10.189288f, 16.997747f, 0.299517f},	{-9.020604f, 12.189320f, 9.772701f},    {-9.438734f, 15.957121f, 5.827312f},
    {-8.057961f, -2.923875f, 11.806355f},	{-8.091411f, -0.179318f, 12.157278f},   {-8.008019f, 3.275681f, 12.335086f},
    {-7.816937f, 6.044703f, 12.483919f},	{-8.120700f, 14.815249f, 8.942833f},    {-7.878103f, 19.730675f, 1.497315f},
    {-8.186925f, 20.331138f, 0.383318f},	{-7.397442f, 9.489304f, 12.544885f},    {-7.514339f, 12.054042f, 11.273590f},
    {-7.517169f, 19.155096f, 4.590410f},	{-6.629962f, 17.892595f, 8.237875f},    {-5.792100f, -2.440694f, 13.946912f},
    {-5.801851f, 0.276528f, 14.264332f},	{-5.730534f, 3.702928f, 14.397524f},    {-5.587247f, 6.454509f, 14.519061f},
    {-5.450397f, 15.639864f, 11.123758f},	{-5.443799f, 21.567499f, 1.547778f},    {-5.716668f, 22.462917f, 0.505506f},
    {-5.275022f, 9.879856f, 14.552585f},	{-5.460338f, 13.146091f, 13.153716f},   {-3.905795f, 20.565057f, 6.494983f},
    {-3.435865f, 18.879481f, 10.440789f},	{-2.514202f, -2.090400f, 15.987600f},   {-2.512046f, 0.611635f, 16.304178f},
    {-2.474295f, 4.013354f, 16.445982f},	{-2.409857f, 6.744343f, 16.555151f},    {-2.272163f, 10.145595f, 16.631941f},
    {-2.751358f, 13.463690f, 15.262593f},	{-3.066467f, 16.013129f, 13.740088f},   {-3.092160f, 22.401417f, 2.212547f},
    {-3.269263f, 23.406842f, 0.812631f},	{0.000003f, -2.028374f, 16.599737f},    {-0.000002f, 1.356942f, 16.914286f},
    {-0.000004f, 4.073691f, 17.046186f},	{-0.000004f, 7.472583f, 17.153095f},    {-0.000008f, 10.182078f, 17.234316f},
    {-0.000001f, 13.530387f, 17.291399f},	{-0.000008f, 16.142463f, 16.119743f},   {-0.000004f, 19.178586f, 13.689209f},
    {-0.000006f, 21.535349f, 9.989663f},	{-0.000004f, 22.701959f, 6.638354f},    {-0.000004f, 23.755341f, 2.686938f},
    {2.514202f, -2.090400f, 15.987600f},	{2.512046f, 0.611635f, 16.304178f},    	{2.474295f, 4.013354f, 16.445982f},
    {2.409857f, 6.744343f, 16.555151f},		{2.272163f, 10.145595f, 16.631941f},    {2.751358f, 13.463690f, 15.262593f},
    {3.066467f, 16.013129f, 13.740088f},	{3.092160f, 22.401417f, 2.212547f},    	{3.435865f, 18.879481f, 10.440789f},
    {3.269263f, 23.406842f, 0.812631f},		{3.905795f, 20.565057f, 6.494983f},    	{5.730534f, 3.702928f, 14.397524f},
    {5.587247f, 6.454509f, 14.519061f},		{5.275022f, 9.879856f, 14.552585f},    	{5.460338f, 13.146091f, 13.153716f},
    {5.792100f, -2.440694f, 13.946912f},	{5.801851f, 0.276528f, 14.264332f},    	{5.450397f, 15.639864f, 11.123758f},
    {5.443799f, 21.567499f, 1.547778f},    	{5.716668f, 22.462917f, 0.505506f},    	{6.629962f, 17.892595f, 8.237875f},
    {7.397442f, 9.489304f, 12.544885f},    	{7.514339f, 12.054042f, 11.273590f},    {7.517169f, 19.155096f, 4.590410f},
    {8.057961f, -2.923875f, 11.806355f},    {8.091411f, -0.179318f, 12.157278f},    {8.008019f, 3.275681f, 12.335086f},
    {7.816937f, 6.044703f, 12.483919f},    	{8.120700f, 14.815249f, 8.942833f},    	{7.878103f, 19.730675f, 1.497315f},
    {8.186925f, 20.331138f, 0.383318f},    	{9.020604f, 12.189320f, 9.772701f},    	{9.438734f, 15.957121f, 5.827312f},
    {10.264641f, 6.054439f, 9.619334f},    	{9.836953f, 8.806199f, 9.677464f},    	{9.898257f, 16.472438f, 1.661365f},
    {10.522602f, -3.642387f, 8.794919f},   	{10.551207f, 2.589759f, 9.432227f},    	{10.541630f, 13.138115f, 6.917870f},
    {10.189288f, 16.997747f, 0.299517f},    {10.615047f, -0.185012f, 9.195008f},    {11.502394f, 9.474796f, 6.826042f},
    {11.042854f, 13.603986f, 4.520883f},    {11.302493f, 14.094957f, 1.300573f},    {11.917985f, -3.975822f, 5.688293f},
    {12.313811f, -3.889191f, 3.134789f},    {12.157673f, -0.252295f, 6.095944f},    {12.186177f, 2.738893f, 6.378841f},
    {11.955834f, 6.489783f, 6.664497f},    	{12.456434f, -3.616474f, -0.301605f},   {12.460141f, 6.815793f, 4.087254f},
    {12.017659f, 9.866876f, 4.306283f},    	{12.597844f, -0.081198f, 3.505195f},    {12.642313f, 7.246576f, 0.645080f},
    {12.227310f, 10.325090f, 0.936300f},    {12.755421f, 0.258216f, 0.030674f},    	{12.659332f, 2.979382f, 3.780670f},
    {12.826631f, 3.366201f, 0.299772f}
};

/* Exported function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/
/* --- precious delay (Nops base) --- */
static void delay_us(uint32_t us) {
    uint32_t count = us * (SystemCoreClock / 1000000) / 10;
    while(count--) { __NOP(); }
}

/* --- CS ctrl (3-bit Logic) --- */
static void Paxini_Select_Node(uint8_t node_idx) {
    GPIO_PinState s1 = GPIO_PIN_SET, s2 = GPIO_PIN_SET, s3 = GPIO_PIN_SET;

    switch (node_idx) {
        case 1: s3=0; s2=0; s1=1; break; // 001
        case 2: s3=0; s2=1; s1=0; break; // 010
        case 3: s3=0; s2=1; s1=1; break; // 011
        case 4: s3=1; s2=0; s1=0; break; // 100
        case 5: s3=1; s2=0; s1=1; break; // 101
        case 6: s3=1; s2=1; s1=0; break; // 110
        case 0: default: s3=1; s2=1; s1=1; break; // 111 (Idle/All High)
    }
    HAL_GPIO_WritePin(PAX_CS1_PORT, PAX_CS1_PIN, s1);
    HAL_GPIO_WritePin(PAX_CS2_PORT, PAX_CS2_PIN, s2);
    HAL_GPIO_WritePin(PAX_CS3_PORT, PAX_CS3_PIN, s3);
}

/* --- CRC8 calc --- */
static uint8_t crc8_cal(const uint8_t *puchMsg, uint32_t usDataLen) {
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    while (usDataLen--) {
        uIndex = uchCRCLo ^ *puchMsg++;
        uchCRCLo = auchCRCLo[uIndex];
    }
    return uchCRCLo;
}

/* --- Force/torque processing --- */
void process_tactile_data(uint8_t* pData, PAXINI_PACKET_t* tactile) {
    float sum_fx = 0;
    float sum_fy = 0;
    float sum_fz = 0;
    float sum_mx = 0;
    float sum_my = 0;
    float sum_mz = 0;
    uint8_t active_nodes = 0;

    for (int i = 0; i < 127; i++) {
    	float temp_fx = (float)((int8_t)pData[i * 3 + 1]);
    	float temp_fy = (float)((int8_t)pData[i * 3 + 2]);
    	float temp_fz = ((float)pData[i * 3 + 3]); //uint8_t casting

    	//float temp_F = sqrt(temp_fx^2 + temp_fy^2 + temp_fz^2);

        if (temp_fz > Force_THRESHOLD) {
        	active_nodes++;

            sum_fx += temp_fx;																	//F_x
            sum_fy += temp_fy;																	//F_y
            sum_fz += temp_fz;																	//F_z
            sum_mx += DP_M2826_POS[i][1]*(float)temp_fz + DP_M2826_POS[i][2]*(float)temp_fy;	//tau_x = y * Fz - z * Fy;
            sum_my += DP_M2826_POS[i][2]*(float)temp_fx + DP_M2826_POS[i][0]*(float)temp_fz;	//tau_y = z * Fx - x * Fz;
            sum_mz += DP_M2826_POS[i][0]*(float)temp_fy + DP_M2826_POS[i][1]*(float)temp_fx;	//tau_z = x * Fy - y * Fx;

        }
    }
    tactile->contact_num = active_nodes;

	if (active_nodes > 0) {
		//meanable contact
		tactile->GET_SUM.Fx = (float)(sum_fx / active_nodes);
		tactile->GET_SUM.Fy = (float)(sum_fy / active_nodes);
		tactile->GET_SUM.Fz = (float)(sum_fz / active_nodes);
		tactile->GET_SUM.Mx = (float)(sum_mx / active_nodes);
		tactile->GET_SUM.My = (float)(sum_my / active_nodes);
		tactile->GET_SUM.Mz = (float)(sum_mz / active_nodes);
	} else {
		// no contact
		tactile->GET_SUM.Fx = 0.0f;
		tactile->GET_SUM.Fy = 0.0f;
		tactile->GET_SUM.Fz = 0.0f;
		tactile->GET_SUM.Mx = 0.0f;
		tactile->GET_SUM.My = 0.0f;
		tactile->GET_SUM.Mz = 0.0f;
	}
}

/* --- INIT --- */
void Paxini_Init(void) {
    Paxini_Select_Node(0); // all pin high to boot spi mode
    delay_us(100);
}

/* --- READ ---*/
RD_RET Paxini_Read(uint8_t node_idx, uint8_t func_code, uint16_t addr, uint16_t len, uint8_t *p_out_data) {
    uint16_t total;

    // input error check
    if (p_out_data == NULL) return RET_NOK;
    if (len > (SPI_BUFF_SIZE - 10)) return RET_NOK;

    // header config
    s_fm_spi_buff[0] = func_code | (1 << 7);
    // LSB
    *(uint16_t*)(s_fm_spi_buff + 1) = addr;
    *(uint16_t*)(s_fm_spi_buff + 3) = len;

    // select node
    Paxini_Select_Node(node_idx);
    delay_us(10);

    // send mosi
    HAL_SPI_Transmit(PAX_SPI, s_fm_spi_buff, HEAD_LEN, 100);
    delay_us(30);

    // receive miso
    HAL_SPI_Receive(PAX_SPI, s_fm_spi_buff + HEAD_LEN, len + 2, 500);

    // unselect node
    delay_us(30);
    Paxini_Select_Node(0);
    delay_us(10);

    // check crc
    total = HEAD_LEN + 1 + len + 1;
    if (crc8_cal(s_fm_spi_buff, total) != 0) {
        return RET_NOK; // crc error return
    }

    memcpy(p_out_data, s_fm_spi_buff + HEAD_LEN, len + 1);

    return RET_OK;
}

/* --- WRITE --- */
void Paxini_Write(uint8_t node_idx, uint8_t func_code, uint16_t addr, uint16_t len, uint8_t *pdata) {
    uint16_t total = HEAD_LEN + len + 1; // Header(5) + Data(len) + CRC(1)

    if (total > SPI_BUFF_SIZE) return;

    // Write: MSB is 0
    s_fm_spi_buff[0] = func_code & (~(1 << 7));
    memcpy(s_fm_spi_buff + 1, &addr, 2);
    memcpy(s_fm_spi_buff + 3, &len, 2);

    memcpy(s_fm_spi_buff + HEAD_LEN, pdata, len);

    // Add CRC8
    uint8_t crc = crc8_cal(s_fm_spi_buff, total - 1);
    s_fm_spi_buff[total - 1] = crc;

    // select node
    Paxini_Select_Node(node_idx);
    delay_us(10);

    // transmit
    HAL_SPI_Transmit(PAX_SPI, s_fm_spi_buff, total, 500);

    // unselect node after 30us
    delay_us(30);
    Paxini_Select_Node(0);
    delay_us(10);
}
