BMS can통신 프로토콜

CAN 2.0B, 250k

상위 컴퓨터 송신
선순위+데이터ID+BMS주소+호스트주소
0x 18 10 01 40
데이터 8bytes (요청시에는 00으로 8바이트 전송)

슬레이브 응답
선순위+데이터ID+호스트주소+BMS주소
0x 18 10 40 01
데이터 8bytes

즉, 호스트주소는 상시 40, bms주소 상시 01.
실제로 달라지는 부분은 데이터 ID인 18부분만.

데이터 ID
전압 및 전류 SOC : 0x90
Byte0~byte1: 총 전압（0.1V）Cumulative total voltage (0.1 V)
Byte2~byte3: 측정 총 전압（0.1V）Gather total voltage (0.1 V) (예비 기능 Reserved function)
Byte4~byte5 : 전류 （30000 Offset，0.1A）
Byte6-Byte7：SOC（0.1%）

최고 및 최저전압 : 0x91
Byte0~byte1: 최고 셀 전압값（mV）Maximum cell voltage value (mV)
Byte2: 최고 셀 전압 셀 번호 No of cell with Maximum voltage
Byte3~byte4: 최저 셀 전압값（mV）Minimum cell voltage value (mV)
Byte5: 최저 전압 셀 번호 No of cell with Minimum voltage

최고 및 최저온도 : 0x92
Byte0: 최고 셀 온도값（40 Offset，°C）Maximum temperature value (40 Offset ,°C)
Byte1: 최고 온도 셀 번호 Maximum temperature cell No
Byte2: 최저 셀 온도값（40 Offset，°C）Minimum temperature value (40 Offset ,°C)
Byte3: 최저 온도 셀 번호 Minimum temperature cell No

충방전 MOS상태 : 0x93
Byte0: 충방전상태（0 정지，1 충전，2 방전）State (0 stationary 1 charge 2 discharge)
Byte1: 충전 MOS 상태 Charge MOS state
Byte2: 방전 MOS 상태 Discharge MOS status
Byte3: BMS life（0~255사이클）(0~255 cycles)
Byte4~Byte7: 잔여 용량（mAH）Remain capacity (mAH)

상태정보 1 : 0x94
Byte0: 배터리 직렬 개수 No of battery string
Byte1: 온도센서 개수 No of Temperature
Byte2: 충전기 상태（0 분리，1 연결）Charger status (0 disconnect 1 access)
Byte3: 부하상태（0 분리，1 연결）Load status (0 disconnect 1 access)
Byte4：
수신
Received
Bit 0: DI1 state
Bit 1: DI2 state
Bit 2: DI3 state
Bit 3: DI4 state
Bit 4: DO1 state
Bit 5: DO2 state
Bit 6: DO3 state
Bit 7: DO4 state
Byte 5~Byte 7: Reserved

셀 개별전압 1~48 : 0x95
각 셀 전압은 2 바이트를 차지하며, 실제 셀 개수에 따라 전송된다. 최대 96 바이트이며 16 프레임으로 나누어 전송한다.
The voltage of each monomer is 2 byte, according to the actual number of cell, the maximum 96 byte, is sent in 16 frames
Byte0: 프레임 번호 (0부터 시작，0xFF는 무효) frame number, starting from 0,0xFF invalid
Byte1~byte6: 셀 전압（1mV）Cell voltage (1 mV)
Byte7: Reserved

셀온도1~16 : 0x96
각 온도값은 1바이트를 차지하며, 실제 사용 중인 온도 센서 개수에 따라 전송된다. 최대 21바이트이며 3개의 프레임으로 나누어 전송한다.
Each temperature accounts for 1 byte, according to the actual number of temperature send, the maximum 21 byte, send in 3
frames
Byte0: 프레임 번호，0부터 시작 frame number, starting at 0
Byte1~byte7: 셀 온도（40 Offset，°C）cell temperature(40 Offset ,℃)

셀 밸런스 상태 : 0x97
0: 꺼짐 1: 켜짐 0 : Closed 1 : Open
Bit0: 1번 셀 밸런싱 여부(ON) 여부
Cell 1 balance state
Bit47: 셀 48 밸런싱 상태 Cell 48 balance state
Bit48~Bit63: reserved

베터리 고장상태 : 0x98
0 → No error
1 → Error

Byte 0
Bit 0: 셀 과전압 1단계 경고 Cell volt high level 1
Bit 1: 셀 과전압 2단계 경고 Cell volt high level 2
Bit 2: 셀 저전압 1단계 경고 Cell volt low level 1
Bit 3: 셀 저전압 2단계 경고 Cell volt low level 2
Bit 4: 팩 과전압 1단계 경고 Sum volt high level 1
Bit 5: 팩 과전압 2단계 경고 Sum volt high level 2
Bit 6: 팩 저전압 1단계 경고 Sum volt low level 1
Bit 7: 팩 저전압 2단계 경고 Sum volt low level 2

Byte 1
Bit 0: 충전 과온도 1단계 경고 Chg temp high level 1
Bit 1: 충전 과온도 2단계 경고 Chg temp high level 2
Bit 2: 충전 저온도 1단계 경고 Chg temp low level 1
Bit 3: 충전 저온도 2단계 경고 Chg temp low level 2
Bit 4: 방전 과온도 1단계 경고 Dischg temp high level 1
Bit 5: 방전 과온도 2단계 경고 Dischg temp high level 2
Bit 6: 방전 저온도 1단계 경고 Dischg temp low level 1
Bit 7: 방전 저온도 2단계 경고 Dischg temp low level 2

Byte 2
Bit 0: 충전 과전류 1단계 경고 Chg overcurrent level 1
Bit 1: 충전 과전류 2단계 경고 Chg overcurrent level 2
Bit 2: 방전 과전류 1단계 경고 Dischg overcurrent level 1
Bit 3: 방전 과전류 2단계 경고 Dischg overcurrent level 2
Bit 4: SOC 높음 1단계 경고 SOC high level 1
Bit 5: SOC 높음 2단계 경고 SOC high level 2
Bit 6: SOC 낮음 1단계 경고 SOC Low level 1
Bit 7: SOC 낮음 2단계 경고 SOC Low level 2

Byte 3 Bit 0: 전압 편차 과대 1단계 경고 Diff volt level 1 Bit 1: 전압 편차 과대 2단계 경고 Diff volt level 2 Bit 2: 온도 편차 과대 1단계 경고 Diff temp level 1 Bit 3: 온도 편차 과대 2단계 경고 Diff temp level 2 Bit 4~Bit7：Reserved Byte 4 Bit 0: 충전 MOS 과온도 경고 Chg MOS temp high alarm Bit 1: 방전 MOS 과온도 경고 Dischg MOS temp high alarm Bit 2: 충전 MOS 온도 센서 오류 Chg MOS temp sensor err Bit 3: 방전 MOS 온도 센서 오류 Dischg MOS temp sensor err Bit 4: 충전 MOS 접착 오류 Chg MOS adhesion err Bit 5: 방전 MOS 접착 경고 Dischg MOS adhesion err Bit 6: 충전 MOS 단선 오류 Chg MOS open circuit err Bit 7: 방전 MOS 단선 오류 Discrg MOS open circuit err Byte 5 Bit 0: AFE 수집 칩 오류 AFE collect chip err Bit 1: 셀 전압 수집 단선 오류 Voltage collect dropped Bit 2: 셀 온도 센서 오류 Cell temp sensor err Bit 3: EEPROM 저장 오류 EEPROM err Bit 4: RTC 시계 오류 RTC err Bit 5: 프리차지 실패 Precharge failure Bit 6: 통신 오류 Communication failure Bit 7: 내부 네트워크 통신 모듈 오류 Internal communication failure Byte6 Bit 0: 전류 모듈 오류 Current module fault Bit 1: 팩 전압 검출 모듈 오류 Sum voltage detect fault Bit 2: 단락(쇼트) 보호 오류 Short circuit protect fault Bit 3: 저전압 충전 금지 오류 Low volt forbidden chg fault Bit4-Bit7: Reserved Byte7：고장 코드 Fault code

**실제 pcan 통신 실기**


type : ID / DLC / DATA
	
	send      : 18900140h / 8 / 00 00 00 00 00 00 00 00
	receive   : 18904001h / 8 / 02 1A 00 00 75 30 02 A8
	
	send      : 18910140h / 8 / 00 00 00 00 00 00 00 00
	receive   : 18914001h / 8 / 0F 0B 02 0F 09 01 00 00
	
	send      : 18920140h / 8 / 00 00 00 00 00 00 00 00
	receive   : 18924001h / 8 / 40 01 40 01 00 00 00 00
	
	send      : 18930140h / 8 / 00 00 00 00 00 00 00 00
	receive   : 18934001h / 8 / 00 01 01 A5 00 00 5C F8
	
	send      : 18980140h / 8 / 00 00 00 00 00 00 00 00
	receive   : 18984001h / 8 / 00 00 00 00 00 00 00 00