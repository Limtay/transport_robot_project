현재 세션은 이전에 테스트 및 검증완료된 ECU, DPC_B를 바탕으로, 동일한 모듈 구조를 공유하는 신규 PRA모듈에 대한 개발 도우미임.

기타 개발문서 세팅은 기존 ECU와 dpcb_config_wiki documents를 참고하여 신규내용들을 pra_config_wiki에 각종md들로 정리하며 진행할 예정.

프로젝트 개요에 대한 간략한 설명임.

PRA란 베터리입력을 감시하고, 스위칭하는 대용량 세이프티 + 스위치의 구조.
러프한 기능으로는
1. 메인파워(100A급) : 제어용 P,N릴레이, 솔리드스테이트 프리차저 컨트롤 (전압 측정기반 순차제어)
2. 서브파워(5A급) : 보조전원 공급. 2채널 
3. BMS와의 CAN통신
4. 스위치 입력을 통한 AUX 및 메인파워 온오프컨트롤
5. ORIN과의 통신 (rs485 모드버스)
를 수행함.

페리페럴 구성은 아래와 같음.

ADC 센싱 :
	ntc(NTCx_ADCx define) x3
	전압측정(NTCx_ADCx define) x2
	전류측정 x1
GPIO 제어 :
	Solid state precharger(PC_EN) x1
	relay(Rx_EN) x2
    Solid state Relay(AUXx_EN) x2
I2C 스위치 입력 (gpio expander mcp23017) :
	led+spst	x2
통신 :
	CAN1 (bms통신용)	x1
	RS485(usart6, orin통신용) x1

ORIN통신용 RS485는 레지스터맵 기반 rs485 modbus이며, 모든 통신용 에셋 및 모듈은 기존 ECU를 기반으로 작성함.

PRA레지스터맵 초안은 pra_config_wiki/raw/register_map_pra.csv에 저장되있음.



작업시 조건
1. USER CODE BEGIN/END영역 확인하고 적절한 위치만 편집요청. MX갱신시 삭제문제 발생할수 있음.
2. main.c의 Task내부에는 ECU와 동일하게 RD_TASK_xxx()형태의 함수만 작성되어있는 구조로 유지바람. 현재는 RD_TASK_RS485()만 추가바람.

현재 main.c내부에 있는 ADC 수신영역을 rd_peripheral.h와 c로 이관 및 




