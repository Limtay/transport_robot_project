#ifndef ORIN_FIRMWARE_BRIDGE__RD_MAP_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_MAP_HPP_

#include "orin_firmware_bridge/rd_comm.hpp"
#include "orin_firmware_bridge/rd_common.hpp"
#include <cstdint>

#define TIMEOUT_MAX 3

namespace orin_bridge {

//=============== ID Define ======================//
namespace TARGET {
    constexpr uint8_t ORIN = 0x01;
    constexpr uint8_t ECU  = 0xE1;
    constexpr uint8_t DPC  = 0xD2;
    constexpr uint8_t PCU  = 0xA1;
}

namespace FUNC {
    constexpr uint8_t WQ = 0x10; // Write Query
    constexpr uint8_t WR = 0x11; // Write Response
    constexpr uint8_t RQ = 0x20; // Read Query
    constexpr uint8_t RR = 0x21; // Read Response
}
/* ------ IDX Mapping --------
           R/W          RO
ECU     0 -  31    128 - 159    
DPC    64 -  95    192 - 223
PCU    96 - 127    224 - 255 
-----------------------------*/
//=============== Robot State ======================//
typedef enum { // Per motor 
    AK_OK       = 0,
    AK_WARM     = 1,
    AK_TIMEOUT  = 2,
    AK_ERROR    = 3
}Motor_health;

typedef enum { // ECU FSM
    ECU_INIT     = 0,
    ECU_MANUAL   = 1,
    ECU_AUTO     = 2,
    ECU_ESTOP_SW = 3,
    ECU_ESTOP_HW = 4,
    ECU_FAULT    = 5
}ECU_FSM_e;

typedef enum { // System Error Codes
    ERR_NONE    = 0x00U,
    ERR_MOTOR   = 0x01U,
    ERR_SENSOR  = 0x02U,
    ERR_COMM    = 0x03U
}System_Err;

struct CommHealth_t {
    uint16_t alive_time;    // [100ms]
    uint16_t timeout_cnt  = 0;      // 누적 타임아웃 횟수
    bool     is_connected = false;  // 연결 상태 (ROS 2로 퍼블리시 하기 좋음)
};

struct HW_Error_t {
    bool ecu_can;
    bool ecu_i2c;
    bool ecu_uart1;
};

// 1. ECU (주행 제어 보드) 상태
struct EcuState_t {
    // state 
    ECU_FSM_e fsm_state;
    CommHealth_t comm;
    HW_Error_t  hw_err; 
    uint8_t  system_state; 
    bool     is_Estop; 
    bool     is_Mode; 

    Motor_health motor_state[4];
    uint8_t motor_tx_err_cnt[4];
    uint8_t motor_rx_err_cnt[4];

    // RX (Feedback)
    float   motor_pose[4];
    float   motor_speed[4];
    float   motor_current[4];
    int8_t  motor_temp[4];
    int8_t  motor_error[4];
    float   motor_cmd_rpm[4];

    float   fb_linear_x;   
    float   fb_angular_z;
       
    uint16_t linkage_encoder[5];

    int16_t imu_quaternion[4];
    int16_t imu_gyro[3];
    int16_t imu_accel[3];

    // TX (Command)
    float   cmd_linear_x;       
    float   cmd_angular_z;      
    int16_t cmd_motor_rpm[4]; 
};

// 2. DPC (전개 제어 보드) 상태
struct DpcState_t {
    bool cmd_jeongae;     // TX
    bool fb_jeongae;      // RX

    // 통신 상태
    CommHealth_t comm;
};

// 3. PCU (전원 제어 보드) 상태
struct PcuState_t {
    uint32_t battery_voltage; // RX

    // 통신 상태
    CommHealth_t comm;
};

// --- [통합 로봇 상태 구조체] ---
struct RobotState_t {
    EcuState_t ecu;
    DpcState_t dpc;
    PcuState_t pcu;

    mutable std::mutex state_mutex;
};

//=============== RdMap Class define ======================//
class RdMap {
public:
    RdMap();
    ~RdMap();

    // [Public API] 외부(main)에서는 이것만 부름
    RD_RET Encode(uint8_t target_id, uint8_t func_code, uint8_t idx, RobotState_t* state, PACKET_comm_t* pkt);
    RD_RET Decode(PACKET_comm_t* pkt, RobotState_t* state);

private:
    uint8_t GetSourceIdFromIdx(uint8_t idx);
    //=============== Packet IDX Define ======================//
    #pragma pack(push, 1)
    // ECU -------------------------------------------------
    // ---------Read / Write-----------//
    struct EcuStatePkt_t {  // IDX 0: State Read (RQ/RR only)
        static constexpr uint8_t ID = 0;
        uint8_t idx;
        // DATA[0]: status flags
        uint8_t fsm_state : 4;  // bits [0-3] ECU_FSM_e
        uint8_t reserved  : 1;  // bit  [4]
        uint8_t hw_can    : 1;  // bit  [5]
        uint8_t hw_i2c    : 1;  // bit  [6]
        uint8_t hw_uart1  : 1;  // bit  [7]
        // DATA[1]: motor health — 2-bit per motor [M4|M3|M2|M1]
        uint8_t  motor_health;
        // DATA[2-3]: motor tx error count — 4-bit per motor [M4|M3|M2|M1]
        uint16_t motor_tx_err;
        // DATA[4-5]: motor rx error count — 4-bit per motor [M4|M3|M2|M1]
        uint16_t motor_rx_err;
        // DATA[6-7]: alive_time [100ms, little-endian]
        uint16_t alive_time;
    };

    struct EcuCmdVel_t {    // IDX 1: Vellocity Write
        static constexpr uint8_t ID = 1;  uint8_t idx;
        float   lin_x;  
        float   ang_z; 
    };

    struct EcuCmdRpm_t {    // IDX 2: RPM Write
        static constexpr uint8_t ID = 2; uint8_t idx;
        int16_t m[4]; 
    };

    // ---------- Read Only ------------//
    struct EcuFbPose_t {    // IDX 128: FeedBack Pose
        static constexpr uint8_t ID = 128; uint8_t idx;
        int16_t m[4]; 
    };

    struct EcuFbRpm_t {     // IDX 129: FeedBack RPM
        static constexpr uint8_t ID = 129; uint8_t idx;
        int16_t m[4]; 
    };

    struct EcuFbCurrent_t { // IDX 130: FeedBack Current
        static constexpr uint8_t ID = 130; uint8_t idx;
        int16_t m[4]; 
    };

    struct EcuFbMotor_t {   // IDX 131: FeedBack Motor Temp/Err
        static constexpr uint8_t ID = 131; uint8_t idx; 
        int8_t temp[4]; int8_t  err[4];
    };

    struct EcuFbImuQuat_t { // IDX 144: FeadBack IMU Quaternion  
        static constexpr uint8_t ID = 144; uint8_t idx;
        int16_t quat[4]; // [z, y, x, w]
    };

    struct EcuFbImuGyro_t { // IDX 145: FeadBack IMU Gyroscope
        static constexpr uint8_t ID = 145; uint8_t idx;
        int16_t unit[3];  int16_t buffer; // [x, y, z]
    };

    struct EcuFbImuAccel_t {// IDX 146: FeadBack IMU Accelermeter
        static constexpr uint8_t ID = 146; uint8_t idx;
        int16_t unit[3];  int16_t buffer; // [x, y, z]
    };

    struct EcuFbLinkage_t { // IDX 152: FeadBack Linkage Angle
        static constexpr uint8_t ID = 152; uint8_t idx;
        uint64_t ang0 : 12; 
        uint64_t ang1 : 12;
        uint64_t ang2 : 12;
        uint64_t ang3 : 12;
        uint64_t ang4 : 12;
        uint64_t buf  : 4;  // 남는 4비트 (버퍼)
    };
    // DPC -------------------------------------------------
    struct DpcStatePkt_t {  // IDX 64: State Read/Write
        static constexpr uint8_t ID = 64; uint8_t idx; 
        int8_t buffer[8];
    };
    // PCU -------------------------------------------------
    struct PcuStatePkt_t {  // IDX 96: State Read/Write 
        static constexpr uint8_t ID = 96; uint8_t idx; 
        int8_t buffer[8];
    };

    // --- [2] 9바이트 통합 Union ---
    typedef union {
        struct { // Raw View : idx[1] + data[8]
            uint8_t idx;
            uint8_t data[8];
        } raw;
        // ---------ECU---------//
        EcuStatePkt_t      ecu;
        EcuCmdVel_t     vel;
        EcuCmdRpm_t     rpm;

        EcuFbPose_t     fb_pose;
        EcuFbRpm_t      fb_rpm;
        EcuFbCurrent_t  fb_current;
        EcuFbMotor_t    motor;
        EcuFbImuQuat_t  imu_quat;
        EcuFbImuGyro_t  imu_gyro;
        EcuFbImuAccel_t imu_accel;
        EcuFbLinkage_t  linkage;
        // ---------DPC---------//
        DpcStatePkt_t      dpc;
        // ---------PCU---------//
        PcuStatePkt_t      pcu;

    } PacketMap_u;

    #pragma pack(pop)
    //=============== Encode, Decode define ======================//
    // ECU

    RD_RET EncodeEcu(uint8_t func_code, uint8_t idx, EcuState_t* ecu, PacketMap_u* view);
    RD_RET DecodeEcu(uint8_t func_code, uint8_t idx, const PacketMap_u* view, EcuState_t* ecu);

    // DPC
    RD_RET EncodeDpc(uint8_t func_code, uint8_t idx, DpcState_t* dpc, PacketMap_u* view);
    RD_RET DecodeDpc(uint8_t func_code, uint8_t idx, const PacketMap_u* view, DpcState_t* dpc);

    // PCU
    RD_RET EncodePcu(uint8_t func_code, uint8_t idx, PcuState_t* pcu, PacketMap_u* view);
    RD_RET DecodePcu(uint8_t func_code, uint8_t idx, const PacketMap_u* view, PcuState_t* pcu);
    
    struct Encoder {
        //----- ECU ----------//
        static void EcuStatePkg(PacketMap_u* view, const EcuState_t* ecu);
        static void EcuCmdVel(PacketMap_u* view, const EcuState_t* ecu);
        static void EcuCmdRpm(PacketMap_u* view, const EcuState_t* ecu);
        //------DPC ----------//
        static void DpcStatePkg(PacketMap_u* view, const DpcState_t* dpc);
        //------ PCU --------- //
        static void PcuStatePkg(PacketMap_u* view, const PcuState_t* pcu);
    };

    struct Decoder {
        //----- ECU ----------//
        static void EcuStatePkg(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbCmdRpm(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbPose(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbRpm(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbCurrent(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbMotor(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbImuQuat(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbImuGyro(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbImuAccel(const PacketMap_u* view, EcuState_t* ecu);
        static void EcuFbLinkage(const PacketMap_u* view, EcuState_t* ecu);
        //------ DPC --------- //
        static void DpcStatePkg(const PacketMap_u* view, DpcState_t* dpc);
        //------ PCU --------- //
        static void PcuStatePkg(const PacketMap_u* view, PcuState_t* pcu);
    };  
};

} // namespace orin_bridge

#endif