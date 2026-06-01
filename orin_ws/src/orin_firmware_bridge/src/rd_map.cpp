#include "orin_firmware_bridge/rd_map.hpp"
#include <iostream>
#include <cstring> // std::memset 용도

namespace orin_bridge {

RdMap::RdMap() {}
RdMap::~RdMap() {}

/*============================ Public =========================*/
RD_RET RdMap::Encode(uint8_t target_id, uint8_t func_code, uint8_t idx, 
                     RobotState_t* state, PACKET_comm_t* pkt) {
    pkt->tx.TargetID     = target_id;
    pkt->tx.FunctionCode = func_code;
    pkt->tx.Index        = idx;
    
    PacketMap_u* view = reinterpret_cast<PacketMap_u*>(&pkt->tx.Index);
    
    std::lock_guard<std::mutex> lock(state->state_mutex);

    switch (target_id) {    // Target ID task 분배
        case TARGET::ECU: return EncodeEcu(func_code, idx, &state->ecu, view);
        case TARGET::DPC: return EncodeDpc(func_code, idx, &state->dpc, view);
        case TARGET::PCU: return EncodePcu(func_code, idx, &state->pcu, view);
        default: 
            std::cerr << "[Map Error] Encode: Unknown Target ID 0x" 
                      << std::hex << std::uppercase << (int)target_id << std::dec << std::endl;
            return RD_ERROR;
    }
}

RD_RET RdMap::Decode(PACKET_comm_t* pkt, RobotState_t* state) {
    uint8_t target_id = pkt->rx.TargetID; 
    uint8_t func_code = pkt->rx.FunctionCode;
    uint8_t idx       = pkt->rx.Index;

    if (target_id != TARGET::ORIN) { // 수신 패킷의 Target ID가 ORIN이 아닌 경우 (잘못된 패킷)
        std::cerr << "[Map Error] Decode: Invalid Target ID 0x" 
                      << std::hex << std::uppercase << (int)target_id << std::dec << std::endl;
        return RD_ERROR;
    }
    uint8_t source_id = GetSourceIdFromIdx(idx);

    const PacketMap_u* view = reinterpret_cast<const PacketMap_u*>(&pkt->rx.Index);
    
    std::lock_guard<std::mutex> lock(state->state_mutex);
    switch (source_id) {
        case TARGET::ECU: return DecodeEcu(func_code, idx, view, &state->ecu);
        case TARGET::DPC: return DecodeDpc(func_code, idx, view, &state->dpc);
        case TARGET::PCU: return DecodePcu(func_code, idx, view, &state->pcu);
        default: 
            std::cerr << "[Map Error] Decode: Unknown IDX Mapping. IDX: " 
                      << (int)idx << std::endl;
            return RD_ERROR;
    }
}

/*============================ Privite =========================*/
uint8_t RdMap::GetSourceIdFromIdx(uint8_t idx) {
    // 1. ECU 대역 (0~31, 128~159)
    if ((idx <= 31) || (idx >= 128 && idx <= 159)) {
        return TARGET::ECU;
    }
    // 2. DPC 대역 (64~95, 192~223)
    else if ((idx >= 64 && idx <= 95) || (idx >= 192 && idx <= 223)) {
        return TARGET::DPC;
    }
    // 3. PCU 대역 (96~127, 224~255)
    else if ((idx >= 96 && idx <= 127) || (idx >= 224)) {
        return TARGET::PCU;
    }
    // Unknown IDX
    return 0xFF; 
}
//-------------------------Define ECU -----------------------------
RD_RET RdMap::EncodeEcu(uint8_t func_code, uint8_t idx, EcuState_t* ecu, PacketMap_u* view) { 
    ecu->comm.timeout_cnt++;
    // 타임아웃이 10회 이상 누적되면 RD_ERROR로 간주
    if (ecu->comm.timeout_cnt > TIMEOUT_MAX) ecu->comm.is_connected = false;

    if (func_code == FUNC::WQ) {    // Write Query 
        switch (idx) {
            case EcuStatePkt_t::ID:     Encoder::EcuStatePkg(view, ecu);  break;
            case EcuCmdVel_t::ID:       Encoder::EcuCmdVel(view, ecu);    break;
            case EcuCmdRpm_t::ID:       Encoder::EcuCmdRpm(view, ecu);    break;
            default:
                std::cerr << "[Map Error] Encode Ecu: Unknown IDX 0x" 
                  << std::hex << std::uppercase << (int)idx << std::dec << std::endl;
                return RD_ERROR;
        }
    } else if (func_code == FUNC::RQ) { // Read Query 
        std::memset(view->raw.data, 0, 8);
        view->raw.idx = idx; 
    } else {
        std::cerr << "[Map Error] Encode Ecu: Unknown Function Code 0x" 
                  << std::hex << std::uppercase << (int)func_code << std::dec << std::endl;
        return RD_ERROR;
    }
    return RD_OK;
}

RD_RET RdMap::DecodeEcu(uint8_t func_code, uint8_t idx, const PacketMap_u* view, EcuState_t* ecu) {
    ecu->comm.timeout_cnt = 0;        // 생존 확인! 카운터 리셋
    ecu->comm.is_connected = true;    // 연결 상태 회복

    if (func_code == FUNC::RR) { // Read Response (요청한 데이터 도착)
        switch (idx) {
            case EcuStatePkt_t::ID:     Decoder::EcuStatePkg(view, ecu);    break;
            case EcuCmdRpm_t::ID:       Decoder::EcuFbCmdRpm(view, ecu);    break;
            case EcuFbPose_t::ID:       Decoder::EcuFbPose(view, ecu);      break;
            case EcuFbRpm_t::ID:        Decoder::EcuFbRpm(view, ecu);       break;
            case EcuFbCurrent_t::ID:    Decoder::EcuFbCurrent(view, ecu);   break;
            case EcuFbMotor_t::ID:      Decoder::EcuFbMotor(view, ecu);     break;
            case EcuFbImuQuat_t::ID:    Decoder::EcuFbImuQuat(view, ecu);   break;
            case EcuFbImuGyro_t::ID:    Decoder::EcuFbImuGyro(view, ecu);   break;
            case EcuFbImuAccel_t::ID:   Decoder::EcuFbImuAccel(view, ecu);  break;
            case EcuFbLinkage_t::ID:    Decoder::EcuFbLinkage(view, ecu);   break;
            default:
                std::cerr << "[Map Error] Decode Ecu: Unknown IDX 0x" 
                  << std::hex << std::uppercase << (int)idx << std::dec << std::endl;
                return RD_ERROR;
        }
    } else if (func_code == FUNC::WR) { // Write Response (명령 수신 확인/Echo)
        // 송신 성공 여부 확인 용도로 사용
    } else {
        std::cerr << "[Map Error] Decode Ecu: Unknown Function Code 0x" 
                  << std::hex << std::uppercase << (int)func_code << std::dec << std::endl;
        return RD_ERROR;
    }
    return RD_OK;
}
//-------------------------Define DPC -----------------------------
RD_RET RdMap::EncodeDpc(uint8_t func_code, uint8_t idx, DpcState_t* dpc, PacketMap_u* view) {
    dpc->comm.timeout_cnt++;
    // 타임아웃이 10회 이상 누적되면 RD_ERROR로 간주
    if (dpc->comm.timeout_cnt > TIMEOUT_MAX) dpc->comm.is_connected = false;

    if (func_code == FUNC::WQ) { // Write Query (명령 쓰기)
        switch (idx) {
            case DpcStatePkt_t::ID: Encoder::DpcStatePkg(view, dpc); break;
            default:
                std::cerr << "[Map Error] Decode Dpc: Unknown IDX 0x" 
                      << std::hex << std::uppercase << (int)idx << std::dec << std::endl;
                return RD_ERROR;
        }
    } else if (func_code == FUNC::RQ) { // Read Query (데이터 요청)
        std::memset(view->raw.data, 0, 8);
        view->raw.idx = idx; 
    } else {
        std::cerr << "[Map Error] Encode Dpc: Unknown Function Code 0x" 
                  << std::hex << std::uppercase << (int)func_code << std::dec << std::endl;
        return RD_ERROR;
    }
    return RD_OK;
}

RD_RET RdMap::DecodeDpc(uint8_t func_code, uint8_t idx, const PacketMap_u* view, DpcState_t* dpc) {
    dpc->comm.timeout_cnt = 0;        // 생존 확인! 카운터 리셋
    dpc->comm.is_connected = true;    // 연결 상태 회복

    if (func_code == FUNC::RR) { // Read Response 
        switch (idx) {
            case DpcStatePkt_t::ID: Decoder::DpcStatePkg(view, dpc); break;
            default:
                std::cerr << "[Map Error] Decode Dpc: Unknown IDX 0x" 
                  << std::hex << std::uppercase << (int)idx << std::dec << std::endl;
                return RD_ERROR;
        }
    } else if (func_code == FUNC::WR) { // Write Response 
    } else {
        std::cerr << "[Map Error] Decode Dpc: Unknown Function Code 0x" 
                  << std::hex << std::uppercase << (int)func_code << std::dec << std::endl;
        return RD_ERROR;
    }
    return RD_OK;
}
//-------------------------Define PCU -----------------------------
RD_RET RdMap::EncodePcu(uint8_t func_code, uint8_t idx, PcuState_t* pcu, PacketMap_u* view) {
    pcu->comm.timeout_cnt++;
    // 타임아웃이 10회 이상 누적되면 RD_ERROR로 간주
    if (pcu->comm.timeout_cnt > TIMEOUT_MAX) pcu->comm.is_connected = false;

    if (func_code == FUNC::WQ) { // Write Query (명령 쓰기)
        switch (idx) {
            case PcuStatePkt_t::ID: Encoder::PcuStatePkg(view, pcu); break;
            default:
                std::cerr << "[Map Error] Encode Pcu: Unknown IDX 0x" 
                  << std::hex << std::uppercase << (int)idx << std::dec << std::endl;
                return RD_ERROR;
        }
    } else if (func_code == FUNC::RQ) { // Read Query (데이터 요청)
        std::memset(view->raw.data, 0, 8);
        view->raw.idx = idx; 
    } else {
        std::cerr << "[Map Error] Encode Pcu: Unknown Function Code 0x" 
                  << std::hex << std::uppercase << (int)func_code << std::dec << std::endl;
        return RD_ERROR;
    }
    return RD_OK;
}

RD_RET RdMap::DecodePcu(uint8_t func_code, uint8_t idx, const PacketMap_u* view, PcuState_t* pcu) {
    pcu->comm.timeout_cnt = 0;        // 생존 확인! 카운터 리셋
    pcu->comm.is_connected = true;    // 연결 상태 회복

    if (func_code == FUNC::RR) { // Read Response (요청한 데이터 도착)
        switch (idx) {
            case PcuStatePkt_t::ID: Decoder::PcuStatePkg(view, pcu); break;
            default:
                std::cerr << "[Map Error] Decode Pcu: Unknown IDX 0x" 
                  << std::hex << std::uppercase << (int)idx << std::dec << std::endl;
                return RD_ERROR;
        }
    } else if (func_code == FUNC::WR) { // Write Response (명령 수신 확인/Echo)
        // 송신 성공 여부 확인 용도로 사용
    } else {
        std::cerr << "[Map Error] Encode Pcu: Unknown Function Code 0x" 
                  << std::hex << std::uppercase << (int)func_code << std::dec << std::endl;
        return RD_ERROR;
    }
    return RD_OK;
}

/*---------------------- ECU Hanlder Define --------------------*/
/*----------Encoder----------*/
void RdMap::Encoder::EcuStatePkg(PacketMap_u* , const EcuState_t* ) {}
void RdMap::Encoder::EcuCmdVel(PacketMap_u* view, const EcuState_t* ecu) {
    view->vel.idx = EcuCmdVel_t::ID;
    view->vel.lin_x = ecu->cmd_linear_x;
    view->vel.ang_z = ecu->cmd_angular_z;
}
void RdMap::Encoder::EcuCmdRpm(PacketMap_u* view, const EcuState_t* ecu) {
    view->rpm.idx = EcuCmdRpm_t::ID;
    for(int i=0; i<4; i++) view->rpm.m[i] = ecu->cmd_motor_rpm[i];
}
/*----------Decoder----------*/
void RdMap::Decoder::EcuStatePkg(const PacketMap_u* view, EcuState_t* ecu) {
    const EcuStatePkt_t& s = view->ecu;
    // DATA[0]: FSM state + HW errors
    ecu->fsm_state        = static_cast<ECU_FSM_e>(s.fsm_state);
    ecu->hw_err.ecu_can   = s.hw_can;
    ecu->hw_err.ecu_i2c   = s.hw_i2c;
    ecu->hw_err.ecu_uart1 = s.hw_uart1;
    // DATA[1]: motor health — 2-bit per motor [M4|M3|M2|M1] (M1 at LSB)
    for (int i = 0; i < 4; i++) {
        ecu->motor_state[i] = static_cast<Motor_health>((s.motor_health >> (i * 2)) & 0x03);
    }
    // DATA[2-3]: motor tx error count — 4-bit per motor (M1 at LSB)
    for (int i = 0; i < 4; i++) {
        ecu->motor_tx_err_cnt[i] = (s.motor_tx_err >> (i * 4)) & 0x0F;
    }
    // DATA[4-5]: motor rx error count — 4-bit per motor (M1 at LSB)
    for (int i = 0; i < 4; i++) {
        ecu->motor_rx_err_cnt[i] = (s.motor_rx_err >> (i * 4)) & 0x0F;
    }
    // DATA[6-7]: alive_time
    ecu->comm.alive_time = s.alive_time;
}
void RdMap::Decoder::EcuFbCmdRpm(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<4; i++) ecu->motor_cmd_rpm[i] = static_cast<float>(view->rpm.m[i]) * 2.0f;
}
void RdMap::Decoder::EcuFbPose(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<4; i++) ecu->motor_pose[i] = static_cast<float>(view->fb_pose.m[i]) * 0.1f;;
}
void RdMap::Decoder::EcuFbRpm(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<4; i++) ecu->motor_speed[i] = static_cast<float>(view->fb_rpm.m[i]) * 10.0f;
}
void RdMap::Decoder::EcuFbCurrent(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<4; i++) ecu->motor_current[i] = static_cast<float>(view->fb_current.m[i]) * 0.01f;
}

void RdMap::Decoder::EcuFbMotor(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<4; i++) {
        ecu->motor_temp[i]  = view->motor.temp[i];
        ecu->motor_error[i] = view->motor.err[i];
    }
}
void RdMap::Decoder::EcuFbImuQuat(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<4; i++) ecu->imu_quaternion[i] = view->imu_quat.quat[i];
}
void RdMap::Decoder::EcuFbImuGyro(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<3; i++) ecu->imu_gyro[i] = view->imu_gyro.unit[i];
}
void RdMap::Decoder::EcuFbImuAccel(const PacketMap_u* view, EcuState_t* ecu) {
    for(int i=0; i<3; i++) ecu->imu_accel[i] = view->imu_accel.unit[i];
}
void RdMap::Decoder::EcuFbLinkage(const PacketMap_u* view, EcuState_t* ecu) {
    ecu->linkage_encoder[0] = static_cast<uint16_t>(view->linkage.ang0);
    ecu->linkage_encoder[1] = static_cast<uint16_t>(view->linkage.ang1);
    ecu->linkage_encoder[2] = static_cast<uint16_t>(view->linkage.ang2);
    ecu->linkage_encoder[3] = static_cast<uint16_t>(view->linkage.ang3);
    ecu->linkage_encoder[4] = static_cast<uint16_t>(view->linkage.ang4);
}
/*---------------------- DPC Hanlder Define --------------------*/
/*----------Encoder----------*/
void RdMap::Encoder::DpcStatePkg(PacketMap_u* , const DpcState_t* ) {}
/*----------Decoder----------*/
void RdMap::Decoder::DpcStatePkg(const PacketMap_u*, DpcState_t*) {}

/*---------------------- PCU Hanlder Define --------------------*/
/*----------Encoder----------*/
void RdMap::Encoder::PcuStatePkg(PacketMap_u* , const PcuState_t* ) {}
/*----------Decoder----------*/
void RdMap::Decoder::PcuStatePkg(const PacketMap_u*, PcuState_t*) {}

} // namespace orin_bridge