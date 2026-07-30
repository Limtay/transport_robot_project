#include "orin_firmware_bridge/core/rd_map.hpp"
#include <iostream>
#include <cstring>

namespace orin_bridge {

RD_RET RdMap::Encode(const TaskConfig_t& config, RobotState_t* state,
                     PACKET_comm_t* pkt, size_t* out_data_len) {
    // tx 헤더 ID / Inst 설정
    pkt->tx.pack.ID   = config.target_id;
    pkt->tx.pack.Inst = static_cast<uint8_t>(config.inst);

    std::lock_guard<std::mutex> lock(state->state_mutex);

    switch (config.target_id) {
        case TARGET::ECU: return EncodeNode(config, &state->ecu, ecu::REG_TOTAL_SIZE, pkt, out_data_len);
        case TARGET::DPC: return EncodeNode(config, &state->dpc, dpc::REG_TOTAL_SIZE, pkt, out_data_len);
        case TARGET::PCU: return EncodeNode(config, &state->pcu, pcu::REG_TOTAL_SIZE, pkt, out_data_len);
        default:
            std::cerr << "[Map Error] Encode: Unknown Target ID 0x"
                      << std::hex << (int)config.target_id << std::dec << std::endl;
            return RD_ERROR;
    }
}

RD_RET RdMap::Decode(PACKET_comm_t* pkt, const TaskConfig_t& sent_config, RobotState_t* state) {
    // STM 은 별도 RESPONSE inst 없이 송신했던 inst(READ/WRITE)를 그대로 echo 한다.
    uint8_t rx_inst       = pkt->rx.pack.Inst;
    uint8_t expected_inst = static_cast<uint8_t>(sent_config.inst);
    if (rx_inst != expected_inst) {
        std::cerr << "[Map Error] Decode: inst mismatch — sent 0x"
                  << std::hex << (int)expected_inst << " got 0x" << (int)rx_inst
                  << std::dec << std::endl;
        return RD_ERROR;
    }
    uint8_t rx_id = pkt->rx.pack.ID;
    if (rx_id != TARGET::ORIN) {
        std::cerr << "[Map Error] Decode: Expected ORIN ID, got 0x"
                  << std::hex << (int)rx_id << std::dec << std::endl;
        return RD_ERROR;
    }

    std::lock_guard<std::mutex> lock(state->state_mutex);

    switch (sent_config.target_id) {
        case TARGET::ECU: return DecodeNode(pkt, sent_config, &state->ecu, ecu::REG_TOTAL_SIZE);
        case TARGET::DPC: return DecodeNode(pkt, sent_config, &state->dpc, dpc::REG_TOTAL_SIZE);
        case TARGET::PCU: return DecodeNode(pkt, sent_config, &state->pcu, pcu::REG_TOTAL_SIZE);
        default:
            std::cerr << "[Map Error] Decode: Unknown Target ID 0x"
                      << std::hex << (int)sent_config.target_id << std::dec << std::endl;
            return RD_ERROR;
    }
}

// ========================= 노드 공통 템플릿 =========================
// ECU/DPC/PCU 의 Encode/Decode/RegionPtr 동작은 동일하므로 하나로 통합한다.
// 노드마다 다른 것은 상태 타입(State)과 레지스터 총 크기(total_size)뿐이다.

template <typename State>
uint8_t* RdMap::GetRegionPtr(State* node, uint16_t start_addr,
                             uint16_t total_size, size_t* region_size) {
    if (start_addr >= total_size) {
        *region_size = 0;
        return nullptr;
    }
    *region_size = total_size - start_addr;
    return reinterpret_cast<uint8_t*>(&node->reg) + start_addr;
}

template <typename State>
RD_RET RdMap::EncodeNode(const TaskConfig_t& config, State* node, uint16_t total_size,
                         PACKET_comm_t* pkt, size_t* out_data_len) {
    node->comm.timeout_cnt++;
    if (node->comm.timeout_cnt > TIMEOUT_MAX) node->comm.is_connected = false;

    // REBOOT / PING: Parameter 없음 — Data 0바이트
    if (config.inst == PacketInst::REBOOT || config.inst == PacketInst::PING) {
        *out_data_len = 0;
        return RD_OK;
    }

    if (config.inst == PacketInst::READ) {
        // READ: Data = [addr_L, addr_H, len_L, len_H] × n (멀티세그먼트, n=1 은 기존과 동일)
        if (config.seg_count >= 1) {
            for (int s = 0; s < config.seg_count; ++s) {
                uint8_t* d = &pkt->tx.pack.Data[s * 4];
                d[0] = static_cast<uint8_t>(config.segs[s].addr & 0xFF);
                d[1] = static_cast<uint8_t>((config.segs[s].addr >> 8) & 0xFF);
                d[2] = static_cast<uint8_t>(config.segs[s].len & 0xFF);
                d[3] = static_cast<uint8_t>((config.segs[s].len >> 8) & 0xFF);
            }
            *out_data_len = static_cast<size_t>(config.seg_count) * 4;
            return RD_OK;
        }
        // 단일 구간 (seg_count==0): start_addr/data_len
        pkt->tx.pack.Data[0] = static_cast<uint8_t>(config.start_addr & 0xFF);
        pkt->tx.pack.Data[1] = static_cast<uint8_t>((config.start_addr >> 8) & 0xFF);
        pkt->tx.pack.Data[2] = static_cast<uint8_t>(config.data_len & 0xFF);
        pkt->tx.pack.Data[3] = static_cast<uint8_t>((config.data_len >> 8) & 0xFF);
        *out_data_len = 4;
        return RD_OK;
    }

    if (config.inst == PacketInst::WRITE) {
        // WRITE: Data = [start_addr_L, start_addr_H, ...register_bytes...]
        pkt->tx.pack.Data[0] = static_cast<uint8_t>(config.start_addr & 0xFF);
        pkt->tx.pack.Data[1] = static_cast<uint8_t>((config.start_addr >> 8) & 0xFF);

        size_t region_size = 0;
        uint8_t* src = GetRegionPtr(node, config.start_addr, total_size, &region_size);
        if (!src) {
            std::cerr << "[Map Error] Encode WRITE: Unknown region target=0x"
                      << std::hex << (int)config.target_id << " addr=0x"
                      << config.start_addr << std::dec << std::endl;
            return RD_ERROR;
        }
        size_t copy_len = (config.data_len <= region_size) ? config.data_len : region_size;
        if (config.data_len > region_size) {
            std::cerr << "[Map Warn] Encode WRITE: data_len(" << config.data_len
                      << ") truncated to region_size(" << region_size << ") target=0x"
                      << std::hex << (int)config.target_id << std::dec << std::endl;
        }
        std::memcpy(&pkt->tx.pack.Data[2], src, copy_len);
        *out_data_len = 2 + copy_len;
        return RD_OK;
    }

    if (config.inst == PacketInst::RW) {
        // RW: Data = [R] + read세그[addrL,H,lenL,H]×R + [waddrL,H] + shadow write 데이터
        //     (STM 처리: WRITE 적용 → READ 스냅샷, 에러 니블 분리)
        if (config.seg_count < 1) {
            std::cerr << "[Map Error] Encode RW: read segment 없음 target=0x"
                      << std::hex << (int)config.target_id << std::dec << std::endl;
            return RD_ERROR;
        }
        uint8_t* d = pkt->tx.pack.Data;
        d[0] = config.seg_count;
        for (int s = 0; s < config.seg_count; ++s) {
            uint8_t* p = &d[1 + s * 4];
            p[0] = static_cast<uint8_t>(config.segs[s].addr & 0xFF);
            p[1] = static_cast<uint8_t>((config.segs[s].addr >> 8) & 0xFF);
            p[2] = static_cast<uint8_t>(config.segs[s].len & 0xFF);
            p[3] = static_cast<uint8_t>((config.segs[s].len >> 8) & 0xFF);
        }
        const size_t woff = 1 + static_cast<size_t>(config.seg_count) * 4;
        d[woff]     = static_cast<uint8_t>(config.start_addr & 0xFF);
        d[woff + 1] = static_cast<uint8_t>((config.start_addr >> 8) & 0xFF);

        size_t region_size = 0;
        uint8_t* src = GetRegionPtr(node, config.start_addr, total_size, &region_size);
        if (!src || config.data_len == 0 || config.data_len > region_size) {
            std::cerr << "[Map Error] Encode RW: write 구간 오류 target=0x"
                      << std::hex << (int)config.target_id << " addr=0x"
                      << config.start_addr << std::dec
                      << " len=" << config.data_len << std::endl;
            return RD_ERROR;
        }
        std::memcpy(&d[woff + 2], src, config.data_len);
        *out_data_len = woff + 2 + config.data_len;
        return RD_OK;
    }

    std::cerr << "[Map Error] Encode: Unsupported inst 0x"
              << std::hex << static_cast<int>(config.inst) << std::dec << std::endl;
    return RD_ERROR;
}

template <typename State>
RD_RET RdMap::DecodeNode(PACKET_comm_t* pkt, const TaskConfig_t& sent_config,
                         State* node, uint16_t total_size) {
    node->comm.timeout_cnt = 0;
    node->comm.is_connected = true;

    // ── RW: 에러 바이트가 니블 분리 (read_err | write_err<<4) — 일반 경로보다 먼저 처리
    if (sent_config.inst == PacketInst::RW) {
        last_rw_err_ = pkt->rx.pack.Data[0];   // 니블 그대로 보관 (§3.4 rw_err)
        const uint8_t rd_err = pkt->rx.pack.Data[0] & 0x0F;
        const uint8_t wr_err = (pkt->rx.pack.Data[0] >> 4) & 0x0F;
        if (wr_err != 0) {
            rw_write_err_total_++;
            // write 거부(모드 lock 등)는 read 스트림을 끊지 않음 — 200Hz 스팸 방지 간헐 로그
            if ((rw_write_err_cnt_++ % 200) == 0) {
                std::cerr << "[Map Warn] RW write rejected: ["
                          << PacketErrorStr(static_cast<PacketError>(wr_err))
                          << "] (cnt=" << rw_write_err_cnt_ << ")" << std::endl;
            }
        } else if (rw_write_err_cnt_ != 0) {
            std::cerr << "[Map Info] RW write recovered (rejected " << rw_write_err_cnt_
                      << " times)" << std::endl;
            rw_write_err_cnt_ = 0;
        }
        if (rd_err != 0) {
            std::cerr << "[Map Error] Decode RW: read error ["
                      << PacketErrorStr(static_cast<PacketError>(rd_err)) << "]" << std::endl;
            return RD_ERROR;
        }
        return ScatterReadSegs(pkt, sent_config, node, total_size);
    }

    PacketError err = static_cast<PacketError>(pkt->rx.pack.Data[0]);
    if (err != PacketError::NONE) {
        std::cerr << "[Map Error] Decode: target=0x"
                  << std::hex << (int)sent_config.target_id << " returned error [0x"
                  << static_cast<int>(pkt->rx.pack.Data[0]) << std::dec
                  << "] " << PacketErrorStr(err) << std::endl;
        return RD_ERROR;
    }

    // WRITE / REBOOT / PING 응답: Data[0]=err 만 확인하면 끝
    if (sent_config.inst == PacketInst::WRITE ||
        sent_config.inst == PacketInst::REBOOT ||
        sent_config.inst == PacketInst::PING) return RD_OK;

    if (sent_config.inst == PacketInst::READ) {
        return ScatterReadSegs(pkt, sent_config, node, total_size);
    }

    std::cerr << "[Map Error] Decode: Unsupported sent inst 0x"
              << std::hex << static_cast<int>(sent_config.inst) << std::dec << std::endl;
    return RD_ERROR;
}

// 응답 연접 read 데이터(Data[1..])를 각 세그먼트의 제 주소로 scatter — READ/RW 공용.
// 단일 구간(seg_count==0, READ 레거시)은 1세그로 취급해 경로를 통일한다.
template <typename State>
RD_RET RdMap::ScatterReadSegs(PACKET_comm_t* pkt, const TaskConfig_t& sent_config,
                              State* node, uint16_t total_size) {
    const Segment_t single{sent_config.start_addr, sent_config.data_len};
    const Segment_t* segs = (sent_config.seg_count >= 1) ? sent_config.segs : &single;
    const int nseg        = (sent_config.seg_count >= 1) ? sent_config.seg_count : 1;

    size_t expected = 0;
    for (int s = 0; s < nseg; ++s) expected += segs[s].len;
    // rx.data_len = Data[] 유효 바이트 수 = Error(1) + register bytes(N)
    size_t received = (pkt->rx.data_len > 1) ? pkt->rx.data_len - 1 : 0;

    if (received != expected) {
        std::cerr << "[Map Error] Decode READ: length mismatch target=0x"
                  << std::hex << (int)sent_config.target_id << std::dec
                  << " expected=" << expected << " received=" << received << std::endl;
        return RD_ERROR;
    }

    size_t off = 1;   // Data[0] = Error 바이트
    for (int s = 0; s < nseg; ++s) {
        size_t region_size = 0;
        uint8_t* dst = GetRegionPtr(node, segs[s].addr, total_size, &region_size);
        if (!dst) {
            std::cerr << "[Map Error] Decode READ: Unknown region target=0x"
                      << std::hex << (int)sent_config.target_id << " addr=0x"
                      << segs[s].addr << std::dec << std::endl;
            return RD_ERROR;
        }
        if (segs[s].len > region_size) {
            std::cerr << "[Map Error] Decode READ: seg len exceeds region size target=0x"
                      << std::hex << (int)sent_config.target_id << " addr=0x"
                      << segs[s].addr << std::dec << std::endl;
            return RD_ERROR;
        }
        std::memcpy(dst, &pkt->rx.pack.Data[off], segs[s].len);
        off += segs[s].len;
    }
    return RD_OK;
}

} // namespace orin_bridge
