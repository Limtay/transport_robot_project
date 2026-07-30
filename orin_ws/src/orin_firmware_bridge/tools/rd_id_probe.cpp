// rd_id_probe — **RS485 버스에 누가 어느 ID 로 있는가** 를 PING 으로 확인한다.
//
// 왜 필요한가: 2026-07-30 에 DPC ID 가 세 번 바뀌었다 (코드 0xD1/0xD2 → 시트 0xD2/0xD1
// → 펌웨어 0xE2). 어느 것이 맞는지 **버스에 물어보는 것**이 문서를 또 읽는 것보다 빠르고
// 확실하다. 브리지는 `TARGET::{ECU,DPC,PCU}` 세 개만 부를 수 있어 이 확인을 못 한다.
//
// L0/L1 만 쓴다 — ROS·FSM 없이 와이어만 본다 (rd_probe.cpp 와 같은 방식).
//
//   빌드: g++ -std=c++17 -I include tools/rd_id_probe.cpp -Lbuild -lrd_core_lib \
//              $(pkg-config --cflags --libs libserial) -o /tmp/rd_id_probe
//   실행: /tmp/rd_id_probe [포트]

#include <chrono>
#include <cstdio>
#include <thread>

#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_uart.hpp"

using namespace orin_bridge;

int main(int argc, char** argv) {
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";

    RdUart uart;
    if (uart.Initialize(port) != RD_OK) {
        printf("포트 열기 실패: %s\n", port);
        return 1;
    }
    RdComm comm(&uart);
    if (comm.Initialize() != RD_OK) { printf("COMM 초기화 실패\n"); return 1; }

    // 후보: 지금까지 문서·코드에 등장한 모든 값 + ECU(대조군).
    struct Cand { uint8_t id; const char* name; };
    const Cand cands[] = {
        {0xE1, "ECU        (대조군)"},
        {0xE2, "DPC?  펌웨어 헤더"},
        {0xD1, "DPC?  구 코드 A / 시트 B"},
        {0xD2, "DPC?  구 코드 B / 시트 A"},
        {0xA1, "PCU / PRA"},
    };

    printf("포트 %s — PING 스캔 (각 5회)\n", port);
    printf("%-28s %s\n", "ID", "응답");
    printf("--------------------------------------------------\n");

    PACKET_comm_t pkt{};
    for (const Cand& c : cands) {
        int ok = 0;
        double rtt_sum = 0.0;
        for (int i = 0; i < 5; i++) {
            // PING(0x01) — Data 없음. 어떤 레지스터 맵이든 응답해야 한다.
            pkt.tx.pack.ID   = c.id;
            pkt.tx.pack.Inst = static_cast<uint8_t>(PacketInst::PING);
            comm.Clear();
            auto t0 = std::chrono::steady_clock::now();
            if (comm.Write(&pkt, 0) != RD_OK) continue;
            const RD_RET r = comm.Read(&pkt, 2, 2);
            auto t1 = std::chrono::steady_clock::now();
            if (r == RD_OK && pkt.rx.pack.ID == c.id) {
                ok++;
                rtt_sum += std::chrono::duration<double, std::milli>(t1 - t0).count();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (ok > 0) printf("0x%02X  %-22s ★ 응답 %d/5  (avg %.2f ms)\n",
                           c.id, c.name, ok, rtt_sum / ok);
        else        printf("0x%02X  %-22s   무응답\n", c.id, c.name);
    }
    printf("--------------------------------------------------\n");
    printf("★ 표시된 ID 가 실제로 버스에 있는 보드다.\n");
    return 0;
}
