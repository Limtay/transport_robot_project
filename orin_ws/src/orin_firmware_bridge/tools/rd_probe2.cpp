// rd_probe2 — 레이아웃 판별을 인과 실험으로 확정한다.
//
// probe1 의 "10회 중 1회 비영" 은 근거가 약했다. rs485_proc_delta 는
//   rd_system.c:643  reg.sys.rs485_proc_delta = rd_delta_tick(now, realtime_tick)
// 로 **직전 트랜잭션의 STM 처리시간**이고, 다음 응답에 실린다(소급 매칭).
//
// 따라서 무거운 요청(DIRECT RW 87B) 직후에 읽은 값은 가벼운 요청(READ 3B) 직후보다
// 체계적으로 커야 한다. 신 펌웨어면 부하에 반응하고, 구 펌웨어면 addr32 는
// reserved0[0] 이라 **영원히 0** 이다. 반응 자체가 판별이 된다.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_uart.hpp"

using namespace orin_bridge;
namespace ecu = orin_bridge::ecu;

static RdComm*       g_comm = nullptr;
static RdMap         g_map;
static PACKET_comm_t g_pkt{};
static RobotState_t  g_state{};

static bool Txn(const TaskConfig_t& cfg, uint8_t* payload_out = nullptr, size_t n = 0) {
    size_t dl = 0;
    if (g_map.Encode(cfg, &g_state, &g_pkt, &dl) != RD_OK) return false;
    g_comm->Clear();
    if (g_comm->Write(&g_pkt, dl) != RD_OK) return false;
    if (g_comm->Read(&g_pkt, 2, 2) != RD_OK) return false;
    if (payload_out && n) memcpy(payload_out, &g_pkt.rx.pack.Data[1], n);
    g_map.Decode(&g_pkt, cfg, &g_state);   // RW write 거부(FAULT)는 무시 — read 는 유효
    return true;
}

static void Stats(const char* label, std::vector<int>& v) {
    if (v.empty()) { printf("  %-28s (샘플 없음)\n", label); return; }
    std::sort(v.begin(), v.end());
    int nz = 0; double sum = 0;
    for (int x : v) { if (x) nz++; sum += x; }
    printf("  %-28s n=%zu  비영=%d(%.0f%%)  평균=%.2f  중앙=%d  최대=%d\n",
           label, v.size(), nz, 100.0 * nz / v.size(), sum / v.size(),
           v[v.size() / 2], v.back());
}

int main(int argc, char** argv) {
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   N    = (argc > 2) ? atoi(argv[2]) : 150;

    RdUart uart(port);
    RdComm comm(&uart);
    g_comm = &comm;
    if (uart.Init() != RD_OK || comm.Init(&g_pkt) != RD_OK) { printf("포트 실패\n"); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    memset(&g_state.ecu.reg, 0, sizeof(g_state.ecu.reg));

    printf("=== rd_probe2 — proc_delta 인과 실험 ===\n\n");

    TaskConfig_t light{TARGET::ECU, PacketInst::READ, 27, 5};          // 3B payload
    TaskConfig_t probe32{TARGET::ECU, PacketInst::READ, 32, 1};
    TaskConfig_t probe228{TARGET::ECU, PacketInst::READ, 228, 1};
    TaskConfig_t heavy{TARGET::ECU, PacketInst::RW,                    // 79B payload / 87B 와이어
        ecu::REG_CMD_MOTOR_OFFSET, ecu::REG_CMD_MOTOR_SIZE, {
        {27, 5}, {ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE},
        {ecu::REG_MOTOR_DATA_OFFSET, 36}, {164, 16},
        {ecu::REG_PROC_DELTA_OFFSET, 1}, {ecu::REG_CMD_MOTOR_OFFSET, 4}}};

    std::vector<int> a32_light, a32_heavy, a228_light, a228_heavy;
    uint8_t b = 0;

    for (int i = 0; i < N; i++) {
        // 가벼운 요청 → 직후 addr32 / addr228 (직전 = light 의 처리시간)
        if (Txn(light)) {
            if (Txn(probe32,  &b, 1)) a32_light.push_back(b);
            if (Txn(probe228, &b, 1)) a228_light.push_back(b);
        }
        // 무거운 요청 → 직후 addr32 / addr228 (직전 = heavy 의 처리시간)
        if (Txn(heavy)) {
            if (Txn(probe32,  &b, 1)) a32_heavy.push_back(b);
            if (Txn(probe228, &b, 1)) a228_heavy.push_back(b);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    printf("[부하별 분포]  (단위: rd_delta_tick — TIM5 10kHz 기준 0.1ms)\n");
    Stats("addr32  after READ 3B",   a32_light);
    Stats("addr32  after RW 87B",    a32_heavy);
    Stats("addr228 after READ 3B",   a228_light);
    Stats("addr228 after RW 87B",    a228_heavy);

    auto nzpct = [](std::vector<int>& v) {
        if (v.empty()) return -1.0;
        int nz = 0; for (int x : v) if (x) nz++;
        return 100.0 * nz / v.size();
    };
    double l32 = nzpct(a32_light), h32 = nzpct(a32_heavy);
    double l228 = nzpct(a228_light), h228 = nzpct(a228_heavy);

    printf("\n[판정]\n");
    printf("  addr32  부하 반응: 가벼움 %.0f%% → 무거움 %.0f%%\n", l32, h32);
    printf("  addr228 부하 반응: 가벼움 %.0f%% → 무거움 %.0f%%\n", l228, h228);

    bool a32_live  = (h32  > l32  + 20.0);
    bool a228_live = (h228 > l228 + 20.0);
    if (a32_live && !a228_live)
        printf("  → **신 펌웨어 확정** — proc_delta 가 addr 32 에서 부하에 반응, 228 은 죽어 있다\n");
    else if (a228_live && !a32_live)
        printf("  → **구 펌웨어 확정** — proc_delta 가 addr 228 에 있다. 현 브리지와 불일치!\n");
    else
        printf("  → 불명확 — 위 수치로 직접 판단\n");

    // hw_* 비트 관찰 — FAULT 원인 교차 확인
    printf("\n[hw 플래그 관찰] bit0=uart1 1=uart2 2=uart6(IMU) 3=can 4=i2c\n");
    uint8_t sys[17] = {};
    for (int i = 0; i < 3; i++) {
        if (Txn(TaskConfig_t{TARGET::ECU, PacketInst::READ, 16, 17}, sys, 17)) {
            printf("  [24]=0x%02X [25]=0x%02X [26]=0x%02X  sys_state=%u\n",
                   sys[8], sys[9], sys[10], sys[11]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("  신 레이아웃 해석: hw_error=0x%02X hw_fatal=0x%02X hw_reset=0x%02X\n",
           sys[8], sys[9], sys[10]);
    printf("  구 레이아웃 해석: hw_reset=0x%02X hw_fatal=0x%02X hw_error=0x%02X\n",
           sys[8], sys[9], sys[10]);
    printf("  rd_system.c:288 은 CAN fatal 시 hw.reset.bit.can(0x08)=1 과 FAULT 를 **함께** 세운다.\n");
    printf("  → sys_state=5(FAULT) 인데 0x08 이 놓인 자리가 곧 hw_reset 이다.\n");

    comm.Stop();
    return 0;
}
