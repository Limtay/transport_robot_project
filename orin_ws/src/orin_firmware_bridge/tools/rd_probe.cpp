// rd_probe — 실기 관문(06 §4.2) 계측기. 모터 없이 돌릴 수 있는 것만 본다.
//
//   A. 어떤 펌웨어가 올라가 있는가 (신 레이아웃 vs 구 레이아웃) — addr 32 / 228 로 판별
//   B. ECU 가 살아 있는가 (realtime_tick 증가)
//   C. P8 게이트 — DIRECT RW 87B 요청 반복. 구 RX 64B 에서는 오버런으로 RD_FATAL 이었다
//   D. auto_mode 4(VELOCITY)/5(POSITION) 수용 — STM #3 자가치유 분기
//
// 브리지 노드가 아니라 L0/L1 만 쓴다 — FSM·ROS 없이 와이어만 본다.
// ⚠ 모든 write 는 shadow 를 0 으로 두고 나간다 (cmd_motor 전 구간 0 = 무토크).

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_uart.hpp"

using namespace orin_bridge;
namespace ecu = orin_bridge::ecu;

static RdUart*        g_uart = nullptr;
static RdComm*        g_comm = nullptr;
static RdMap          g_map;
static PACKET_comm_t  g_pkt{};
static RobotState_t   g_state{};

static const char* RetStr(RD_RET r) {
    switch (r) {
        case RD_OK:      return "RD_OK";
        case RD_ERROR:   return "RD_ERROR";
        case RD_TIMEOUT: return "RD_TIMEOUT";
        case RD_FATAL:   return "RD_FATAL";
        default:         return "RD_?";
    }
}

// ExecuteTask(rd_schedule.cpp:505) 와 같은 순서: Encode → Clear → Write → Read → Decode
struct TxnResult {
    RD_RET  encode = RD_OK;
    RD_RET  write  = RD_OK;
    RD_RET  read   = RD_OK;
    RD_RET  decode = RD_OK;
    bool    ok     = false;
    double  rtt_ms = 0.0;
    uint8_t err0   = 0;   // 응답 Data[0]
};

static TxnResult Txn(const TaskConfig_t& cfg) {
    TxnResult r;
    size_t data_len = 0;
    r.encode = g_map.Encode(cfg, &g_state, &g_pkt, &data_len);
    if (r.encode != RD_OK) return r;

    g_comm->Clear();
    auto t0 = std::chrono::steady_clock::now();
    r.write = g_comm->Write(&g_pkt, data_len);
    if (r.write != RD_OK) return r;

    r.read = g_comm->Read(&g_pkt, 2, 2);
    auto t1 = std::chrono::steady_clock::now();
    r.rtt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (r.read != RD_OK) return r;

    r.err0   = g_pkt.rx.pack.Data[0];
    r.decode = g_map.Decode(&g_pkt, cfg, &g_state);
    r.ok     = (r.decode == RD_OK);
    return r;
}

// 레지스터 한 구간을 그대로 읽어 raw 로 돌려준다 (state 를 안 거치고 응답 바이트를 본다).
static bool ReadRaw(uint16_t addr, uint16_t len, uint8_t* out) {
    TaskConfig_t cfg{TARGET::ECU, PacketInst::READ, addr, len};
    TxnResult r = Txn(cfg);
    if (!r.ok) {
        printf("      READ %u:%u 실패 — write=%s read=%s decode=%s err0=0x%02X\n",
               addr, len, RetStr(r.write), RetStr(r.read), RetStr(r.decode), r.err0);
        return false;
    }
    memcpy(out, &g_pkt.rx.pack.Data[1], len);   // Data[0] = err
    return true;
}

static void Hex(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) printf("%02X ", p[i]);
}

static bool WriteByte(uint16_t addr, uint8_t val) {
    auto* raw = reinterpret_cast<uint8_t*>(&g_state.ecu.reg);
    raw[addr] = val;
    TaskConfig_t cfg{TARGET::ECU, PacketInst::WRITE, addr, 1};
    TxnResult r = Txn(cfg);
    if (!r.ok) {
        printf("      WRITE %u=%u 실패 — write=%s read=%s decode=%s err0=0x%02X\n",
               addr, val, RetStr(r.write), RetStr(r.read), RetStr(r.decode), r.err0);
    }
    return r.ok;
}

int main(int argc, char** argv) {
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   n_p8 = (argc > 2) ? atoi(argv[2]) : 200;

    printf("=== rd_probe — port %s ===\n\n", port);

    RdUart uart(port);
    RdComm comm(&uart);
    g_uart = &uart; g_comm = &comm;

    if (uart.Init() != RD_OK) { printf("포트 열기 실패\n"); return 1; }
    if (comm.Init(&g_pkt) != RD_OK) { printf("comm Init 실패\n"); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---------- A. 링크 + 펌웨어 레이아웃 판별 ----------
    printf("[A] 링크 확인 & 펌웨어 레이아웃 판별\n");

    uint8_t sys[17] = {};
    int link_ok = 0;
    for (int i = 0; i < 5; i++) {
        if (ReadRaw(ecu::REG_SYS_OFFSET, 17, sys)) { link_ok++; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!link_ok) { printf("  ✗ ECU 응답 없음 — 배선/전원/보레이트 확인\n"); return 2; }

    printf("  SYS 16..32 raw: "); Hex(sys, 17); printf("\n");
    printf("    [24]=%02X [25]=%02X [26]=%02X  sys_state[27]=%u\n",
           sys[8], sys[9], sys[10], sys[11]);
    uint32_t tick0; memcpy(&tick0, &sys[12], 4);
    printf("    realtime_tick[28..31] = %u\n", tick0);
    printf("    addr32 = %u  (신: rs485_proc_delta / 구: reserved0[0])\n", sys[16]);

    // 구 펌웨어는 proc_delta 가 DIAG 228 에 있다. 둘 다 읽어 어느 쪽이 살아있는지 본다.
    uint8_t old_delta = 0;
    ReadRaw(228, 1, &old_delta);
    printf("    addr228 = %u  (구: rs485_proc_delta / 신: reserved)\n", old_delta);

    // 여러 번 샘플링 — proc_delta 는 매 트랜잭션 갱신되므로 변동/비영이 곧 살아있다는 뜻
    int nz32 = 0, nz228 = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t a = 0, b = 0;
        ReadRaw(32, 1, &a); ReadRaw(228, 1, &b);
        if (a) nz32++;
        if (b) nz228++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("    10회 샘플 중 비영: addr32=%d/10  addr228=%d/10\n", nz32, nz228);

    bool is_new = (nz32 > 0 && nz228 == 0);
    bool is_old = (nz228 > 0 && nz32 == 0);
    if (is_new)      printf("  → 판정: **신 펌웨어** (proc_delta 가 addr 32) ✔ 브리지 미러와 일치\n");
    else if (is_old) printf("  → 판정: **구 펌웨어** (proc_delta 가 addr 228) ✗ 현 브리지와 불일치\n");
    else             printf("  → 판정: **불명확** — 아래 수치를 보고 판단할 것\n");

    // ---------- B. ECU 살아있음 ----------
    printf("\n[B] realtime_tick 증가 확인\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    uint8_t sys2[17] = {};
    if (ReadRaw(ecu::REG_SYS_OFFSET, 17, sys2)) {
        uint32_t tick1; memcpy(&tick1, &sys2[12], 4);
        printf("  tick %u → %u (Δ=%d, 200ms 경과 → 0.1ms 단위면 ~2000 기대)\n",
               tick0, tick1, static_cast<int>(tick1 - tick0));
        printf("  → %s\n", (tick1 != tick0) ? "✔ ECU 태스크 동작 중" : "✗ tick 정지 — ECU 멈춤?");
    }

    // ---------- C. P8 게이트 — DIRECT RW 87B ----------
    printf("\n[C] P8 게이트 — DIRECT RW (write 128:52, 요청 79B payload / 87B 와이어) ×%d\n", n_p8);
    printf("    ⚠ shadow 를 0 으로 두고 보낸다 — cmd_motor 전 구간 0 = 무토크\n");

    memset(&g_state.ecu.reg, 0, sizeof(g_state.ecu.reg));   // 전 구간 0

    if (!WriteByte(ecu::REG_AUTO_MODE_OFFSET, ecu::AUTO_MODE_DIRECT)) {
        printf("  auto_mode=DIRECT 설정 실패 — P8 게이트 생략\n");
    } else {
        uint8_t am = 0xFF;
        ReadRaw(ecu::REG_AUTO_MODE_OFFSET, 1, &am);
        printf("  auto_mode read-back = %u %s\n", am, am == 2 ? "✔" : "✗");

        TaskConfig_t direct{TARGET::ECU, PacketInst::RW,
            ecu::REG_CMD_MOTOR_OFFSET, ecu::REG_CMD_MOTOR_SIZE, {
            {27, 5}, {ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE},
            {ecu::REG_MOTOR_DATA_OFFSET, 36}, {164, 16},
            {ecu::REG_PROC_DELTA_OFFSET, 1}, {ecu::REG_CMD_MOTOR_OFFSET, 4}}};

        // 요청 크기를 실제로 확인 (87B 여야 P8 재현 조건)
        size_t dl = 0;
        g_map.Encode(direct, &g_state, &g_pkt, &dl);
        printf("  요청 payload=%zuB, 와이어=%zuB %s\n", dl, dl + 8,
               (dl + 8 == 87) ? "✔ P8 재현 조건" : "⚠ 87B 아님");

        int fatal = 0, err = 0, tmo = 0, ok = 0, decode_err = 0, werr = 0;
        double rtt_sum = 0, rtt_max = 0;
        for (int i = 0; i < n_p8; i++) {
            TxnResult r = Txn(direct);
            if (r.write == RD_FATAL || r.read == RD_FATAL) fatal++;
            else if (r.read == RD_TIMEOUT) tmo++;
            else if (!r.ok && r.decode != RD_OK) decode_err++;
            else if (!r.ok) err++;
            else { ok++; rtt_sum += r.rtt_ms; if (r.rtt_ms > rtt_max) rtt_max = r.rtt_ms; }
            if (r.ok && (r.err0 >> 4)) werr++;      // write_err 니블
            std::this_thread::sleep_for(std::chrono::milliseconds(5));   // 200Hz
        }
        printf("  결과: ok=%d  RD_FATAL=%d  timeout=%d  err=%d  decode_err=%d  write_err니블=%d\n",
               ok, fatal, tmo, err, decode_err, werr);
        if (ok) printf("  rtt: 평균 %.2fms / 최대 %.2fms\n", rtt_sum / ok, rtt_max);
        printf("  → P8 게이트: %s (통과 조건 = RD_FATAL 0)\n",
               fatal == 0 ? "✔ 통과" : "✗ 실패");
    }

    // ---------- D. auto_mode 4/5 수용 ----------
    printf("\n[D] auto_mode VELOCITY(4)/POSITION(5) 수용 — STM #3\n");
    for (uint8_t m : {uint8_t{4}, uint8_t{5}}) {
        if (!WriteByte(ecu::REG_AUTO_MODE_OFFSET, m)) continue;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        uint8_t rb = 0xFF, ctr[4] = {};
        ReadRaw(ecu::REG_AUTO_MODE_OFFSET, 1, &rb);
        ReadRaw(ecu::REG_CMD_MOTOR_OFFSET, 4, ctr);
        printf("  auto_mode=%u → read-back %u %s | ctr_mode[128..131] = %u %u %u %u\n",
               m, rb, (rb == m) ? "✔" : "✗", ctr[0], ctr[1], ctr[2], ctr[3]);
    }

    // 원복 — 남겨두면 다음 기동이 엉뚱한 모드로 시작한다
    printf("\n[정리] auto_mode → 1(CURRENT) 원복\n");
    memset(&g_state.ecu.reg, 0, sizeof(g_state.ecu.reg));
    WriteByte(ecu::REG_AUTO_MODE_OFFSET, ecu::AUTO_MODE_CURRENT);
    uint8_t fin = 0xFF;
    ReadRaw(ecu::REG_AUTO_MODE_OFFSET, 1, &fin);
    printf("  read-back = %u %s\n", fin, fin == 1 ? "✔" : "✗");

    comm.Stop();
    printf("\n=== 완료 ===\n");
    return 0;
}
