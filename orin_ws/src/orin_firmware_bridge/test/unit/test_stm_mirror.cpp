// STM 펌웨어의 레지스터 헤더와 Orin 미러가 **바이트 단위로 같은지** 대조한다.
//
// 왜 필요한가: 두 헤더는 손으로 동기화되는데, 어긋나면 컴파일도 되고 테스트도 통과하면서
// 브리지가 엉뚱한 주소를 읽는다. 2026-07-27 SYS 재배치(01 §9.1) 때 실제로 위험했다 —
// `rd_register_ecu.hpp` 의 static_assert 는 **크기만** 보므로 필드 순서가 바뀌어도 통과한다.
// (같은 이유로 `rd_map_ecu.c` 의 memcpy 가 hw_reset↔hw_error 를 뒤바꿀 뻔했다.)
//
// 이 테스트는 stm_ws 가 함께 있을 때만 빌드된다 (CMakeLists 에서 헤더 존재 확인).
// Orin 에 배포된 워크스페이스에는 stm_ws 가 없으므로 그쪽에서는 조용히 빠진다.

#include <gtest/gtest.h>
#include <cstddef>

#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

namespace stm {
extern "C" {
#include "rd_register_ecu.h"
}
}  // namespace stm

namespace {

// 두 헤더에서 같은 이름의 필드가 같은 절대 주소에 있는지 본다.
#define EXPECT_SAME_SYS_OFFSET(field)                                             \
    EXPECT_EQ(offsetof(stm::REGISTER_t, sys) + offsetof(stm::DATA_SYSTEM_t, field), \
              offsetof(orin_bridge::ecu::REGISTER_t, sys) +                        \
                  offsetof(orin_bridge::ecu::DATA_SYSTEM_t, field))                \
        << "sys." #field " 의 절대 주소가 STM 과 다르다"

#define EXPECT_SAME_REG_OFFSET(field)                                             \
    EXPECT_EQ(offsetof(stm::REGISTER_t, field),                                    \
              offsetof(orin_bridge::ecu::REGISTER_t, field))                       \
        << "reg." #field " 의 주소가 STM 과 다르다"

TEST(StmMirror, TotalSize) {
    EXPECT_EQ(sizeof(stm::REGISTER_t), sizeof(orin_bridge::ecu::REGISTER_t));
    EXPECT_EQ(sizeof(stm::REGISTER_t), 256u);
}

TEST(StmMirror, SystemRegionLayout) {
    EXPECT_EQ(sizeof(stm::DATA_SYSTEM_t), sizeof(orin_bridge::ecu::DATA_SYSTEM_t));
    EXPECT_EQ(sizeof(stm::DATA_SYSTEM_t), 17u) << "01 §9.1: SYS 는 16→17B";

    EXPECT_SAME_SYS_OFFSET(degraded_cnt);
    EXPECT_SAME_SYS_OFFSET(hw_error);
    EXPECT_SAME_SYS_OFFSET(hw_fatal);
    EXPECT_SAME_SYS_OFFSET(hw_reset);
    EXPECT_SAME_SYS_OFFSET(sys_state);
    EXPECT_SAME_SYS_OFFSET(realtime_tick);
    EXPECT_SAME_SYS_OFFSET(rs485_proc_delta);
}

// 재배치 후 확정 주소 — 01 §9.1 표 그대로. 숫자를 직접 박아 의도를 고정한다.
TEST(StmMirror, AbsoluteAddresses) {
    using namespace orin_bridge::ecu;
    const size_t sys = offsetof(REGISTER_t, sys);
    EXPECT_EQ(sys, 16u);
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, degraded_cnt),     16u);
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, hw_error),         24u) << "구 26 에서 이동";
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, hw_fatal),         25u);
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, hw_reset),         26u) << "구 24 에서 이동";
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, sys_state),        27u);
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, realtime_tick),    28u);
    EXPECT_EQ(sys + offsetof(DATA_SYSTEM_t, rs485_proc_delta), 32u) << "구 DIAG 228 에서 이동";
    EXPECT_EQ(REG_PROC_DELTA_OFFSET, 32u);
}

// control 읽기 세그 {26,7} 이 hw_reset ~ proc_delta 를 정확히 덮는가 (01 §9.1 검산).
TEST(StmMirror, ControlSegmentFitsExactly) {
    using namespace orin_bridge::ecu;
    const size_t sys   = offsetof(REGISTER_t, sys);
    const size_t begin = sys + offsetof(DATA_SYSTEM_t, hw_reset);
    const size_t end   = sys + offsetof(DATA_SYSTEM_t, rs485_proc_delta) + 1;
    EXPECT_EQ(begin, 26u);
    EXPECT_EQ(end - begin, 7u) << "{26,7} 이 7B 가 아니면 프리셋 계산이 틀린다";
    EXPECT_EQ(REG_SYS_SIZE, 17u) << "{16,17} = SYS 전체";
}

// 하위 영역이 밀리지 않았는가 — 밀리면 기존 bag·분석의 오프셋 가정이 전부 깨진다.
TEST(StmMirror, LowerRegionsUnshifted) {
    EXPECT_SAME_REG_OFFSET(reg_df);
    EXPECT_SAME_REG_OFFSET(sys);
    EXPECT_SAME_REG_OFFSET(reserved0);
    EXPECT_SAME_REG_OFFSET(loadcell);
    EXPECT_SAME_REG_OFFSET(imu);
    EXPECT_SAME_REG_OFFSET(encoder);
    EXPECT_SAME_REG_OFFSET(uart2);
    EXPECT_SAME_REG_OFFSET(rc);
    EXPECT_SAME_REG_OFFSET(motor_data);
    EXPECT_SAME_REG_OFFSET(cmd_motor);
    EXPECT_SAME_REG_OFFSET(cmd_system);
    EXPECT_SAME_REG_OFFSET(diag);

    using namespace orin_bridge::ecu;
    EXPECT_EQ(offsetof(REGISTER_t, reserved0), 33u) << "RSVD0 32→33 (SYS 가 1B 먹었다)";
    EXPECT_EQ(offsetof(REGISTER_t, loadcell),  42u) << "여기부터는 절대 안 밀린다";
    EXPECT_EQ(offsetof(REGISTER_t, motor_data), 88u);
    EXPECT_EQ(offsetof(REGISTER_t, cmd_motor), 128u);
}

// STM 패킷 버퍼가 DIRECT RW 요청(87B)을 담는가 — P8 근본 수정 (01 §9.2).
// 값은 CMake 가 rd_uart.h / rd_comm_ecu.h 에서 뽑아 -D 로 넘긴다 (HAL 의존 회피).
TEST(StmMirror, PacketBuffersFitDirectRw) {
    // DIRECT RW 요청: payload = 1 + 4*6(세그) + 2(waddr) + 52(write) = 79 → 와이어 87B
    constexpr int kDirectWire = 87;
    EXPECT_GT(STM_RX_BUFFER_SIZE, kDirectWire)
        << "RX 링버퍼가 DIRECT 요청보다 작으면 오버런 → rx_length 가 (tail-head+N)%N 로 "
           "오계산 → 파싱/CRC 실패 → RD_FATAL. 이것이 P8 의 원인이었다 (01 §9.2)";
    EXPECT_GE(STM_PACKET_DATA_BUF_SIZE, 256) << "01 §9.2: 90→256";
    EXPECT_GE(STM_TX_BUFFER_SIZE, STM_PACKET_DATA_BUF_SIZE + 8) << "payload + 프레임 8B";
}

}  // namespace
