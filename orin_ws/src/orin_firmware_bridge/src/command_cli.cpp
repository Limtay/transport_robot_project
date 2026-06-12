// Command 입력 CLI 노드 (Code_modify.md §2 "Command 입력 node")
//
//  bridge(firmware_bridge_node) 의 서비스를 호출하는 대화형 터미널 노드.
//  Orin 에서 bridge 실행 + 노트북에서 본 노드 실행 (같은 ROS_DOMAIN_ID) 구조 지원.
//
//  사용법:
//      ros2 run orin_firmware_bridge command_cli
//
//  명령 문법은 아래 PrintHelp() 참조.

#include <rclcpp/rclcpp.hpp>
#include <mgs01_base_msgs/srv/command_set.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <cstring>
#include <iostream>
#include <thread>
#include <sstream>
#include <string>
#include <vector>
#include <map>

using CommandSet = mgs01_base_msgs::srv::CommandSet;
using SetBool    = std_srvs::srv::SetBool;
using namespace std::chrono_literals;

namespace {

constexpr uint8_t TARGET_ECU = 0xE1;
constexpr uint8_t TARGET_DPC = 0xD2;
constexpr uint8_t TARGET_PCU = 0xA1;

// ECU reg5/reg54 hw_reset 비트 (STM HARDWARE_STATUS_t 와 동일)
const std::map<std::string, uint8_t> kHwResetBits = {
    {"uart1", 1u << 0}, {"uart2", 1u << 1}, {"uart6", 1u << 2},
    {"can1",  1u << 3}, {"i2c1",  1u << 4},
};

void PrintHelp() {
    std::cout <<
        "\n===== Command CLI =====\n"
        "set <slot|auto> <dur> <target> read <addr> <len>      : READ — 수신값 bridge 터미널에 출력\n"
        "set <slot|auto> <dur> <target> writereg <addr> <len>  : 기능1 — bridge reg 섀도 값 전송\n"
        "set <slot|auto> <dur> <target> write <addr> <val...>  : 기능2 — 사용자 데이터 전송\n"
        "set <slot|auto> <dur> <target> reboot                 : 보드 재부팅 (3초 접근 차단)\n"
        "reset <slot>                                          : 슬롯 공란으로\n"
        "macro hw_reset <uart1|uart2|uart6|can1|i2c1>          : ECU reg5 비트 write (unlock→write→lock 자동)\n"
        "macro unlock <on|off>                                 : ECU DEFINE(1~15) 쓰기 잠금 해제/잠금 (reg0)\n"
        "macro jeongae_lock <on|off>                           : jeongae 전개 lock 토글\n"
        "macro dpc_led <on|off>                                : TODO — DPC 레지스터 미정\n"
        "macro pcu_relay <off|on|reboot>                       : TODO — PCU 레지스터 미정\n"
        "macro gongbyeok <1|2> <open|close|default>            : TODO — DPC 레지스터 미정\n"
        "help / quit\n"
        "---------------------------------------------\n"
        "  slot   : 0~3 (낮을수록 우선순위 높음) 또는 auto (빈 슬롯 자동)\n"
        "  dur    : forever | once (RET_OK 까지 반복, 2s timeout) | 2~100 (초)\n"
        "  target : ecu | dpc | pcu\n"
        "  addr   : 십진수 또는 0x 16진수\n"
        "  val    : 타입 캐스팅 자동 변환 — 예) (f)20.5 (u32)15 (i16)-5 (u8)3, 무표기=u8\n"
        "           {(float)20, (uint32)15} → 8바이트 little-endian 으로 변환 전송\n"
        "=======================================\n\n";
}

bool ParseUint(const std::string& tok, uint32_t* out) {
    try {
        *out = static_cast<uint32_t>(std::stoul(tok, nullptr, 0));  // 0x 자동 인식
        return true;
    } catch (...) { return false; }
}

// "(f)20.5" / "(u32)15" / "(i16)-5" / "15" → little-endian 바이트로 변환해 append
bool AppendTypedValue(const std::string& tok, std::vector<uint8_t>* out) {
    std::string type = "u8";
    std::string val  = tok;
    if (tok.size() > 2 && tok[0] == '(') {
        auto close = tok.find(')');
        if (close == std::string::npos || close + 1 >= tok.size()) return false;
        type = tok.substr(1, close - 1);
        val  = tok.substr(close + 1);
    }

    auto push = [out](const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        out->insert(out->end(), b, b + n);
    };

    try {
        if (type == "f" || type == "float") {
            float v = std::stof(val); push(&v, 4);
        } else if (type == "u8" || type == "uint8") {
            uint8_t v = static_cast<uint8_t>(std::stoul(val, nullptr, 0)); push(&v, 1);
        } else if (type == "u16" || type == "uint16") {
            uint16_t v = static_cast<uint16_t>(std::stoul(val, nullptr, 0)); push(&v, 2);
        } else if (type == "u32" || type == "uint32") {
            uint32_t v = static_cast<uint32_t>(std::stoul(val, nullptr, 0)); push(&v, 4);
        } else if (type == "i8" || type == "int8") {
            int8_t v = static_cast<int8_t>(std::stol(val, nullptr, 0)); push(&v, 1);
        } else if (type == "i16" || type == "int16") {
            int16_t v = static_cast<int16_t>(std::stol(val, nullptr, 0)); push(&v, 2);
        } else if (type == "i32" || type == "int32") {
            int32_t v = static_cast<int32_t>(std::stol(val, nullptr, 0)); push(&v, 4);
        } else {
            std::cout << "  [오류] 알 수 없는 타입: (" << type << ") — f/u8/u16/u32/i8/i16/i32\n";
            return false;
        }
    } catch (...) {
        std::cout << "  [오류] 값 변환 실패: " << tok << "\n";
        return false;
    }
    return true;
}

bool ParseTarget(const std::string& tok, uint8_t* out) {
    if (tok == "ecu") { *out = TARGET_ECU; return true; }
    if (tok == "dpc") { *out = TARGET_DPC; return true; }
    if (tok == "pcu") { *out = TARGET_PCU; return true; }
    return false;
}

bool ParseDuration(const std::string& tok, uint16_t* out) {
    if (tok == "forever") { *out = 0; return true; }
    if (tok == "once")    { *out = 1; return true; }
    uint32_t v;
    if (ParseUint(tok, &v) && v >= 2 && v <= 100) { *out = static_cast<uint16_t>(v); return true; }
    return false;
}

} // namespace

class CommandCli : public rclcpp::Node {
public:
    CommandCli() : Node("command_cli_node") {
        cli_command_ = create_client<CommandSet>("/carrier/command_set");
        cli_lock_    = create_client<SetBool>("/carrier/jeongae_lock");
    }

    void Run() {
        PrintHelp();
        std::string line;
        while (rclcpp::ok()) {
            std::cout << "cmd> " << std::flush;
            if (!std::getline(std::cin, line)) break;

            std::istringstream iss(line);
            std::vector<std::string> tok;
            std::string t;
            while (iss >> t) tok.push_back(t);
            if (tok.empty()) continue;

            if (tok[0] == "quit" || tok[0] == "exit") break;
            if (tok[0] == "help")  { PrintHelp(); continue; }
            if (tok[0] == "set")   { HandleSet(tok);   continue; }
            if (tok[0] == "reset") { HandleReset(tok); continue; }
            if (tok[0] == "macro") { HandleMacro(tok); continue; }
            std::cout << "  [오류] 알 수 없는 명령 — help 입력\n";
        }
    }

private:
    rclcpp::Client<CommandSet>::SharedPtr cli_command_;
    rclcpp::Client<SetBool>::SharedPtr    cli_lock_;

    void CallCommand(CommandSet::Request::SharedPtr req) {
        if (!cli_command_->wait_for_service(2s)) {
            std::cout << "  [오류] /carrier/command_set 서비스 없음 — bridge 실행/네트워크 확인\n";
            return;
        }
        auto fut = cli_command_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut, 3s)
                != rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "  [오류] 서비스 응답 timeout\n";
            return;
        }
        auto res = fut.get();
        std::cout << (res->accepted ? "  [OK] " : "  [거부] ") << res->message << "\n";
    }

    // ECU 단일 바이트 once WRITE (매크로용)
    void SendOnceWrite(uint16_t addr, uint8_t value) {
        auto req        = std::make_shared<CommandSet::Request>();
        req->action     = CommandSet::Request::ACTION_SET;
        req->slot       = 255;  // 빈 슬롯 자동
        req->duration   = CommandSet::Request::DURATION_ONCE;
        req->target_id  = TARGET_ECU;
        req->inst       = CommandSet::Request::INST_WRITE_DATA;
        req->start_addr = addr;
        req->data       = {value};
        CallCommand(req);
    }

    void HandleSet(const std::vector<std::string>& tok) {
        // set <slot|auto> <dur> <target> <inst> [...]
        if (tok.size() < 5) { std::cout << "  [오류] 인자 부족 — help 참조\n"; return; }

        auto req = std::make_shared<CommandSet::Request>();
        req->action = CommandSet::Request::ACTION_SET;

        if (tok[1] == "auto") req->slot = 255;
        else {
            uint32_t v;
            if (!ParseUint(tok[1], &v) || v > 3) { std::cout << "  [오류] slot: 0~3 또는 auto\n"; return; }
            req->slot = static_cast<uint8_t>(v);
        }
        if (!ParseDuration(tok[2], &req->duration)) { std::cout << "  [오류] dur: forever|once|2~100\n"; return; }
        if (!ParseTarget(tok[3], &req->target_id))  { std::cout << "  [오류] target: ecu|dpc|pcu\n"; return; }

        const std::string& inst = tok[4];
        uint32_t addr = 0, len = 0;
        if (inst == "read" || inst == "writereg") {
            if (tok.size() != 7 || !ParseUint(tok[5], &addr) || !ParseUint(tok[6], &len)) {
                std::cout << "  [오류] " << inst << " <addr> <len>\n"; return;
            }
            req->inst       = (inst == "read") ? CommandSet::Request::INST_READ
                                               : CommandSet::Request::INST_WRITE_REG;
            req->start_addr = static_cast<uint16_t>(addr);
            req->data_len   = static_cast<uint16_t>(len);
        } else if (inst == "write") {
            if (tok.size() < 7 || !ParseUint(tok[5], &addr)) {
                std::cout << "  [오류] write <addr> <val...>\n"; return;
            }
            req->inst       = CommandSet::Request::INST_WRITE_DATA;
            req->start_addr = static_cast<uint16_t>(addr);
            for (size_t i = 6; i < tok.size(); i++) {
                if (!AppendTypedValue(tok[i], &req->data)) return;
            }
            std::cout << "  변환 결과: " << req->data.size() << " 바이트\n";
        } else if (inst == "reboot") {
            req->inst = CommandSet::Request::INST_REBOOT;
        } else {
            std::cout << "  [오류] inst: read|writereg|write|reboot\n"; return;
        }
        CallCommand(req);
    }

    void HandleReset(const std::vector<std::string>& tok) {
        uint32_t v;
        if (tok.size() != 2 || !ParseUint(tok[1], &v) || v > 3) {
            std::cout << "  [오류] reset <slot 0~3>\n"; return;
        }
        auto req    = std::make_shared<CommandSet::Request>();
        req->action = CommandSet::Request::ACTION_RESET;
        req->slot   = static_cast<uint8_t>(v);
        CallCommand(req);
    }

    void HandleMacro(const std::vector<std::string>& tok) {
        if (tok.size() < 2) { std::cout << "  [오류] macro <이름> [...]\n"; return; }
        const std::string& name = tok[1];

        // ECU hw_reset: reg5 에 해당 채널 비트 write (§2.4)
        // reg5 는 DEFINE 잠금 영역 → unlock(reg0=1) → reg5 write → lock(reg0=0) 순차 전송.
        // once 커맨드는 보통 1 프레임(200ms) 내 완료되므로 호출 간 0.5s 대기로 순서 보장.
        if (name == "hw_reset") {
            if (tok.size() != 3 || kHwResetBits.find(tok[2]) == kHwResetBits.end()) {
                std::cout << "  [오류] macro hw_reset <uart1|uart2|uart6|can1|i2c1>\n"; return;
            }
            std::cout << "  1/3 unlock (reg0=1)\n";
            SendOnceWrite(0, 1);
            std::this_thread::sleep_for(500ms);
            std::cout << "  2/3 hw_reset write (reg5=0x"
                      << std::hex << static_cast<int>(kHwResetBits.at(tok[2])) << std::dec << ")\n";
            SendOnceWrite(5, kHwResetBits.at(tok[2]));
            std::this_thread::sleep_for(500ms);
            std::cout << "  3/3 lock (reg0=0)\n";
            SendOnceWrite(0, 0);
            return;
        }

        // ECU DEFINE 잠금 토글: reg0 (sys_write_mode) — 항상 쓰기 가능한 unlock 키
        if (name == "unlock") {
            if (tok.size() != 3 || (tok[2] != "on" && tok[2] != "off")) {
                std::cout << "  [오류] macro unlock <on|off>\n"; return;
            }
            SendOnceWrite(0, (tok[2] == "on") ? 1 : 0);
            return;
        }

        // jeongae lock 토글 (§2.4: orin 기본 변수)
        if (name == "jeongae_lock") {
            if (tok.size() != 3 || (tok[2] != "on" && tok[2] != "off")) {
                std::cout << "  [오류] macro jeongae_lock <on|off>\n"; return;
            }
            if (!cli_lock_->wait_for_service(2s)) {
                std::cout << "  [오류] /carrier/jeongae_lock 서비스 없음\n"; return;
            }
            auto req  = std::make_shared<SetBool::Request>();
            req->data = (tok[2] == "on");
            auto fut  = cli_lock_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut, 3s)
                    != rclcpp::FutureReturnCode::SUCCESS) {
                std::cout << "  [오류] 서비스 응답 timeout\n"; return;
            }
            std::cout << "  [OK] " << fut.get()->message << "\n";
            return;
        }

        // TODO(§2.4): DPC/PCU 레지스터 미확정 매크로
        if (name == "dpc_led" || name == "pcu_relay" || name == "gongbyeok") {
            std::cout << "  [TODO] '" << name << "' 매크로는 DPC/PCU 레지스터 확정 후 구현 예정\n";
            return;
        }
        std::cout << "  [오류] 알 수 없는 매크로 — help 참조\n";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandCli>();
    node->Run();
    rclcpp::shutdown();
    return 0;
}
