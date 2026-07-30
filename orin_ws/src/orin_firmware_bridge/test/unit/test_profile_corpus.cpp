// 공유 프로파일 코퍼스 (redesign/05 §3.5)
//
// **C++ 과 Python 이 같은 판정을 내리는지** 고정한다. 두 구현이 갈라지면 lint 를 통과한
// 프로파일이 실기에서 거부되고, 그 원인을 찾는 시간은 대개 실험 준비 중에 든다.
//
// 이 테스트는 C++ 쪽만 본다. Python 쪽은 `lint_profiles.py` 를 같은 코퍼스에 돌려
// 대조한다 (test/profiles/README.md).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "orin_firmware_bridge/policy/rd_profile.hpp"

#ifndef RD_CORPUS_DIR
#define RD_CORPUS_DIR "test/profiles"
#endif

namespace {

std::string Slurp(const std::filesystem::path& p) {
    std::ifstream f(p);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::vector<std::filesystem::path> Corpus(const std::string& prefix) {
    std::vector<std::filesystem::path> out;
    for (const auto& e : std::filesystem::directory_iterator(RD_CORPUS_DIR)) {
        const auto n = e.path().filename().string();
        if (n.rfind(prefix, 0) == 0 && e.path().extension() == ".yaml") out.push_back(e.path());
    }
    return out;
}

TEST(ProfileCorpus, AllOkProfilesLoad) {
    const auto files = Corpus("ok_");
    ASSERT_FALSE(files.empty()) << "코퍼스를 못 찾았다 — RD_CORPUS_DIR=" << RD_CORPUS_DIR;
    for (const auto& f : files) {
        orin_bridge::RdProfile p;
        std::string err;
        // 전역 클램프는 넉넉히 — 코퍼스는 프로파일 자체의 유효성만 본다
        EXPECT_TRUE(p.LoadFromYaml(Slurp(f), 0x0F, 1000.0f, &err))
            << f.filename().string() << " 가 거부됐다: " << err;
    }
}

TEST(ProfileCorpus, AllRejectProfilesRejected) {
    const auto files = Corpus("reject_");
    ASSERT_FALSE(files.empty());
    for (const auto& f : files) {
        orin_bridge::RdProfile p;
        std::string err;
        EXPECT_FALSE(p.LoadFromYaml(Slurp(f), 0x0F, 1000.0f, &err))
            << f.filename().string() << " 가 수락됐다 — 05 §3.3 제약이 빠졌다";
    }
}

}  // namespace
