#include <gtest/gtest.h>
#include <opendrive-engine/common/param.h>
#include <opendrive-engine/engine.h>
#include <tinyxml2.h>

#include <cassert>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opendrive-cpp/common/common.hpp"

class TestEmpty : public testing::Test {
 public:
  static void SetUpTestCase();     // 在第一个case之前执行
  static void TearDownTestCase();  // 在最后一个case之后执行
  void SetUp() override;           // 在每个case之前执行
  void TearDown() override;        // 在每个case之后执行
  static opendrive::engine::Engine* GetEngine() {
    static opendrive::engine::Engine* instance = nullptr;
    if (!instance) {
      static std::once_flag flag;
      std::call_once(flag, [&] {
        instance = new (std::nothrow) opendrive::engine::Engine();
        opendrive::engine::common::Param param;
        param.map_file = MAP_FILE;
        instance->Init(param);
      });
    }
    return instance;
  }
  static std::string MAP_FILE;
};

std::string TestEmpty::MAP_FILE =
    "/opt/xodr/share/xodr/carla-simulator/Town01.xodr";
void TestEmpty::SetUpTestCase() {}
void TestEmpty::TearDownTestCase() {}
void TestEmpty::TearDown() {}
void TestEmpty::SetUp() {}

TEST_F(TestEmpty, TestInit) {
  auto engine = TestEmpty::GetEngine();
  ASSERT_TRUE(engine->GetLanes().size() > 0);
}

TEST_F(TestEmpty, TestPointId) {
  auto engine = TestEmpty::GetEngine();
  ASSERT_TRUE(engine->GetLanes().size() > 0);
  auto lane = engine->GetLanes().front();
  int point_size = lane->central_curve().pts().size();
  ASSERT_EQ(lane->central_curve().pts().size(),
            lane->left_boundary().curve().pts().size());
  ASSERT_EQ(lane->central_curve().pts().size(),
            lane->right_boundary().curve().pts().size());
  for (int i = 0; i < point_size; i++) {
    auto split =
        opendrive::common::Split(lane->central_curve().pts().at(i).id(), "_");
    ASSERT_EQ(4, split.size());
    ASSERT_EQ(i, std::atoi(split.at(3).c_str()));
    auto left_split = opendrive::common::Split(
        lane->left_boundary().curve().pts().at(i).id(), "_");
    ASSERT_EQ(4, left_split.size());
    ASSERT_EQ(i, std::atoi(left_split.at(3).c_str()));
    auto right_split = opendrive::common::Split(
        lane->right_boundary().curve().pts().at(i).id(), "_");
    ASSERT_EQ(4, right_split.size());
    ASSERT_EQ(i, std::atoi(right_split.at(3).c_str()));
    ASSERT_EQ(split.at(0), left_split.at(0));
    ASSERT_EQ(split.at(1), left_split.at(1));
    ASSERT_EQ(split.at(2), left_split.at(2));
    ASSERT_EQ(split.at(3), left_split.at(3));
    ASSERT_EQ(split.at(0), right_split.at(0));
    ASSERT_EQ(split.at(1), right_split.at(1));
    ASSERT_EQ(split.at(2), right_split.at(2));
    ASSERT_EQ(split.at(3), right_split.at(3));
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
