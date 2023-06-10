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
#include "opendrive-engine/geometry/geometry.h"

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
  for (const auto& lane : engine->GetLanes()) {
    for (const auto& point : lane->central_curve().pts()) {
      ASSERT_EQ(5, opendrive::common::Split(point.id(), "_").size());
    }
    for (const auto& point : lane->left_boundary().curve().pts()) {
      ASSERT_EQ(5, opendrive::common::Split(point.id(), "_").size());
    }
    for (const auto& point : lane->right_boundary().curve().pts()) {
      ASSERT_EQ(5, opendrive::common::Split(point.id(), "_").size());
    }
  }
  auto first_lane = engine->GetLanes().front();
  int point_size = first_lane->central_curve().pts().size();
  ASSERT_EQ(first_lane->central_curve().pts().size(),
            first_lane->left_boundary().curve().pts().size());
  ASSERT_EQ(first_lane->central_curve().pts().size(),
            first_lane->right_boundary().curve().pts().size());
  for (int i = 0; i < point_size; i++) {
    auto split = opendrive::common::Split(
        first_lane->central_curve().pts().at(i).id(), "_");
    ASSERT_EQ(5, split.size());
    ASSERT_EQ(i, std::atoi(split.at(3).c_str()));
    auto left_split = opendrive::common::Split(
        first_lane->left_boundary().curve().pts().at(i).id(), "_");
    ASSERT_EQ(5, left_split.size());
    ASSERT_EQ(i, std::atoi(left_split.at(3).c_str()));
    auto right_split = opendrive::common::Split(
        first_lane->right_boundary().curve().pts().at(i).id(), "_");
    ASSERT_EQ(5, right_split.size());
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

TEST_F(TestEmpty, TestGetNearestPoint) {
  auto engine = TestEmpty::GetEngine();
  ASSERT_TRUE(nullptr != engine);
  opendrive::engine::geometry::Point2D point2d;  // id 207_1_-1_18_2
  point2d.set_x(88.6349);
  point2d.set_y(-330.582);
  auto search_ret = engine->GetNearestPoints(point2d, 1);
  ASSERT_EQ(1, search_ret.size());
  ASSERT_FLOAT_EQ(88.6349, search_ret.front().x);
  ASSERT_FLOAT_EQ(-330.582, search_ret.front().y);
  ASSERT_TRUE(search_ret.front().dist < 0.001);
  ASSERT_EQ("207_1_-1_18_2", search_ret.front().id);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
