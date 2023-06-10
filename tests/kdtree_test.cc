#include "opendrive-engine/algo/kdtree/kdtree.h"

#include <gtest/gtest.h>
#include <opendrive-engine/common/param.h>
#include <opendrive-engine/engine.h>
#include <tinyxml2.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nanoflann.hpp"
#include "opendrive-cpp/common/common.hpp"

class TestKDTree : public testing::Test {
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

std::string TestKDTree::MAP_FILE =
    "/opt/xodr/share/xodr/carla-simulator/Town01.xodr";
void TestKDTree::SetUpTestCase() {}
void TestKDTree::TearDownTestCase() {}
void TestKDTree::TearDown() {}
void TestKDTree::SetUp() {}

TEST_F(TestKDTree, TestKDTreeAll) {
  opendrive::engine::kdtree::KDTree kdtree;
  // generate samples
  opendrive::engine::kdtree::SamplePoints samples;
  for (int i = 0; i < 1000; i++) {
    opendrive::engine::kdtree::SamplePoint point;
    point.mutable_x() = i;
    point.mutable_y() = i + 1;
    point.set_id(std::to_string(i));
    samples.emplace_back(point);
  }
  kdtree.Init(samples);
  opendrive::engine::kdtree::SamplePoint target_node(1, 2);
  auto knn_ret = kdtree.Query(target_node, 2);
  ASSERT_EQ(2, knn_ret.size());
  ASSERT_EQ(1, knn_ret.front().x);
  ASSERT_EQ(2, knn_ret.front().y);
  ASSERT_EQ("1", knn_ret.front().id);
  ASSERT_DOUBLE_EQ(0, knn_ret.front().dist);
}

TEST_F(TestKDTree, TestRadius) {
  opendrive::engine::kdtree::KDTree kdtree;
  opendrive::engine::kdtree::SamplePoints samples;
  for (int i = 0; i < 1000; i++) {
    opendrive::engine::kdtree::SamplePoint point;
    point.mutable_x() = i;
    point.mutable_y() = i + 1;
    point.set_id(std::to_string(i));
    samples.emplace_back(point);
  }
  kdtree.Init(samples);
  opendrive::engine::kdtree::SamplePoint target_node(1.2, 2.3);
  auto knn_ret = kdtree.QueryByRadius(target_node, 0.1);
  ASSERT_EQ(0, knn_ret.size());
  ASSERT_EQ(1, kdtree.QueryByRadius(0, 1, 0.1).size());
  ASSERT_EQ(1, kdtree.QueryByRadius(0, 1, 1).size());
  knn_ret = kdtree.QueryByRadius(0, 1, 2);
  ASSERT_EQ(2, knn_ret.size());
  ASSERT_EQ(0, knn_ret.front().x);
  ASSERT_EQ(1, knn_ret.front().y);
  ASSERT_EQ(0, knn_ret.front().dist);
  ASSERT_EQ(1, knn_ret.back().x);
  ASSERT_EQ(2, knn_ret.back().y);
  ASSERT_EQ(std::sqrt(std::pow(knn_ret.back().x - knn_ret.front().x, 2) +
                      std::pow(knn_ret.back().y - knn_ret.front().y, 2)),
            knn_ret.back().dist);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
