#include <gtest/gtest.h>
#include <hdmap_engine/core/kdtree/kdtree.h>
#include <hdmap_engine/engine.h>

#include <cstdint>
#include <memory>

class TestKDTree : public testing::Test {
 protected:
  static hdmap::kdtree::KDTree::Ptr kdtree_;
};

hdmap::kdtree::KDTree::Ptr TestKDTree::kdtree_ =
    std::make_shared<hdmap::kdtree::KDTree>();

/**
 * build: colcon build
 * test: colcon test
 * result: colcon test-result --all --verbose
 * */
TEST_F(TestKDTree, TestBuild) {
  ASSERT_TRUE(nullptr != kdtree_);
  hdmap::geometry::Curve::Points pts;
  for (int i = 0; i < 800; i++) {
    hdmap::geometry::Curve::Point point;
    point.set_x(i);
    point.set_y(i + 1);
    point.set_z(0);
    pts.emplace_back(point);
  }
  kdtree_->Init(pts);
}

TEST_F(TestKDTree, TestSearch) {
  ASSERT_TRUE(nullptr != kdtree_);
  const double abs_error = 0.000001;
  {
    const double target_x = 0;
    const double target_y = 1;
    const int num_closest = 1;
    auto search_results = kdtree_->Query(target_x, target_y, num_closest);
    ASSERT_TRUE(1 == search_results.size());
    ASSERT_NEAR(target_x, search_results.begin()->x, abs_error);
    ASSERT_NEAR(target_y, search_results.begin()->y, abs_error);
    ASSERT_NEAR(0, search_results.begin()->dist, abs_error);
  }

  {
    const double target_x = 0;
    const double target_y = 0;
    const int num_closest = 1;
    auto search_results = kdtree_->Query(target_x, target_y, num_closest);
    ASSERT_TRUE(1 == search_results.size());
    ASSERT_NEAR(target_x, search_results.begin()->x, abs_error);
    ASSERT_NEAR(1, search_results.begin()->y, abs_error);
    ASSERT_NEAR(1, search_results.begin()->dist, abs_error);
  }

  {
    const double target_x = 0;
    const double target_y = 0;
    const int num_closest = 2;
    auto search_results = kdtree_->Query(target_x, target_y, num_closest);
    ASSERT_TRUE(2 == search_results.size());
  }
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
