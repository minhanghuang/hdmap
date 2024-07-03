#include <gtest/gtest.h>
#include <hdmap_common/util.h>

#include <string>
#include <vector>

class TestCommon : public testing::Test {};

TEST_F(TestCommon, TestSplit) {
  const std::string str = "111_222_333_444";
  std::vector<std::string> ret = hdmap::common::Split(str, "_");
  ASSERT_TRUE(4 == ret.size());
  ASSERT_TRUE("111" == ret.at(0));
  ASSERT_TRUE("222" == ret.at(1));
  ASSERT_TRUE("333" == ret.at(2));
  ASSERT_TRUE("444" == ret.at(3));
}

TEST_F(TestCommon, TestGetLaneIdById) {
  {
    const std::string point_id = "0_1_0_100_1";
    const std::string lane_id = hdmap::common::GetLaneIdByPointId(point_id);
    ASSERT_TRUE(!lane_id.empty());
    ASSERT_TRUE("0_1_0" == lane_id);
  }

  {
    const std::string point_id = "0_1_0_100_";
    const std::string lane_id = hdmap::common::GetLaneIdByPointId(point_id);
    std::cout << "lane_id: " << lane_id << std::endl;
    ASSERT_TRUE(lane_id.empty());
  }
}

TEST_F(TestCommon, TestMinValue) {
  const double value1 = 2.0;
  const float value2 = 5.0f;
  const int value3_min = 1;
  ASSERT_TRUE(value3_min == hdmap::common::MinValue(value3_min));
  ASSERT_TRUE(value3_min ==
              hdmap::common::MinValue(value1, value2, value3_min));
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
