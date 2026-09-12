#include <gtest/gtest.h>
#include <go2_control/classic_gait.hpp>
#include <string>
#include <vector>
namespace {
struct FakeSport {
  int move_code=0, classic_code=0;
  std::vector<std::string> calls;
  int Move(float x,float y,float yaw) {
    EXPECT_FLOAT_EQ(0,x); EXPECT_FLOAT_EQ(0,y); EXPECT_FLOAT_EQ(0,yaw);
    calls.push_back("zero"); return move_code;
  }
  int ClassicWalk(bool on) {
    EXPECT_TRUE(on); calls.push_back("classic_on"); return classic_code;
  }
};
}
TEST(ClassicGait, EnableExplicitlyRequestsClassicAfterZero) {
  FakeSport sdk;
  EXPECT_TRUE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ((std::vector<std::string>{"zero","classic_on"}),sdk.calls);
}
TEST(ClassicGait, FailedZeroCannotSelectGait) {
  FakeSport sdk; sdk.move_code=3102;
  EXPECT_FALSE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ((std::vector<std::string>{"zero"}),sdk.calls);
}
TEST(ClassicGait, ClassicRejectionCannotArmOrFallback) {
  for(int code:{3102,4205,7004}) {
    FakeSport sdk; sdk.classic_code=code;
    const auto r=go2_control::requestClassicWalk(sdk);
    EXPECT_FALSE(r.accepted()); EXPECT_EQ(code,r.classic_result);
    EXPECT_EQ(2U,sdk.calls.size());
  }
}
TEST(ClassicGait, EveryNewEnableReappliesClassic) {
  FakeSport sdk;
  EXPECT_TRUE(go2_control::requestClassicWalk(sdk).accepted());
  sdk.calls.clear();
  EXPECT_TRUE(go2_control::requestClassicWalk(sdk).accepted());
  EXPECT_EQ((std::vector<std::string>{"zero","classic_on"}),sdk.calls);
}
TEST(ClassicGait, CommandsThatChangeGaitOrPostureInvalidatePolicy) {
  for(int api:{1001,1003,1004,1005,1028,1061,1062,1063,2045,2049})
    EXPECT_TRUE(go2_control::changesSportMode(api));
  for(int api:{1008,1015,1034,2055})
    EXPECT_FALSE(go2_control::changesSportMode(api));
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS();
}
