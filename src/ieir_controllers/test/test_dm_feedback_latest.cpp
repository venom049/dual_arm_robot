// Real hardware plugin, synthetic MotorStateArray only. Never activate/write.
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
#include <unistd.h>
#include "ieir_controllers/dm_hardware_interface.hpp"

using namespace std::chrono_literals;
using Hardware=ieir_controllers::DMHardwareInterface;
using Frame=w3_robot_bridge::msg::MotorStateArray;
using JS=sensor_msgs::msg::JointState;

class FeedbackLatest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "196") << "Isolated test domain required";
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    calibration=std::filesystem::temp_directory_path()/(
      "feedback_latest_"+std::to_string(getpid())+".yaml");
    std::ofstream(calibration) << "offsets:\n- {name: joint0, zero_offset: 0.1, axis_sign: -1.0}\n"
      "- {name: joint1, zero_offset: -0.2, axis_sign: 1.0}\n";
    hardware_interface::HardwareInfo info;info.name="isolated_latest";info.type="system";
    info.hardware_parameters={{"offsets_yaml",calibration.string()},
      {"motor_topic_ns","/isolated_feedback_latest"},{"auto_enable","false"}};
    for (int j=0;j<2;++j) {
      hardware_interface::ComponentInfo joint;joint.name="joint"+std::to_string(j);joint.type="joint";
      joint.parameters={{"channel","1"},{"slot",std::to_string(j)},{"motor_type","DM4310"}};
      for (const auto &name:{"position","velocity","effort","stiffness","damping"}) {
        hardware_interface::InterfaceInfo iface;iface.name=name;joint.command_interfaces.push_back(iface);
      }
      for (const auto &name:{"position","velocity","effort"}) {
        hardware_interface::InterfaceInfo iface;iface.name=name;joint.state_interfaces.push_back(iface);
      }
      info.joints.push_back(joint);
    }
    hardware=std::make_unique<Hardware>();
    ASSERT_EQ(hardware->on_init(info),hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),hardware_interface::CallbackReturn::SUCCESS);
    states=hardware->export_state_interfaces();
    node=std::make_shared<rclcpp::Node>("isolated_feedback_test");
    publisher=node->create_publisher<Frame>("/isolated_feedback_latest/state",rclcpp::QoS(50).best_effort());
    subscription=node->create_subscription<JS>("/dm_hardware_interface/feedback_state",rclcpp::SensorDataQoS(),
      [this](JS::ConstSharedPtr m){outputs.push_back(*m);});
    auto deadline=std::chrono::steady_clock::now()+2s;
    while (publisher->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline) pump();
    ASSERT_GT(publisher->get_subscription_count(),0U);
    pump_for(30ms);
  }
  void TearDown() override
  {
    subscription.reset();publisher.reset();node.reset();states.clear();hardware.reset();
    std::filesystem::remove(calibration);
  }
  void pump()
  {
    hardware->read(rclcpp::Time(0),rclcpp::Duration(0,3333333));
    rclcpp::spin_some(node);std::this_thread::sleep_for(1ms);
  }
  void pump_for(std::chrono::milliseconds duration)
  {
    auto end=std::chrono::steady_clock::now()+duration;
    while(std::chrono::steady_clock::now()<end) pump();
  }
  double state(int j,const std::string &iface)
  {
    for (auto &v:states) if (v.get_name()=="joint"+std::to_string(j)+"/"+iface) return v.get_value();
    ADD_FAILURE()<<"Missing state interface "<<iface;return NAN;
  }
  Frame frame(double q)
  {
    Frame msg;msg.header.stamp=node->now();
    for (int j=0;j<2;++j) {
      w3_robot_bridge::msg::MotorState m;m.channel=1;m.motor_index=j;m.can_id=j+1;
      m.header=msg.header;m.online=true;m.enabled=true;m.error_flags=1;
      m.position=j==0?.1-q:q-.2;m.velocity=j==0?-.03:.03;m.torque=j==0?-.4:.4;
      msg.motors.push_back(m);
    }
    return msg;
  }
  void deliver(const Frame &msg) {publisher->publish(msg);pump_for(10ms);}
  std::unique_ptr<Hardware> hardware;
  std::vector<hardware_interface::StateInterface> states;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Frame>::SharedPtr publisher;
  rclcpp::Subscription<JS>::SharedPtr subscription;
  std::vector<JS> outputs;
  std::filesystem::path calibration;
};

TEST_F(FeedbackLatest, BurstConsumesLatestAndPreservesSourceStampAndUnits)
{
  Frame last;
  for(int i=0;i<40;++i) {last=frame(.001*i);publisher->publish(last);}
  std::this_thread::sleep_for(8ms);pump();pump_for(5ms);
  EXPECT_NEAR(state(0,"position"),.039,1e-7);
  EXPECT_NEAR(state(1,"velocity"),.03,1e-7);
  EXPECT_NEAR(state(0,"effort"),.4,1e-7);
  EXPECT_EQ(state(0,"feedback_valid"),1.0);
  ASSERT_FALSE(outputs.empty());
  EXPECT_EQ(outputs.back().header.stamp,last.header.stamp);
  EXPECT_NEAR(outputs.back().position[0],.039,1e-7);
  EXPECT_EQ(hardware->export_command_interfaces().size(),10U);
}

TEST_F(FeedbackLatest, DoesNotRelabelStaleValuesAsFreshOrZero)
{
  EXPECT_TRUE(std::isnan(state(0,"feedback_source_time")));
  EXPECT_EQ(state(0,"feedback_valid"),0.0);
  auto msg=frame(.2);deliver(msg);ASSERT_FALSE(outputs.empty());
  const auto count=outputs.size();pump_for(70ms);
  EXPECT_NEAR(state(0,"position"),.2,1e-7);
  EXPECT_EQ(state(0,"feedback_valid"),0.0);
  EXPECT_GT(state(0,"feedback_receive_age"),.05);
  EXPECT_EQ(outputs.size(),count);
}

TEST_F(FeedbackLatest, RejectsReplayFutureOldAndMalformedWithoutPoisoningRecovery)
{
  auto original=frame(.1);deliver(original);
  auto bad=original;bad.motors[0].position=-.9;deliver(bad); // identical/replayed
  EXPECT_NEAR(state(0,"position"),.1,1e-7);
  bad=frame(.9);bad.header.stamp.sec+=30;deliver(bad);
  EXPECT_NEAR(state(0,"position"),.1,1e-7);
  bad=frame(.9);bad.header.stamp.sec-=1;deliver(bad);
  EXPECT_NEAR(state(0,"position"),.1,1e-7);
  bad=frame(.9);bad.header.stamp.nanosec=1000000000U;deliver(bad);
  bad=frame(.9);bad.header.stamp.sec=0;bad.header.stamp.nanosec=0;deliver(bad);
  EXPECT_NEAR(state(0,"position"),.1,1e-7);
  deliver(frame(.3));EXPECT_NEAR(state(0,"position"),.3,1e-7);
  EXPECT_EQ(state(0,"feedback_valid"),1.0);
}

TEST_F(FeedbackLatest, MissingOfflineAndNonfiniteJointsAreNotPublishedAsComplete)
{
  deliver(frame(.1));ASSERT_FALSE(outputs.empty());auto count=outputs.size();
  auto partial=frame(.2);partial.motors.pop_back();deliver(partial);
  EXPECT_EQ(state(1,"feedback_valid"),0.0);EXPECT_EQ(outputs.size(),count);
  auto offline=frame(.3);offline.motors[1].online=false;deliver(offline);
  EXPECT_NEAR(state(1,"position"),.1,1e-7);EXPECT_EQ(outputs.size(),count);
  auto nan=frame(.4);nan.motors[1].velocity=NAN;deliver(nan);
  EXPECT_EQ(state(1,"feedback_valid"),0.0);EXPECT_EQ(outputs.size(),count);
  deliver(frame(.5));EXPECT_EQ(state(1,"feedback_valid"),1.0);
  EXPECT_NEAR(state(1,"position"),.5,1e-7);EXPECT_GT(outputs.size(),count);
}

TEST_F(FeedbackLatest, MixedSourceStampUsesOldestAndOldMotorCannotRefreshCache)
{
  auto msg=frame(.1);const auto oldest=rclcpp::Time(msg.header.stamp)-rclcpp::Duration(0,1000000);
  msg.motors[0].header.stamp=oldest;deliver(msg);ASSERT_FALSE(outputs.empty());
  EXPECT_EQ(outputs.back().header.stamp,static_cast<builtin_interfaces::msg::Time>(oldest));
  auto newer=frame(.2);newer.motors[0].header.stamp=oldest;auto count=outputs.size();deliver(newer);
  EXPECT_NEAR(state(0,"position"),.1,1e-7);EXPECT_EQ(state(0,"feedback_valid"),0.0);
  EXPECT_EQ(outputs.size(),count);
}

TEST_F(FeedbackLatest, FasterProducerAndConsumerPauseDoNotBuildFeedbackHistory)
{
  std::atomic<bool> running{true};std::atomic<int> sequence{0};
  std::thread producer([&] {
    auto next=std::chrono::steady_clock::now();
    while(running) {publisher->publish(frame(.0001*++sequence));next+=2857143ns;std::this_thread::sleep_until(next);}
  });
  std::vector<double> ages;auto start=std::chrono::steady_clock::now();auto next=start;
  while(std::chrono::steady_clock::now()-start<3s) {
    hardware->read(rclcpp::Time(0),rclcpp::Duration(0,3333333));
    if(std::chrono::steady_clock::now()-start>1s) ages.push_back(state(0,"feedback_source_age"));
    next+=3333333ns;std::this_thread::sleep_until(next);
  }
  // Explicitly pause reads while the producer continues, then resume once.
  std::this_thread::sleep_for(200ms);pump();const double resumed_age=state(0,"feedback_source_age");
  running=false;producer.join();
  std::sort(ages.begin(),ages.end());ASSERT_FALSE(ages.empty());
  const double p95=ages[size_t(ages.size()*.95)];
  std::cout<<"LATEST_FEEDBACK_METRICS source_age_p95_ms="<<p95*1000
    <<" resumed_age_ms="<<resumed_age*1000<<" samples="<<ages.size()<<std::endl;
  EXPECT_LT(p95,.02);EXPECT_LT(resumed_age,.02);
  EXPECT_EQ(state(0,"feedback_valid"),1.0);
}
