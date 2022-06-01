#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define TOTAL_STEPS 80000
#define REVOLUTION 6.28319

using namespace std::chrono_literals;
using std::placeholders::_1;

class JointStatePublisher : public rclcpp::Node
{
  public:
    JointStatePublisher()
    : Node("joint_state_publisher")
    {
      jointPublisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
      jointPublisherTimer_ = this->create_wall_timer(
        10ms, std::bind(&JointStatePublisher::jointPublisherTimerCallback, this)
      );

      jointSubscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/j0001/motor_state", 1, std::bind(&JointStatePublisher::jointSubscriberCallback, this, _1)
      );
    }
  private:
    void jointSubscriberCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      //get motor position in steps
      jointPosition_ = msg->data;
      //Convert motor steps into rads
      //80,000 steps = 6.28319 rad. Thus 1 step = 0.0000785399 rads
      jointPosition_ = (jointPosition_ * (REVOLUTION/TOTAL_STEPS));
      //round to two decimal point
      double tmp = (int)(jointPosition_ * 100 + 0.5);
      jointPosition_ = (double)tmp / 100;
    }
    void jointPublisherTimerCallback()
    {
      auto jointState = sensor_msgs::msg::JointState();
      auto currentTime = JointStatePublisher::get_clock();
      jointState.header.stamp.sec = currentTime->now().seconds();
      jointState.header.stamp.nanosec = currentTime->now().nanoseconds();
      jointState.header.frame_id = "Teensy Motor";
      jointState.name.resize(1);
      jointState.name[0] = "base_to_rotate_joint";
      jointState.position.resize(1);
      jointState.position[0] = jointPosition_;
      jointPublisher_->publish(jointState);
    }
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;
    rclcpp::TimerBase::SharedPtr jointPublisherTimer_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr jointSubscriber_;

    double jointPosition_  = 0.0;
    double jointVelocity_  = 0.0;
    double jointEffort_    = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}