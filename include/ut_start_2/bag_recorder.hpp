#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <audio_common_msgs/msg/audio_stamped.hpp>
#include <rosbag2_cpp/writer.hpp>


namespace ut_start_2 {

class BagRecorder : public rclcpp::Node {
  public:
    BagRecorder();  

  private:
    void telemetry_callback(const rclcpp::SerializedMessage msg);
    void audio_callback(const rclcpp::SerializedMessage msg);

    rclcpp::Time time_stamp_;

    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr telemetry_sub_; 
    rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr audio_sub_; 
    
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};


};  // namespace ut_start_2