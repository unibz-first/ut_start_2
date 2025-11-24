#include "ut_start_2/bag_recorder.hpp"


namespace ut_start_2 {

BagRecorder::BagRecorder() : rclcpp::Node("bag_recorder") {
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("my_bag");

    telemetry_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined",
        qos,
        std::bind(&BagRecorder::telemetry_callback, this, std::placeholders::_1)
    );

    audio_sub_ = create_subscription<audio_common_msgs::msg::AudioStamped>(
        "/audio",
        qos,
        std::bind(&BagRecorder::audio_callback, this, std::placeholders::_1)
    );


}

void BagRecorder::telemetry_callback(const rclcpp::SerializedMessage msg) {
    time_stamp_ = this->now();
    writer_->write(msg, "/fmu/out/sensor_combined", "px4_msgs/msg/SensorCombined", time_stamp_);
}

void BagRecorder::audio_callback(const rclcpp::SerializedMessage msg) {
    time_stamp_ = this->now();
    writer_->write(msg, "/audio", "audio_common_msgs/msg/AudioStamped", time_stamp_);
}

}