#include <rclcpp/rclcpp.hpp>
#include "ut_start_2/bag_recorder.hpp"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ut_start_2::BagRecorder>());
    rclcpp::shutdown();
    return 0;
}