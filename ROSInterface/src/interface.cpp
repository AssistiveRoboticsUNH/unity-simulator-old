#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "interface.h"

using namespace std::chrono_literals;


#include <iostream>
#include <vector>


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void publish(const NativeTransform *input) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "unity";
        t.child_frame_id = input->frame_id;
        t.transform.translation.x = input->translation.x;
        t.transform.translation.y = input->translation.y;
        t.transform.translation.z = input->translation.z;
        t.transform.rotation.x = input->rotation.x;
        t.transform.rotation.y = input->rotation.y;
        t.transform.rotation.z = input->rotation.z;
        t.transform.rotation.w = input->rotation.w;

        tf_broadcaster_->sendTransform(t);

        auto message = std_msgs::msg::String();
        message.data = "hi!!";
        publisher_->publish(message);
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

bool rosInitialized = false;

void initROS() {
    if (!rosInitialized) {
        auto name = "ros_node";
        rclcpp::init(1, &name);
        rosInitialized = true;
    }
}

class ROSInterface {
public:

    ROSInterface(){
        initROS();
        node_ = std::make_shared<MinimalPublisher>();
    }

    void PublishTF(NativeTransform *input) {
        node_->publish(input);
    }

    std::shared_ptr<MinimalPublisher> node_;

};

std::intptr_t Init() {
    return (std::intptr_t) new ROSInterface();
}

void Destroy(std::intptr_t handle) {
    auto ptr = (ROSInterface *) handle;
    delete ptr;
}

void PublishTF(std::intptr_t handle, NativeTransform *input) {
    auto ptr = (ROSInterface *) handle;
    ptr->PublishTF(input);
}

