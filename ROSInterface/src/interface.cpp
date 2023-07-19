#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "interface.h"

using namespace std::chrono_literals;


#include <iostream>
#include <vector>


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher") {
        eating_publisher_ = this->create_publisher<std_msgs::msg::Int32>("person_eating", 10);
        taking_medicine_publisher_ = this->create_publisher<std_msgs::msg::Int32>("person_taking_medicine", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void publishTF(const NativeTransform *input) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.child_frame_id = input->frame_id;
        t.header.frame_id = input->child_frame_id;
        t.transform.translation.x = input->translation.x;
        t.transform.translation.y = input->translation.y;
        t.transform.translation.z = input->translation.z;
        t.transform.rotation.x = input->rotation.x;
        t.transform.rotation.y = input->rotation.y;
        t.transform.rotation.z = input->rotation.z;
        t.transform.rotation.w = input->rotation.w;

        tf_broadcaster_->sendTransform(t);
    }

    void PublishActivity(const NativeActivity *input) {

        auto message = std_msgs::msg::Int32();
        message.data = input->person_eating;
        eating_publisher_->publish(message);
        message.data = input->person_taking_medicine;
        taking_medicine_publisher_->publish(message);
    }

    void PublishOdom(const NativeOdom *input) {
        auto message = nav_msgs::msg::Odometry();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = input->pose.frame_id;
        message.child_frame_id = input->pose.child_frame_id;

        message.pose.pose.position.x = input->pose.translation.x;
        message.pose.pose.position.y = input->pose.translation.y;
        message.pose.pose.position.z = input->pose.translation.z;
        message.pose.pose.orientation.x = input->pose.rotation.x;
        message.pose.pose.orientation.y = input->pose.rotation.y;
        message.pose.pose.orientation.z = input->pose.rotation.z;
        message.pose.pose.orientation.w = input->pose.rotation.w;
        std::memcpy(&message.pose.covariance, input->pose_covariance, sizeof(input->pose_covariance));

        message.twist.twist.linear.x = input->linear_velocity.x;
        message.twist.twist.linear.y = input->linear_velocity.y;
        message.twist.twist.linear.z = input->linear_velocity.z;

        message.twist.twist.angular.x = input->angular_velocity.x;
        message.twist.twist.angular.y = input->angular_velocity.y;
        message.twist.twist.angular.z = input->angular_velocity.z;

        std::memcpy(&message.twist.covariance, input->twist_covariance, sizeof(input->twist_covariance));
        odom_publisher_->publish(message);
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr eating_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr taking_medicine_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
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

    ROSInterface() {
        initROS();
        node_ = std::make_shared<MinimalPublisher>();
    }

    void PublishTF(NativeTransform *input) {
        node_->publishTF(input);
    }

    void PublishActivity(NativeActivity *input) {
        node_->PublishActivity(input);
    }

    void PublishOdom(NativeOdom *input) {
        node_->PublishOdom(input);
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

void PublishActivity(std::intptr_t handle, NativeActivity *input) {
    auto ptr = (ROSInterface *) handle;
    ptr->PublishActivity(input);
}

void PublishOdom(std::intptr_t handle, NativeOdom *input) {
    auto ptr = (ROSInterface *) handle;
    ptr->PublishOdom(input);
}

//void ReceiveCmdVel(std::intptr_t handle, NativeVector3 *output) {
//    auto ptr = (ROSInterface *) handle;
////    ptr->ReceiveOdom(output);
//}
