#include <Eigen/Dense>

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <apa_msgs/VehicleOdomEst2D.h>
#include <apa_msgs/SlotInfoStamped.h>

#include <ros_esdcan_bridge/can_coder.h>
#include "bbox.h"
#include "common_names.h"

using namespace esdcan;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parking_info_parser");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string json_file, can_channel;
    private_nh.param<std::string>("meta_can_file_path", json_file, "./src/can_msg_parser/config/can_dev_1.json");
    private_nh.param<std::string>("can_channel", can_channel, "/esdcan_channel_1");

    std::map<int32_t, CanFrame> all_frame;
    if (!CanFrame::loadAllCanFrame(json_file, all_frame))
        return EXIT_FAILURE;   

    const auto& frame_0x191 = all_frame.at(0x191);
    const auto& frame_0x192 = all_frame.at(0x192);
    const auto& frame_0x193 = all_frame.at(0x193);
    const auto& frame_0x194 = all_frame.at(0x194);
    const auto& frame_0x195 = all_frame.at(0x195);
    const auto& frame_0x196 = all_frame.at(0x196);
    const auto& frame_0x197 = all_frame.at(0x197);

    can_msgs::Frame can_msg;
    can_msg.header.frame_id = local_map_frame_id;
    can_msg.dlc = 8;
    ros::Publisher pub_can_msg = nh.advertise<can_msgs::Frame>(can_channel + "/can_rx", 10);

    auto encode = [](double val, const CanFrame& frame ,const char* name, can_msgs::Frame& msg)
    {
        static FrameEncoder encoder(false);
        encoder.encode(val, frame.signal_list.at(name), msg.data.begin());
    };

    auto wheel_odom_handler = [&](const apa_msgs::VehicleOdomEst2DConstPtr& msg)
    {
        can_msg.header.stamp = msg->header.stamp;
        can_msg.id = frame_0x193.id;
        encode(msg->distance, frame_0x193, "distance", can_msg);
        pub_can_msg.publish(can_msg);        
    };

    auto base_twist_handler = [&](const geometry_msgs::TwistStampedConstPtr& msg)
    {
        can_msg.header.stamp = msg->header.stamp;
        can_msg.id = frame_0x191.id;
        encode(msg->twist.linear.x, frame_0x191, "vx", can_msg);
        encode(msg->twist.linear.y, frame_0x191, "vy", can_msg);
        encode(msg->twist.angular.z * 180.0 / M_PI, frame_0x191, "yaw_rate", can_msg);
        pub_can_msg.publish(can_msg);
    };

    auto base_odom_handler = [&](const nav_msgs::OdometryConstPtr& msg)
    {
        can_msg.header.stamp = msg->header.stamp;
       
        can_msg.id = frame_0x192.id;
        encode(msg->pose.pose.position.x, frame_0x192, "x", can_msg);
        encode(msg->pose.pose.position.y, frame_0x192, "y", can_msg);
        Eigen::AngleAxisd rot(Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));        
        encode(rot.axis().normalized()[2] * BBox::Modulus2PI(rot.angle()) * 180.0 / M_PI, frame_0x192, "yaw", can_msg);
        pub_can_msg.publish(can_msg);        
    };

    auto slot_info_handler = [&](const apa_msgs::SlotInfoStampedConstPtr& msg)
    {
        can_msg.header.stamp = msg->header.stamp;

        BBox ref_box(msg->ref_pose.x, msg->ref_pose.y, msg->ref_pose.theta, msg->ref_extend_x, msg->ref_extend_y);
        BBox obstacle_box(msg->obstacle_pose.x, msg->obstacle_pose.y, msg->obstacle_pose.theta, msg->obstacle_extend_x, msg->obstacle_extend_y);

        can_msg.id = frame_0x194.id;
        encode(ref_box.x, frame_0x194, "ref_x_outside", can_msg);
        encode(ref_box.y, frame_0x194, "ref_y_outside", can_msg);
        encode(ref_box.angle * 180.0 / M_PI, frame_0x194, "ref_yaw", can_msg);
        pub_can_msg.publish(can_msg);

        can_msg.id = frame_0x195.id;
        const auto ref_p_in = ref_box.getYAxisPoint();
        encode(ref_p_in.x, frame_0x195, "ref_x_inside", can_msg);
        encode(ref_p_in.y, frame_0x195, "ref_y_inside", can_msg);
        encode(msg->slot_width, frame_0x195, "slot_width", can_msg);
        pub_can_msg.publish(can_msg);

        can_msg.id = frame_0x196.id;
        encode(obstacle_box.x, frame_0x196, "obstacle_x_outside", can_msg);
        encode(obstacle_box.y, frame_0x196, "obstacle_y_outside", can_msg);
        pub_can_msg.publish(can_msg);

        can_msg.id = frame_0x197.id;
        const auto obstacle_p_in = obstacle_box.getYAxisPoint();
        encode(obstacle_p_in.x, frame_0x197, "obstacle_x_inside", can_msg);
        encode(obstacle_p_in.y, frame_0x197, "obstacle_y_inside", can_msg);
        pub_can_msg.publish(can_msg);
    };

    ros::Subscriber sub_wheel_odom = nh.subscribe<apa_msgs::VehicleOdomEst2D>(topic_extra_odom_info, 8, wheel_odom_handler);
    ros::Subscriber sub_base_twsit = nh.subscribe<geometry_msgs::TwistStamped>(topic_velometer + std::string("/") + base_link_frame_id + "_local", 16, base_twist_handler);
    ros::Subscriber sub_base_odom = nh.subscribe<nav_msgs::Odometry>(topic_odometer + std::string("/") + local_map_frame_id + "/" + base_link_frame_id, 16, base_odom_handler);
    ros::Subscriber sub_slot_info = nh.subscribe<apa_msgs::SlotInfoStamped>(topic_slot_info, 3, slot_info_handler);

    ros::spin();
}