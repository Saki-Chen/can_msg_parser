#include <functional>
#include <mutex>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>

#include <ros_esdcan_bridge/can_coder.h>
#include "common_names.h"

using namespace esdcan;

constexpr double deg2rad = M_PI / 180.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rt3000_msg_parser");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string json_file, can_channel;
    private_nh.param<std::string>("meta_can_file_path", json_file, "./src/can_msg_parser/config/can_dev_1.json");
    private_nh.param<std::string>("can_channel", can_channel, "/esdcan_channel_1");

    std::map<int32_t, CanFrame> all_frames;
    if (!CanFrame::loadAllCanFrame(json_file, all_frames))
        return EXIT_FAILURE;
    
    const auto &gps_status = all_frames.at(0x500);
    const auto &lat_lon = all_frames.at(0x601);
    const auto &alt = all_frames.at(0x602);
    const auto &V_in_level_frame = all_frames.at(0x604);
    const auto &rotation = all_frames.at(0x607);

    const auto &body_acc = all_frames.at(0x605);
    const auto &body_angular = all_frames.at(0x608);

    const auto &sig_pos_mode = gps_status.signal_list.at("PositionMode");
    const auto &sig_lat = lat_lon.signal_list.at("Latitude");
    const auto &sig_lon = lat_lon.signal_list.at("Longitude");
    const auto &sig_alt = alt.signal_list.at("Altitude");
    const auto &sig_heading = rotation.signal_list.at("Heading");
    const auto &sig_pitch = rotation.signal_list.at("Pitch");
    const auto &sig_roll = rotation.signal_list.at("Roll");

    const auto &sig_acc_x = body_acc.signal_list.at("Body_Acc_X");
    const auto &sig_acc_y = body_acc.signal_list.at("Body_Acc_Y");
    const auto &sig_acc_z = body_acc.signal_list.at("Body_Acc_Z");

    const auto &sig_angular_x = body_angular.signal_list.at("Body_Angular_X");
    const auto &sig_angular_y = body_angular.signal_list.at("Body_Angular_Y");
    const auto &sig_angular_z = body_angular.signal_list.at("Body_Angular_Z");

    const auto &sig_v_forward = V_in_level_frame.signal_list.at("Vx");
    const auto &sig_v_right = V_in_level_frame.signal_list.at("Vy");

    FrameDecoder decoder(false);

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>(topic_gps_fix, 32);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(topic_imu_msg, 32);
    ros::Publisher local_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("rt3000_local_velocity", 32);
    
    sensor_msgs::NavSatFix gps_data;
    gps_data.header.frame_id = gps_link_frame_id;

    sensor_msgs::Imu imu_data;
    imu_data.header.frame_id = imu_link_frame_id;
    imu_data.orientation_covariance[0] = -1;
    imu_data.orientation.w = 1;
    imu_data.orientation.x = imu_data.orientation.y = imu_data.orientation.z = 0;

    geometry_msgs::TwistStamped local_velocity;
    local_velocity.header.frame_id = imu_link_frame_id;

    bool alt_init, pose_type_init, acc_update;
    alt_init = pose_type_init = acc_update = false;

    std::function<void(const can_msgs::FrameConstPtr &msg)> callback = [&](const can_msgs::FrameConstPtr &msg) {
        if(msg->id == gps_status.id)
        {
            double pos_code;
            decoder.decode(msg->data.data(), sig_pos_mode, pos_code);
            gps_data.status.status = int(pos_code);
            pose_type_init = true;
        }
        else if (msg->id == lat_lon.id)
        {
            if(!alt_init || !pose_type_init) return;
            double lat, lon;
            decoder.decode(msg->data.data(), sig_lat, lat);
            decoder.decode(msg->data.data(), sig_lon, lon);
            gps_data.header.stamp = msg->header.stamp;
            gps_data.latitude = lat;
            gps_data.longitude = lon;
            gps_pub.publish(gps_data);
        }
        else if (msg->id == alt.id)
        {
            double alt;
            decoder.decode(msg->data.data(), sig_alt, alt);
            gps_data.altitude = alt;
            alt_init = true;
        }
        else if (msg->id == rotation.id)
        {
            double heading, pitch, roll;
            decoder.decode(msg->data.data(), sig_heading, heading);
            decoder.decode(msg->data.data(), sig_pitch, pitch);
            decoder.decode(msg->data.data(), sig_roll, roll);
            auto q = tf::createQuaternionFromRPY(deg2rad * roll, -deg2rad * pitch, deg2rad * (90.0 - heading));
            imu_data.orientation.w = q.w();
            imu_data.orientation.x = q.x();
            imu_data.orientation.y = q.y();
            imu_data.orientation.z = q.z();
            imu_data.orientation_covariance[0] = 0;
        }
        else if (msg->id == body_acc.id)
        {
            double ax, ay, az;
            decoder.decode(msg->data.data(), sig_acc_x, ax);
            decoder.decode(msg->data.data(), sig_acc_y, ay);
            decoder.decode(msg->data.data(), sig_acc_z, az);
            imu_data.linear_acceleration.x = ax;
            imu_data.linear_acceleration.y = -ay;
            imu_data.linear_acceleration.z = -az;
            acc_update = true;
        }
        else if (msg->id == body_angular.id)
        {
            if(!acc_update) return;
            acc_update = false;
            double rx, ry, rz;
            decoder.decode(msg->data.data(), sig_angular_x, rx);
            decoder.decode(msg->data.data(), sig_angular_y, ry);
            decoder.decode(msg->data.data(), sig_angular_z, rz);
            imu_data.header.stamp = msg->header.stamp;
            imu_data.angular_velocity.x = deg2rad * rx;
            imu_data.angular_velocity.y = deg2rad * -ry;
            imu_data.angular_velocity.z = deg2rad * -rz;
            imu_pub.publish(imu_data);
        }
        else if(msg->id == V_in_level_frame.id)
        {
            double vx, vy;
            decoder.decode(msg->data.data(), sig_v_forward, vx);
            decoder.decode(msg->data.data(), sig_v_right, vy);
            local_velocity.header.stamp = msg->header.stamp;
            local_velocity.twist.linear.x = vx;
            local_velocity.twist.linear.y = -vy;
            local_velocity_pub.publish(local_velocity);
        }
        
    };

    const auto sub = nh.subscribe<can_msgs::Frame>(can_channel + "/can_tx", 32, callback);

    ros::spin();
}