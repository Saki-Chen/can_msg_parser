#include <map>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include <apa_msgs/IntStamped.h>
#include <apa_msgs/FloatStamped.h>
#include <apa_msgs/WheelEncoderStamped.h>
#include <apa_msgs/SteeringAngleStamped.h>
#include <apa_msgs/ChasisSpeedStamped.h>
#include <ros_esdcan_bridge/can_coder.h>
#include "common_names.h"

using namespace esdcan;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chasis_msg_parser");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Time::waitForValid();

    std::string json_file, can_channel;
    private_nh.param<std::string>("meta_can_file_path", json_file, "./src/can_msg_parser/config/can_dev_2.json");
    private_nh.param<std::string>("can_channel", can_channel, "/esdcan_channel_2");
    std::map<int32_t, CanFrame> all_frame;
    if (!CanFrame::loadAllCanFrame(json_file, all_frame))
        return EXIT_FAILURE;

    const auto &wheeling = all_frame.at(0x305);
    const auto &chasis_speed = all_frame.at(0x302);
    const auto &EPS = all_frame.at(0x18b);
    const auto &frame_0x191 = all_frame.at(0x191);
    const auto &frame_0x303 = all_frame.at(0x303);

    apa_msgs::SteeringAngleStamped steering_status;
    apa_msgs::WheelEncoderStamped wheel_status;
    apa_msgs::ChasisSpeedStamped speed_status;
    apa_msgs::IntStamped shift_poistion;
    apa_msgs::FloatStamped brake_pedal_position;
    apa_msgs::IntStamped time_enable;

    steering_status.header.frame_id = wheel_status.header.frame_id = speed_status.header.frame_id = shift_poistion.header.frame_id = brake_pedal_position.header.frame_id = time_enable.header.frame_id = chasis_frame_id;

    ros::Publisher pub_wheeling, pub_steering, pub_speed, pub_shift, pub_brake, pub_enable;

    pub_wheeling = nh.advertise<apa_msgs::WheelEncoderStamped>(topic_wheeling_count, 3);
    pub_steering = nh.advertise<apa_msgs::SteeringAngleStamped>(topic_steering_angle, 3);
    pub_speed = nh.advertise<apa_msgs::ChasisSpeedStamped>(topic_chasis_speed, 3);
    pub_shift = nh.advertise<apa_msgs::IntStamped>(topic_shift_position, 3);
    pub_brake = nh.advertise<apa_msgs::FloatStamped>(topic_brake_pedal_position, 3);
    pub_enable = nh.advertise<apa_msgs::IntStamped>(topic_time_enable, 3);

    const auto &sig_FL = wheeling.signal_list.at("FL");
    const auto &sig_RL = wheeling.signal_list.at("RL");
    const auto &sig_FR = wheeling.signal_list.at("FR");
    const auto &sig_RR = wheeling.signal_list.at("RR");
    const auto &sig_motor_speed = chasis_speed.signal_list.at("motor_speed");
    const auto &sig_vehicle_speed = chasis_speed.signal_list.at("vehicle_speed");
    const auto &sig_steering_angle = EPS.signal_list.at("steering_angle");
    const auto &sig_shift = frame_0x303.signal_list.at("shift_position");
    const auto &sig_brake = frame_0x303.signal_list.at("brake_pedal_position");
    const auto &sig_enable = frame_0x191.signal_list.at("time_enable");  

    FrameDecoder decoder(true);
    FrameDecoder decoder2(false);

    std::function<void(const can_msgs::FrameConstPtr &msg)> callback = [&](const can_msgs::FrameConstPtr &msg) {
        // std::cout << *msg;
        if (msg->id == wheeling.id)
        {
            double fl, fr, rl, rr;
            decoder.decode(msg->data.data(), sig_FL, fl);
            decoder.decode(msg->data.data(), sig_FR, fr);
            decoder.decode(msg->data.data(), sig_RL, rl);
            decoder.decode(msg->data.data(), sig_RR, rr);
            wheel_status.FL = fl;
            wheel_status.FR = fr;
            wheel_status.RL = rl;
            wheel_status.RR = rr;
            wheel_status.header.stamp = msg->header.stamp;
            pub_wheeling.publish(wheel_status);
        }
        else if (msg->id == chasis_speed.id)
        {
            double motor_speed, vehicle_speed;
            decoder.decode(msg->data.data(), sig_motor_speed, motor_speed);
            decoder.decode(msg->data.data(), sig_vehicle_speed, vehicle_speed);
            speed_status.MotorSpeed = motor_speed;
            speed_status.VehicleSpeed = vehicle_speed;
            speed_status.header.stamp = msg->header.stamp;
            pub_speed.publish(speed_status);
        }
        else if (msg->id == EPS.id)
        {
            double angle;
            decoder.decode(msg->data.data(), sig_steering_angle, angle);
            steering_status.angle = angle;
            steering_status.header.stamp = msg->header.stamp;
            pub_steering.publish(steering_status);
        }
        else if (msg->id == frame_0x303.id)
        {
            double value;
            decoder.decode(msg->data.data(), sig_shift, value);
            shift_poistion.value = value;
            decoder.decode(msg->data.data(), sig_brake, value);
            brake_pedal_position.value = value;
            shift_poistion.header.stamp = brake_pedal_position.header.stamp = msg->header.stamp;
            pub_shift.publish(shift_poistion);
            pub_brake.publish(brake_pedal_position);
        }
        else if (msg->id == frame_0x191.id)
        {
            double value;
            decoder2.decode(msg->data.data(), sig_enable, value);
            time_enable.value = value;
            time_enable.header.stamp = msg->header.stamp;
            pub_enable.publish(time_enable);
        }
    };

    const auto sub = nh.subscribe<can_msgs::Frame>(can_channel + "/can_tx", 10, callback);

    ros::spin();
  
}