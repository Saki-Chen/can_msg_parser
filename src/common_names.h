#pragma once
#include <string>
#include "frame_id.h"

static constexpr char topic_wheeling_count[]("wheeling_count_fl_fr_rl_rr");
static constexpr char topic_steering_angle[]("steering_angle_deg");
static constexpr char topic_chasis_speed[]("chasis_speed");
static constexpr char topic_brake_pedal_position[]("brake_pedal_position");
static constexpr char topic_shift_position[]("shift_position");
static constexpr char topic_odometer[]("odometer");
static constexpr char topic_velometer[]("velometer");
static constexpr char topic_extra_odom_info[]("extra_odom_info");
static constexpr char topic_slot_info[]("parking_slot_info");
static constexpr char topic_gps_fix[]("gps/fix");
static constexpr char topic_imu_msg[]("imu/data");
static constexpr char topic_time_enable[]("time_enable");