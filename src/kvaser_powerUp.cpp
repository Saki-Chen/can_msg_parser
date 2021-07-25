#include <iostream>
#include <iomanip>
#include <array>
#include <string>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "cmdline.h"
#include <ros_esdcan_bridge/can_encode_decode_inl.h>

const std::string device_list[]{"dual_lidar", "middle_lidar", "gps", "camera", "ultrasonic", "radar", "sick", "avm", "inverter"};

int main(int argc, char **argv)
{
   cmdline::parser p;
   p.set_program_name("PowerUp");
   for (const auto &device : device_list)
   {
      p.add(device, device[0], "Power Up " + device);
   }
   p.add<int>("can_device", 0, "can device id, available : {0, 1, 2, 3}", false, 2, cmdline::range(0, 3));
   p.add<int>("frame_id", 0, "can frame identifier", false, 0xc);
   p.add<int>("start_bit", 0, "can signal start bit", false, 0);
   p.add<int>("len", 0, "can signal bit length", false, 16);
   p.add("help", 'h', "print this message");

   if (argc < 2)
   {
      std::string answer{""};

      while (answer[0] != 'y' && answer[0] != 'Y' && answer[0] != 'n' && answer[0] != 'N')
      {
         std::cout << "Do you mean Power OFF all device?(yes/no)" << std::endl;
         std::cin >> answer;
      }
      if (answer[0] == 'n' || answer[0] == 'N')
      {
         std::cerr << p.usage();
         return EXIT_FAILURE;
      }
   }

   p.parse_check(argc, argv);

   uint16_t cmd = 0;

   for (const auto& device : device_list)
   {
      cmd <<= 1u;
      std::cout << std::setw(14) << std::left << device;
      if (p.exist(device))
      {
         cmd |= 1u;
         std::cout << "Power ON\n";
      }
      else
      {
         std::cout << "Power OFF\n";
      }
   }

   std::cout << std::setw(14) << std::left << "final cmd:"
             << "0x" << std::hex << cmd << std::endl;

   ros::init(argc, argv, "kvaser_powerUp");
   ros::NodeHandle nh;
   
   const auto pub = nh.advertise<can_msgs::Frame>("/kvaser_channel_2/can_rx", 1);

   can_msgs::Frame can_data; 
   can_data.dlc = 8;
   can_data.is_error = can_data.is_extended =can_data.is_rtr = false;
   can_data.id = p.get<int>("frame_id");
   encode(can_data.data.data(), cmd, p.get<int>("start_bit"), p.get<int>("len"), false, false, 1, 1);
   can_data.header.stamp = ros::Time::now();
   
   ros::Rate rate(100);

   while(nh.ok())
   {  
      rate.sleep();
      if(pub.getNumSubscribers() > 0) 
      {
         pub.publish(can_data);
         rate.sleep();
         break;
      }
   }

   return EXIT_SUCCESS;
}