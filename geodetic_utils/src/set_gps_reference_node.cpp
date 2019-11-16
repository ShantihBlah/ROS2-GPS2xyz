/*************************************************************************
	> File Name: set_gps_reference_node.cpp
	> Author: weilang
	> Mail: 1602822815@qq.com 
	> Created Time: Fri 25 Oct 2019 12:17:00 AM PDT
 ************************************************************************/

#include<iostream>
// using namespace std;

#include <chrono>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <std_srvs/srv/empty.hpp>
#include "std_msgs/msg/string.hpp"
#include "geodetic_utils/msg/initgps.hpp"

// #include "geodetic_utils/geodetic_conv.hpp"

using namespace std::chrono_literals;
using  std::string;


double g_lat_ref;
double g_lon_ref;
double g_alt_ref;
int g_count = 1;
bool gps_ref_is_init;
int g_its;
std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp::Publisher<geodetic_utils::msg::Initgps> > set_gps_init_pub;

enum EMode {
	MODE_AVERAGE = 0,
	MODE_WAIT
};
// average over, or wait for, n GPS fixes
EMode g_mode;


//	std_srvs::srv::Empty_Request &, 
//	std_srvs::srv::Empty_Response &
bool reset_callback(
const std::shared_ptr<rmw_request_id_t> request_header,
const std::shared_ptr<std_srvs::srv::Empty_Request> request,
const std::shared_ptr<std_srvs::srv::Empty_Response> response
)
{
	std::cout << "START--->ResetCallback" << std::endl;
	(void)request_header;
	g_count = 1;
	g_lat_ref = 0.0;
	g_lon_ref = 0.0;
	g_alt_ref = 0.0;
	gps_ref_is_init = node->declare_parameter("/gps_ref_is_init", false);

	return true;
}


void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	// RCLCPP_INFO(node->get_logger(), "start->GpsCallback");
	node->get_parameter("gps_ref_is_init", gps_ref_is_init);

	if (!gps_ref_is_init){


		if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
		//   ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
		//   RCLCPP_WARN(node->get_logger(), "No GPS fix");
		  return;
		}

		g_lat_ref += msg->latitude;
		g_lon_ref += msg->longitude;
		g_alt_ref += msg->altitude;

		// ROS_INFO("Current measurement: %3.8f, %3.8f, %4.2f", msg->latitude, msg->longitude, msg->altitude);
		RCLCPP_INFO(node->get_logger(), "Current measurement: %3.8f, %3.8f, %4.2f", 
		                                 msg->latitude, msg->longitude, msg->altitude);

		// std::cout << g_count << std::endl;
		// std::cout << g_its << std::endl;
		if (g_count == g_its)
		{
			// RCLCPP_WARN(node->get_logger(), "in loop");
			if (g_mode == MODE_AVERAGE) {
				// RCLCPP_WARN(node->get_logger(), "g_mode == MODE_AVERAGE");
				g_lat_ref /= g_its;
				g_lon_ref /= g_its;
				g_alt_ref /= g_its;
			} else {
				// RCLCPP_WARN(node->get_logger(), "g_mode != MODE_AVERAGE");
				g_lat_ref = msg->latitude;
				g_lon_ref = msg->longitude;
				g_alt_ref = msg->altitude;
			}

			// RCLCPP_WARN(node->get_logger(), "BEFORE-PARAM");

			g_lat_ref = node->declare_parameter("/gps_ref_latitude", g_lat_ref);
			g_lon_ref = node->declare_parameter("/gps_ref_longitude", g_lon_ref);
			g_alt_ref = node->declare_parameter("/gps_ref_altitude", g_alt_ref);
			gps_ref_is_init = node->declare_parameter("/gps_ref_is_init", true);

			RCLCPP_INFO(node->get_logger(), "Final reference position: %3.8f, %3.8f, %4.8f", 
			                                g_lat_ref, g_lon_ref, g_alt_ref);
			RCLCPP_INFO(node->get_logger(), "Final reference position: %d, %d, %d",
						node->get_parameter("/gps_ref_latitude", g_lat_ref),
						node->get_parameter("/gps_ref_longitude", g_lat_ref),
						node->get_parameter("/gps_ref_altitude", g_lat_ref));
			return;
		} 
		else {
			//   ROS_INFO("    Still waiting for %d measurements", g_its - g_count);
			RCLCPP_INFO(node->get_logger(), "Still waiting for %d measurements", g_its - g_count);
		}

	g_count++;
	}
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
	RCLCPP_INFO(node->get_logger(), "IMU--->start");
}

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
	RCLCPP_INFO(node->get_logger(), "STRING--->start");
	RCLCPP_INFO(node->get_logger(), "I head: '%s'", msg->data.c_str());
}

int main(int argc, char **argv) 
{

	std::cout << "\n==========BEGIN===========" << std::endl << std::endl;
	
	rclcpp::init(argc, argv);
	node = rclcpp::Node::make_shared("set_gps_reference");
	
	g_lat_ref = 0.0;
	g_lon_ref = 0.0;
	g_alt_ref = 0.0;

	// ros::V_string args;
	// ros::removeROSArgs(argc, argv, args);
	

	g_its = 2;  // default number of fixes
	g_mode = MODE_AVERAGE;  // average by default

	// if (args.size() >= 2) {
	//     const int parsed = atoi(args[1].c_str());
	//     if (parsed > 0)
	//         g_its = parsed;
	// }
	
	// if (args.size() >= 3) {
	// 	if (args[2] == "wait")
	// 		g_mode = MODE_WAIT;
	// }
	
	
	RCLCPP_INFO(node->get_logger(), "Usage: set_gps_reference [n_fixes] [average|wait], defaults: n_fixes=50, average");
	
	RCLCPP_INFO(node->get_logger(),
				"Taking %d measurements and %s\n",g_its,
				(g_mode == MODE_AVERAGE) ? "averaging to get the reference" : "taking the last as reference");

	// std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> fnc = std::bind(
	// 	&imu_callback, this, std::placeholders::_1>);

	// rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub=
	// 		node->create_subscription<sensor_msgs::msg::Imu>("imu", imu_callback);

	// auto subscription = 
	// 	node->create_subscription<std_msgs::msg::String>("weiL_topic", topic_callback);

	auto gps_sub =
		node->create_subscription<sensor_msgs::msg::NavSatFix>("gps_data", gps_callback, rmw_qos_profile_sensor_data);

	// std::cout << "passby gps_sub" << std::endl;

	auto server_srv =
    	node->create_service<std_srvs::srv::Empty>("reset_gps_reference", reset_callback);

	// std::cout << "passby server_srv" << std::endl;

	set_gps_init_pub =
		node->create_publisher<geodetic_utils::msg::Initgps>("gps_init"); // options=1

	// rclcpp::spin(node);

  	// rclcpp::shutdown();
	rclcpp::WallRate loop_rate(50ms);

	while (rclcpp::ok()) {
		auto gps_init_msg = std::make_shared<geodetic_utils::msg::Initgps>();
		gps_init_msg->latitude = g_lat_ref;
		gps_init_msg->longitude = g_lon_ref;
		gps_init_msg->altitude = g_alt_ref;
		set_gps_init_pub->publish(gps_init_msg);
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	return 0;
}
