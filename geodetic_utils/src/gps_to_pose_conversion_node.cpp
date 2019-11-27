/*************************************************************************
	> File Name: gps_to_pose_conversion_node.cpp
	> Author: weilang
	> Mail: 1602822815@qq.com 
	> Created Time: Mon 04 Nov 2019 08:30:43 PM PST
 ************************************************************************/

#include<iostream>
using namespace std;

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geodetic_utils/geodetic_conv.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "std_msgs/msg/float64.hpp"
#include "geodetic_utils/msg/initgps.hpp"
#include "nav_msgs/msg/odometry.hpp"

std::shared_ptr<rclcpp::Node> node;

bool g_is_sim;
bool g_publish_pose;

geodetic_converter::GeodeticConverter g_geodetic_converter;
sensor_msgs::msg::Imu g_latest_imu_msg;
std_msgs::msg::Float64 g_latest_altitude_msg;
bool g_got_imu;
bool g_got_altitude;


std::shared_ptr<rclcpp::Publisher< geometry_msgs::msg::PoseWithCovarianceStamped> > g_gps_pose_pub;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TransformStamped> > g_gps_transform_pub;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped> > g_gps_position_pub;
std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry> > g_gps_odomgps_pub;


bool g_trust_gps = false;
double g_covariance_position_x = 0.0001;  //default = 4.0
double g_covariance_position_y = 0.0001;  //default = 4.0
double g_covariance_position_z = 10.0;    //default = 10.0s
double g_covariance_orientation_x = 0.02;
double g_covariance_orientation_y = 0.02;
double g_covariance_orientation_z = 0.11;
std::string g_frame_id = "world";
std::string g_tf_child_frame_id = "gps_receiver";

std::shared_ptr<tf2_ros::TransformBroadcaster> p_tf_broadcaster;
// rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr p_tf_broadcaster;
// std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage> >  p_tf_broadcaster;


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
 	g_latest_imu_msg = *msg;
	g_got_imu = true;
}

void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
	// Only the z value in the PointStamped message is used
	g_latest_altitude_msg = *msg;
	g_got_altitude = true;
}


void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	if (!g_got_imu) {
		// ROS_WARN_STREAM_THROTTLE(1, "No IMU data yet");
		RCLCPP_WARN(node->get_logger(), "No IMU data yet");
		return;
	}

	if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
		// ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
		RCLCPP_WARN(node->get_logger(), "No GPS fix");
		return;
	}

	if (!g_geodetic_converter.isInitialised()) {
		// ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing"    );
		RCLCPP_WARN(node->get_logger(), "No GPS reference point set, not publishing");
		return;
	}

	double x, y, z;
	g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

  	// (NWU -> ENU) for simulation
	if (g_is_sim) {                                                           
		double aux = x;
		x = y;
		y = -aux;
		//z = z;
	}

	// Fill up pose message
	geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg(
		new geometry_msgs::msg::PoseWithCovarianceStamped);
	pose_msg->header = msg->header;
	pose_msg->header.frame_id = g_frame_id;
	pose_msg->pose.pose.position.x = x;
	pose_msg->pose.pose.position.y = y;
	pose_msg->pose.pose.position.z = z;
	pose_msg->pose.pose.orientation = g_latest_imu_msg.orientation;

	// Fill up position message
	geometry_msgs::msg::PointStamped::SharedPtr position_msg(
		new geometry_msgs::msg::PointStamped);
	position_msg->header = pose_msg->header;
	position_msg->header.frame_id = g_frame_id;
	position_msg->point = pose_msg->pose.pose.position;
	position_msg->point.x = -position_msg->point.x;
	position_msg->point.y = -position_msg->point.y;

	// If external altitude messages received, include in pose and position me    ssages
	if (g_got_altitude) {
		pose_msg->pose.pose.position.z = g_latest_altitude_msg.data;
		position_msg->point.z = g_latest_altitude_msg.data;
	}

	// std::array<double, 36> 
	// pose_msg->pose.set__covariance(); //weil's comment

	// Set default covariances
	pose_msg->pose.covariance[6 * 0 + 0] = g_covariance_position_x;
	pose_msg->pose.covariance[6 * 1 + 1] = g_covariance_position_y;
	pose_msg->pose.covariance[6 * 2 + 2] = g_covariance_position_z;
	pose_msg->pose.covariance[6 * 3 + 3] = g_covariance_orientation_x;
  	pose_msg->pose.covariance[6 * 4 + 4] = g_covariance_orientation_y;
	pose_msg->pose.covariance[6 * 5 + 5] = g_covariance_orientation_z;

	// Take covariances from GPS
	if (g_trust_gps) {
		if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN
			|| msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
			// Fill in completely
			for (int i = 0; i <= 2; i++) {
				for (int j = 0; j <= 2; j++) {
					pose_msg->pose.covariance[6 * i + j] = msg->position_covariance[3     * i + j];
				}
			}
		} else if (msg->position_covariance_type
			== sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
			// Only fill in diagonal
			for (int i = 0; i <= 2; i++) {
				pose_msg->pose.covariance[6 * i + i] = msg->position_covariance[3 *     i + i];                                                                     
			}
		}
	}

	if (g_publish_pose) {
		g_gps_pose_pub->publish(pose_msg);
	}
	g_gps_position_pub->publish(position_msg);

	// Fill up transform message
	geometry_msgs::msg::TransformStamped::SharedPtr transform_msg(
		new geometry_msgs::msg::TransformStamped);
	transform_msg->header = msg->header;
	transform_msg->header.frame_id = g_frame_id;
	transform_msg->transform.translation.x = x;
	transform_msg->transform.translation.y = y;
  	transform_msg->transform.translation.z = z;
	transform_msg->transform.rotation = g_latest_imu_msg.orientation;

	if (g_got_altitude) {
		transform_msg->transform.translation.z = g_latest_altitude_msg.data;
	}

	g_gps_transform_pub->publish(transform_msg);

	auto gps_odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
	gps_odom_msg->header.stamp = msg->header.stamp;
	gps_odom_msg->header.frame_id = "gps->odom";
	gps_odom_msg->child_frame_id = "base_link";
	gps_odom_msg->pose.pose.position = position_msg->point;
	gps_odom_msg->pose.pose.orientation = pose_msg->pose.pose.orientation;
	gps_odom_msg->pose.covariance = {1.0e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	                                 1.0e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									 0.001};
	gps_odom_msg->twist.twist.linear.x = 0.0;
	gps_odom_msg->twist.twist.linear.y = 0.0;
	gps_odom_msg->twist.twist.linear.z = 0.0;
	gps_odom_msg->twist.twist.angular.x = 0.0;
	gps_odom_msg->twist.twist.angular.y = 0.0;
	gps_odom_msg->twist.twist.angular.z = 0.0;
	gps_odom_msg->twist.covariance = {1.0e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									  1.0e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									  1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									  1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									  1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									  0.001};

	g_gps_odomgps_pub->publish(gps_odom_msg);

	// // Fill up TF broadcaster
	tf2::Transform transform;
	transform.setOrigin(tf2::Vector3(x, y, z));
	transform.setRotation(tf2::Quaternion(g_latest_imu_msg.orientation.x,
											  g_latest_imu_msg.orientation.y,                            
											  g_latest_imu_msg.orientation.z,                            
											  g_latest_imu_msg.orientation.w));
	auto transforms_geo = tf2::Stamped<geometry_msgs::msg::TransformStamped>();
	// auto transforms_geo = tf2::Stamped<tf2::Transform>();
	// auto transforms_geo = tf2::Stamped<geometry_msgs::msg::TransformStamped>(transform,
	// 																		 tf2::get_now(),
	// 					g_gps_odomgps_pub													 g_frame_id);
	p_tf_broadcaster->sendTransform(transforms_geo);
	// p_tf_broadcaster->sendTransform(tf2_ros::StampedTransform(transform,
	// 														  ros::Time::now(), 
	// 														  g_frame_id,
	// 														  g_tf_child_frame_id));                                         
}

int g_count = 0;

void gps_init_callback(const geodetic_utils::msg::Initgps::SharedPtr msg)
{
	std::cout << "############" << std::endl;
	std::cout << msg->latitude << std::endl;
	std::cout << msg->longitude << std::endl;
	std::cout << msg->altitude << std::endl;
	if (g_count == 0){
		node->declare_parameter("/gps_ref_latitude", msg->altitude);
		node->declare_parameter("/gps_ref_longitude", msg->longitude);
		node->declare_parameter("/gps_ref_altitude", msg->latitude);
		g_count += 10;
	}
}


int main(int argc, char **argv)
{
	std::cout << "\n==========BEGIN===========" << std::endl << std::endl;

	rclcpp::init(argc, argv);
	node = rclcpp::Node::make_shared("gps_to_pose_conversion_node");
	// auto set_node = rclcpp::Node::make_shared("set_gps_reference");

	g_got_imu = false;
	g_got_altitude = false;
	p_tf_broadcaster =
		std::make_shared<tf2_ros::TransformBroadcaster>(node);

	// Use different coordinate transform if using simulator

	if (!node->get_parameter("is_sim", g_is_sim)) {
		RCLCPP_WARN(node->get_logger(), "Could not fetch 'sim' param, defaulting to 'false'");
		g_is_sim = false;
	}

	rclcpp::WallRate loop_rate(2); //0.5s

	
	node->declare_parameter("/gps_ref_latitude_fake", 0.000001);
	node->declare_parameter("/gps_ref_longitude_fake", 0.000002);
	node->declare_parameter("/gps_ref_altitude_fake", 1.34);
	double latitude, longitude, altitude;
	auto gps_int_sub =
		node->create_subscription<geodetic_utils::msg::Initgps>("gps_init", gps_init_callback);
	do {

		RCLCPP_INFO(node->get_logger(), "Waiting for GPS reference parameters...");
		if (node->get_parameter("/gps_ref_latitude_fake", latitude) &&
			node->get_parameter("/gps_ref_longitude_fake", longitude) &&
			node->get_parameter("/gps_ref_altitude_fake", altitude))
		{
			g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
		} 
		else {
			RCLCPP_WARN(node->get_logger(), 
				"GPS reference not ready yet, use set_gps_reference_node to set it");

			loop_rate.sleep(); // sleep for half a second
		}
	} while (!g_geodetic_converter.isInitialised());

	// set initial GPS again
	node->get_parameter("/gps_ref_latitude", latitude);
	node->get_parameter("/gps_ref_longitude", longitude);
	node->get_parameter("/gps_ref_altitude", altitude);
	g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
                                           
	double initial_latitude, initial_longitude, initial_altitude;
	g_geodetic_converter.getReference(&initial_latitude, &initial_longitude,
									&initial_altitude);


	RCLCPP_INFO(node->get_logger(), "GPS reference initialized correctly %f, %f, %f", initial_latitude,
	            initial_longitude, initial_altitude);

	// Initialize publishers
	g_gps_pose_pub =
		node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gps_pose", 1); // options=1
	g_gps_transform_pub =
		node->create_publisher<geometry_msgs::msg::TransformStamped>("gps_transform", 1);
	g_gps_position_pub =
		node->create_publisher<geometry_msgs::msg::PointStamped>("gps_position", 1);
	g_gps_odomgps_pub =
		node->create_publisher<nav_msgs::msg::Odometry>("gps_odom", 1);

	// Subscribe to IMU and GPS fixes, and convert in GPS callback
// 	auto imu_sub =
// 		node->create_subscription<sensor_msgs::msg::Imu>("imu", imu_callback, rmw_qos_profile_sensor_data);
	auto imu_sub =
		node->create_subscription<sensor_msgs::msg::Imu>(
			"imu", rclcpp::SensorDataQoS(), imu_callback /*, rmw_qos_profile_sensor_data */);
// 	auto gps_sub =
// 		node->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_data", gps_callback, rmw_qos_profile_sensor_data);
	auto gps_sub =
		node->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_data", rclcpp::SensorDataQoS(), gps_callback);
	auto altitude_sub =
		node->create_subscription<std_msgs::msg::Float64>("external_altitude", 1, altitude_callback);

	rclcpp::spin(node);


	return 0;
}
