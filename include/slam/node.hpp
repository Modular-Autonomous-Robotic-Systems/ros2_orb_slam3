#ifndef SLAM_NODE_HPP
#define SLAM_NODE_HPP

#include "slam/slam.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "custom_interfaces/msg/point_cloud3.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;
using MapMsg = custom_interfaces::msg::PointCloud3;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

bool readYAMLFile(std::string &yamlPath, YAML::Node &output);

class SlamNode : public rclcpp_lifecycle::LifecycleNode{
	public:
		SlamNode(std::string nodeName);
		Sophus::SE3f mpTwc;
	protected:
		void Update();
		std::unique_ptr<tf2_ros::TransformBroadcaster> mpTfBroadcaster;
		std::string mpSettingsFilePath = "";
	private:

		// Publication Callbacks
		void PublishPositionAsTransform(Sophus::SE3f &Tcw);
		void PublishState(int trackingState);
};

#endif
