#include "slam/orbslam3/monocular.hpp"

#include <opencv2/core/core.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;

using namespace std;

using std::placeholders::_1;

MonoORBSLAM3::MonoORBSLAM3(rclcpp::Logger logger, std::string settingsFilePath, std::string vocabFilePath, std::string setupCameraType) : Slam(logger), mpSettingsFilePath(settingsFilePath), mpVocabFilePath(vocabFilePath){
	RCLCPP_INFO(mpLogger, "Creating ORBSLAM3 Object");	
	RCLCPP_INFO(mpLogger, "Config File Path: ", settingsFilePath.c_str());
	RCLCPP_INFO(mpLogger, "Vocab File Path: ", vocabFilePath.c_str());
	ORB_SLAM3::System::eSensor cameraType;
	YAML::Node slamConfig;
	if(setupCameraType == "monocular-only"){
		cameraType = ORB_SLAM3::System::eSensor::MONOCULAR;
	}
	mpORBSlam3 = std::make_unique<ORB_SLAM3::System>(vocabFilePath, settingsFilePath, cameraType, true);
}

void MonoORBSLAM3::TrackMonocular(Frame &frame, Sophus::SE3f &tcw){
	tcw = mpORBSlam3->TrackMonocular(frame.getImage(), frame.getTimestampSec());
}


void MonoORBSLAM3::SetFrameMapPointUpdateCallback(std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> callback){
	mpORBSlam3->SetFrameMapPointUpdateCallback(callback);
}
