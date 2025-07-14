#include "slam/monocular.hpp"

VisualSlamNode::VisualSlamNode() : SlamNode("orbslam3_mono_node"){
	rcl_interfaces::msg::ParameterDescriptor camera_topic_name_desc;
	camera_topic_name_desc.description = "Camera Topic Name";
	camera_topic_name_desc.type = 4;
	this->declare_parameter<std::string>("camera_topic_name", "/camera", camera_topic_name_desc);

	rcl_interfaces::msg::ParameterDescriptor vocab_file_path_desc;
	vocab_file_path_desc.description = "Path to ORBSLAM3 Vocabulary File";
	vocab_file_path_desc.type = 4;
	this->declare_parameter<std::string>("vocab_file_path", "/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin", vocab_file_path_desc);
	
	mpOrbToROSTransform <<  1, 0, 0, 0, 0, -1, 0, 1, 0;
}

CallbackReturn VisualSlamNode::on_configure(const rclcpp_lifecycle::State & previous_state){
	mpSettingsFilePath = this->get_parameter("settings_file_path").as_string();
	RCLCPP_INFO(this->get_logger(), "Settings File Path: %s", mpSettingsFilePath.c_str());
	mpCameraTopicName = this->get_parameter("camera_topic_name").as_string();
	mpVocabFilePath = this->get_parameter("vocab_file_path").as_string();
	RCLCPP_INFO(this->get_logger(), "Camera Topic Name: %s", mpCameraTopicName.c_str());
	RCLCPP_INFO(this->get_logger(), "Vocabulary File Path: %s", mpVocabFilePath.c_str());
	return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_activate(const rclcpp_lifecycle::State & previous_state){
	RCLCPP_INFO(this->get_logger(), "Creating SLAM Object in Mono Slam Node");
#ifdef USE_ORBSLAM3
	mpSlam = std::unique_ptr<Slam>(new MonoORBSLAM3(this->get_logger(), mpSettingsFilePath, mpVocabFilePath, "monocular-only"));
	RCLCPP_INFO(this->get_logger(), "Created SLAM Object for ORBSLAM3 in Mono Slam Node");
	std::function<void(std::vector<ORB_SLAM3::MapPoint*>&, const Sophus::SE3<float>&)> cb = [this](std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
		PublishMapPointsCallback(mapPoints, tcw);
	};
	mpSlam->SetFrameMapPointUpdateCallback(cb);
	mpState = ORB_SLAM3::Tracking::SYSTEM_NOT_READY;
#endif
#ifdef USE_MORBSLAM
	mpSlam = std::unique_ptr<Slam>(new MonoMORBSLAM(this->get_logger()));
	mpState = MORB_SLAM::TrackingState::SYSTEM_NOT_READY;
	RCLCPP_INFO(this->get_logger(), "Created SLAM Object for MORBSLAM in Mono Slam Node");
#endif

	mpTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(tf2_ros::TransformBroadcaster(this)); 

	RCLCPP_INFO(this->get_logger(), "Got Camera topic name: %s", mpCameraTopicName.c_str());
	RCLCPP_INFO(this->get_logger(), "Got Config file path: %s", mpSettingsFilePath.c_str());	
	RCLCPP_INFO(this->get_logger(), "Creating Frame Subscription for topic: %s perform to visual odometry", mpCameraTopicName.c_str());

	mpFrameSubscriber = this->create_subscription<ImageMsg>(
			mpCameraTopicName,
			10,
			std::bind(&VisualSlamNode::GrabImage, this, std::placeholders::_1));
	mpAnnotatedFramePublisher = this->create_publisher<ImageMsg>("~/annotated_frame", 10);
	mpMapPointPublisher = this->create_publisher<MapMsg>("~/map_points", 10);
	// mpMapPublisher = this->create_publisher<MarkerMsg>("~/slam_map");

	RCLCPP_INFO(this->get_logger(), "ORBSLAM3 initialisation complete");
	return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_deactivate(const rclcpp_lifecycle::State& previous_state){
	if (mpMapPointPublisher){
		mpMapPointPublisher.reset();
	}
	if(mpAnnotatedFramePublisher){
		mpAnnotatedFramePublisher.reset();
	}
	if(mpTfBroadcaster){
		mpTfBroadcaster.reset();
	}
	if(mpFrameSubscriber){
		mpFrameSubscriber.reset();
	}
	if(mpSlam){
		mpSlam->Shutdown();
		mpSlam.reset();
	}
	return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_cleanup(const rclcpp_lifecycle::State& previous_state){
	return CallbackReturn::SUCCESS;
}

void VisualSlamNode::Update(){
	SlamNode::Update();
	PublishFrame();
}


VisualSlamNode::~VisualSlamNode()
{
    // Stop all threads
	mpSlam.reset();
}


void VisualSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
	RCLCPP_DEBUG(this->get_logger(), "Received Image Message for Tracking");

    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
	// TODO check what kind of buffering and mutex is needed for performing SLAM tracking
	int sec = msg->header.stamp.sec;
	int nsec = msg->header.stamp.nanosec;
	long timestamp = sec * 1e9 + nsec;
   	mpCurrentFrame = Frame(std::make_shared<cv::Mat>(m_cvImPtr->image), timestamp);
	cv::Mat img = mpCurrentFrame.getImage();
	Sophus::SE3f tcw;
	if(!img.empty()){
		mpSlam->TrackMonocular(mpCurrentFrame, tcw);
		// The output of ORBSLAM3 is the transformation that can convert points
		// from world frame to camera frame. Thus, Tcw * Xw would be equivalent to
		// Xw to Xc. To get the position of the camera with respect to slam origin,
		// the transformation needs to be inverted to obtain camera position in 
		// world reference
		// The following link documents an issue related to this in ORBSLAM2 repo
		// https://github.com/raulmur/ORB_SLAM2/issues/226
		// One may also refer to the following link to check the implementation of
		// of inverse for Sophus::SE3f 
		mpTwc = tcw.inverse();
		// Publishes all data common to all SLAM algorithms
		Update();
	}
    
    // UpdateSLAMState();
    // UpdateMapState();
    //
    // PublishCurrentCamera();
    // PublishMapPoints();
    // PublishKeyFrames();
}

void VisualSlamNode::PublishFrame()
{

	cv::Mat drawnFrame = mpSlam->GetCurrentFrame();
    // cv::Mat im = DrawFrame();

	sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
	cv_bridge::CvImage(
			std_msgs::msg::Header(),
			sensor_msgs::image_encodings::BGR8,
			drawnFrame.clone()
			).toImageMsg(*img_msg_ptr);
	img_msg_ptr->header.stamp = this->now();
	// img_msg_ptr->height = frame.height;
	// img_msg_ptr->width = frame.width;
	// img_msg_ptr->is_bigendian = false;
	// img_msg_ptr->step = frame.width * frame.frame.elemSize();

    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr->image = drawnFrame.clone();
    //
    // cv_ptr->header.stamp = this->now();
    // cv_ptr->encoding = "bgr8";

    mpAnnotatedFramePublisher->publish(std::move(img_msg_ptr));

}

#ifdef USE_ORBSLAM3
void VisualSlamNode::PublishMapPointsCallback(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw){
	if(mapPoints.size() == 0){
		RCLCPP_ERROR(this->get_logger(), "No Map Points to add to octomap");
		return;
	}
	RCLCPP_DEBUG(this->get_logger(), "Got %d map points for publication", mapPoints.size());
	MapMsg::UniquePtr msg = std::make_unique<MapMsg>();
	MapPointsToPointCloud(mapPoints, tcw, msg->cloud);
	Sophus::SE3<float> twc = tcw.inverse();
	SophusToGeometryMsgTransform(twc, msg->pose);
	mpMapPointPublisher->publish(std::move(msg));
}

void VisualSlamNode::MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw, sensor_msgs::msg::PointCloud2 &cloud){
	// sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z
	long current_frame_time_ = mpCurrentFrame.getTimestampNSec();
	Eigen::Matrix3d cam_base_R_ = mpOrbToROSTransform * Eigen::Matrix3d::Identity();
	Eigen::Vector3d cam_base_T_ = mpOrbToROSTransform * Eigen::Vector3d::Zero();

    cloud.header.stamp.sec = static_cast<int32_t>(current_frame_time_ / 1e9);
	cloud.header.stamp.nanosec = static_cast<int32_t>(current_frame_time_ % static_cast<long long>(1e9));
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = mapPoints.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
	int relevant_map_points = 0;
    for (unsigned int i=0; i<cloud.width; i++) {
        if(!mapPoints[i])
            continue;

        if (mapPoints.at(i)->nObs >= mpNumMinObsPerPoint) {
            Eigen::Vector3f map_pt_f = mapPoints.at(i)->GetWorldPos();

            Eigen::Vector3d map_pt = map_pt_f.cast<double>();

            map_pt = cam_base_R_ * map_pt;
            map_pt += cam_base_T_;

            data_array[0] = map_pt[0];
            data_array[1] = map_pt[1];
            data_array[2] = map_pt[2];
			if(!(map_pt[0] == 0 && map_pt[1] == 0 && map_pt[2] == 0)){
            	memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
				relevant_map_points++;
			}
        }
    }
	RCLCPP_DEBUG(this->get_logger(), "Pushing %d map points for publication", relevant_map_points);
    // return cloud;
}

void VisualSlamNode::SophusToGeometryMsgTransform(const Sophus::SE3<float>& se3, geometry_msgs::msg::Transform &pose) {
    // Extract translation
    pose.translation.x = se3.translation().x();
    pose.translation.y = se3.translation().y();
    pose.translation.z = se3.translation().z();

    // Extract rotation as quaternion
    Eigen::Quaternionf quaternion(se3.rotationMatrix());
    pose.rotation.x = quaternion.x();
    pose.rotation.y = quaternion.y();
    pose.rotation.z = quaternion.z();
    pose.rotation.w = quaternion.w();
}

#endif
