#include "slam/basalt/node.hpp"
#include "slam/slam.hpp"

BasaltSLAMNode::BasaltSLAMNode() : SlamNode("basalt_slam_node") {
    rcl_interfaces::msg::ParameterDescriptor camera_topic_name_desc;
    camera_topic_name_desc.description = "Camera Topic Name";
    camera_topic_name_desc.type = 4;
    this->declare_parameter<std::string>("camera_topic_name", "/camera",
                                         camera_topic_name_desc);

    rcl_interfaces::msg::ParameterDescriptor calib_file_path_desc;
    calib_file_path_desc.description = "Path to Basalt Calibration File";
    calib_file_path_desc.type = 4;
    this->declare_parameter<std::string>("calibration_file_path", "",
                                         calib_file_path_desc);

    rcl_interfaces::msg::ParameterDescriptor config_file_path_desc;
    config_file_path_desc.description = "Path to Basalt Configuration File";
    config_file_path_desc.type = 4;
    this->declare_parameter<std::string>("configuration_file_path", "",
                                         config_file_path_desc);

    rcl_interfaces::msg::ParameterDescriptor slam_type_desc;
    slam_type_desc.description = "Type of SLAM to configure and activate. This "
                                 "node supports VSLAM and VISLAM";
    slam_type_desc.type = 4;
    this->declare_parameter<std::string>("slam_type", "VSLAM", slam_type_desc);

    mpBasaltToROSTransform = Eigen::Matrix3d::Identity();
}

BasaltSLAMNode::~BasaltSLAMNode() {
    if (mpSlam) {
        mpSlam.reset();
    }
}

CallbackReturn
BasaltSLAMNode::on_configure(const rclcpp_lifecycle::State &previous_state) {
    mpCameraTopicName = this->get_parameter("camera_topic_name").as_string();
    mpCalibrationFilePath =
        this->get_parameter("calibration_file_path").as_string();
    mpConfigurationFilePath =
        this->get_parameter("configuration_file_path").as_string();
    std::string SLAMType = this->get_parameter("slam_type").as_string();
    if (SLAMType == "VSLAM") {
        mpSLAMType = eSLAMType::VSLAM;
    } else if (SLAMType == "VISLAM") {
        mpSLAMType = eSLAMType::VISLAM;
    }

    RCLCPP_INFO(this->get_logger(), "Camera Topic Name: %s",
                mpCameraTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "Calibration File Path: %s",
                mpCalibrationFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Configuration File Path: %s",
                mpConfigurationFilePath.c_str());

    return CallbackReturn::SUCCESS;
}

CallbackReturn
BasaltSLAMNode::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(this->get_logger(), "Creating SLAM Object in Basalt Slam Node");

    mpSlam = std::make_unique<BasaltSLAM>(
        this->get_logger(), mpConfigurationFilePath, mpCalibrationFilePath,
        "monocular-only");

    mpTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(
        tf2_ros::TransformBroadcaster(this));

    mpFrameSubscriber = this->create_subscription<ImageMsg>(
        mpCameraTopicName, 10,
        std::bind(&BasaltSLAMNode::GrabImage, this, std::placeholders::_1));

    mpAnnotatedFramePublisher =
        this->create_publisher<ImageMsg>("~/annotated_frame", 10);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
BasaltSLAMNode::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    if (mpAnnotatedFramePublisher) {
        mpAnnotatedFramePublisher.reset();
    }
    if (mpFrameSubscriber) {
        mpFrameSubscriber.reset();
    }
    if (mpTfBroadcaster) {
        mpTfBroadcaster.reset();
    }
    if (mpSlam) {
        mpSlam.reset();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn
BasaltSLAMNode::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
}

void BasaltSLAMNode::Update() {
    SlamNode::Update();
    PublishFrame();
}

void BasaltSLAMNode::GrabImage(const ImageMsg::SharedPtr msg) {
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int sec = msg->header.stamp.sec;
    int nsec = msg->header.stamp.nanosec;
    long timestamp = sec * 1e9 + nsec;
    mpCurrentFrame =
        Frame(std::make_shared<cv::Mat>(m_cvImPtr->image), timestamp);
    cv::Mat img = mpCurrentFrame.getImage();
    Sophus::SE3f tcw;

    if (!img.empty()) {
        mpSlam->TrackMonocular(mpCurrentFrame, tcw);
        mpTwc = tcw.inverse();
        Update();
    }
}

void BasaltSLAMNode::GrabIMU(const ImuMsg::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "received IMU msg");
    if (mpSLAMType == eSLAMType::VISLAM) {
        std::shared_ptr<Imu> imuPtr = std::make_shared<Imu>(msg);
        mpSlam->GrabIMU(imuPtr);
    } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "dropping IMU data as it is not needed");
    }
}

void BasaltSLAMNode::PublishFrame() {
    sensor_msgs::msg::Image::UniquePtr img_msg_ptr(
        new sensor_msgs::msg::Image());
    cv_bridge::CvImage(std_msgs::msg::Header(),
                       sensor_msgs::image_encodings::BGR8,
                       mpCurrentFrame.getImage().clone())
        .toImageMsg(*img_msg_ptr);
    img_msg_ptr->header.stamp = this->now();
    mpAnnotatedFramePublisher->publish(std::move(img_msg_ptr));
}
