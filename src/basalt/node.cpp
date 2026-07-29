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
    slam_type_desc.description =
        "Type of SLAM to configure and activate. This "
        "node supports VSLAM and VISLAM";
    slam_type_desc.type = 4;
    this->declare_parameter<std::string>("slam_type", "VSLAM", slam_type_desc);

    rcl_interfaces::msg::ParameterDescriptor imu_topic_name_desc;
    imu_topic_name_desc.description =
        "IMU Topic Name (required when slam_type == VISLAM)";
    imu_topic_name_desc.type = 4;  // PARAMETER_STRING
    this->declare_parameter<std::string>("imu_topic_name", "",
                                         imu_topic_name_desc);
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

    mpIMUTopicName = this->get_parameter("imu_topic_name").as_string();

    if (mpSLAMType == eSLAMType::VISLAM && mpIMUTopicName.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "slam_type is VISLAM but 'imu_topic_name' is empty. "
                     "Configuration failed.");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "IMU Topic Name: %s",
                mpIMUTopicName.c_str());

    return CallbackReturn::SUCCESS;
}

CallbackReturn
BasaltSLAMNode::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(this->get_logger(), "Creating SLAM Object in Basalt Slam Node");

    std::string setupCameraType;
    if (mpSLAMType == eSLAMType::VSLAM) {
        setupCameraType = "monocular-only";
    } else if (mpSLAMType == eSLAMType::VISLAM) {
        setupCameraType = "monocular-inertial";
    }
    mpSlam = std::make_unique<BasaltSLAM>(
        this->get_logger(), mpConfigurationFilePath, mpCalibrationFilePath,
        setupCameraType);

    mpTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    mpFrameSubscriber = this->create_subscription<ImageMsg>(
        mpCameraTopicName, 10,
        std::bind(&BasaltSLAMNode::GrabImage, this, std::placeholders::_1));

    if (mpSLAMType == eSLAMType::VISLAM) {
        // /ap/imu/experimental/data is published BEST_EFFORT / VOLATILE /
        // KEEP_LAST(5) by AP_DDS (AP_DDS_Topic_Table.h). A RELIABLE subscriber
        // -- which is what a bare depth means -- will not match it, so the
        // callback would never fire and VIO would run with zero IMU data.
        // rclcpp::SensorDataQoS() is exactly BEST_EFFORT / VOLATILE /
        // KEEP_LAST(5) and is also the ROS 2 convention for sensor streams.
        // See ros_ws/context/ros-qos-compatibility.md.
        mpIMUSubscriber = this->create_subscription<ImuMsg>(
            mpIMUTopicName, rclcpp::SensorDataQoS(),
            std::bind(&BasaltSLAMNode::GrabIMU, this, std::placeholders::_1));
    }

    mpAnnotatedFramePublisher =
        this->create_publisher<ImageMsg>("~/annotated_frame", 10);

    return CallbackReturn::SUCCESS;
}

CallbackReturn BasaltSLAMNode::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    CallbackReturn result = SlamNode::on_deactivate(previous_state);
    mpSlam->StopSlamVisualiser();

    if (mpAnnotatedFramePublisher) {
        mpAnnotatedFramePublisher.reset();
    }
    if (mpFrameSubscriber) {
        mpFrameSubscriber.reset();
    }
    if (mpIMUSubscriber) {
        mpIMUSubscriber.reset();
    }
    if (mpTfBroadcaster) {
        mpTfBroadcaster.reset();
    }
    if (mpSlam) {
        mpSlam.reset();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn BasaltSLAMNode::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
    return CallbackReturn::SUCCESS;
}

void BasaltSLAMNode::Update() {
    SlamNode::Update();
    PublishFrame();
}

void BasaltSLAMNode::GrabImage(const ImageMsg::SharedPtr msg) {
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int sec = msg->header.stamp.sec;
    int nsec = msg->header.stamp.nanosec;
    long timestamp = sec * 1e9 + nsec;
    RCLCPP_DEBUG(this->get_logger(), "Received image frame with timestamp: %ld",
                 timestamp);
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
        RCLCPP_DEBUG(this->get_logger(),
                     "Received imu frame with timestamp: %ld",
                     imuPtr->getTimestampNSec());
        if (mpLastIMUTimestamp == -1) {
            mpLastIMUTimestamp = imuPtr->getTimestampNSec();
        } else if (mpLastIMUTimestamp > imuPtr->getTimestampNSec()) {
            RCLCPP_DEBUG(this->get_logger(),
                         "Received out of order IMU, dropping");
            return;
        } else if (mpLastIMUTimestamp == imuPtr->getTimestampNSec()) {
            RCLCPP_DEBUG(this->get_logger(),
                         "Received duplicate IMU, dropping");
            return;
        }
        mpLastIMUTimestamp = imuPtr->getTimestampNSec();
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
