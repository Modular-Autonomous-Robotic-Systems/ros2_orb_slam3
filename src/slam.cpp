#include "slam/slam.hpp"

bool readYAMLFile(std::string &yamlPath, YAML::Node &output) {
    if (std::filesystem::exists(yamlPath)) {
        if (!std::filesystem::is_directory(yamlPath)) {
            try {
                output = YAML::LoadFile(yamlPath);
            } catch (const std::exception &err) {
                std::cout << err.what() << std::endl;
            }
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

Frame::Frame(std::shared_ptr<cv::Mat> image, long &timestamp) {
    mpImage = image;
    mpTimestamp = timestamp;
}

Imu::Imu(ImuMsg::SharedPtr msg) {
    mpTimestamp =
        msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;

    if (msg->linear_acceleration_covariance[0] != -1.0) {
        mpHasLinearAcceleration = true;
        mpLinearAcceleration << msg->linear_acceleration.x,
            msg->linear_acceleration.y, msg->linear_acceleration.z;
        mpLinearAccelerationCovariance = convertCovarianceArray(msg->linear_acceleration_covariance);
    }

    if (msg->angular_velocity_covariance[0] != -1.0) {
        mpHasAngularVelocity = true;
        mpAngularVelocity << msg->angular_velocity.x, msg->angular_velocity.y,
            msg->angular_velocity.z;
        mpAngularVelocityCovariance = convertCovarianceArray(msg->angular_velocity_covariance);
    }

    if (msg->orientation_covariance[0] != -1.0) {
        mpHasOrientation = true;
        mpOrientation =
            Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                               msg->orientation.y, msg->orientation.z);
        mpOrientationCovariance = convertCovarianceArray(msg->orientation_covariance);
    }
}

Eigen::Matrix3d Imu::convertCovarianceArray(const std::array<double, 9>& cov_array) const {
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = cov_array[i * 3 + j];
        }
    }
    return mat;
}

Slam::Slam(rclcpp::Logger logger) : mpLogger(logger) {
    RCLCPP_INFO(mpLogger, "Creating SLAM Base Object");
}
