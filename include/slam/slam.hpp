#ifndef SLAM_OBJECT_HPP
#define SLAM_OBJECT_HPP

// ORBSLAM3 Dependencies
#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <memory>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <thread>

#include "custom_interfaces/srv/startup_slam.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#ifdef USE_ORBSLAM3
#include "MapPoint.h"
#endif
#ifdef USE_MORBSLAM
#include "MORB_SLAM/MapPoint.h"
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "custom_interfaces/msg/point_cloud3.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

// ABI guard, parent (slam package) side. See src/slam/context/SIMD.md and
// CMakeLists.txt's add_compile_definitions(). If the pin is removed or a
// subproject reintroduces a differing -march, this fires at compile time
// instead of corrupting the heap at runtime.
#ifdef NRT_EIGEN_ABI_PIN
static_assert(EIGEN_MAX_ALIGN_BYTES == NRT_EIGEN_ABI_PIN,
              "EIGEN_MAX_ALIGN_BYTES disagrees with the pin set in "
              "slam/CMakeLists.txt - see ros_ws/src/slam/context/SIMD.md");
#endif

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#if __has_include(<basalt/imu/imu_types.h>)
#include <basalt/imu/imu_types.h>
#endif

using StartupSlam = custom_interfaces::srv::StartupSlam;
using ShutdownSlam = std_srvs::srv::Trigger;
using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;
using MarkerMsg = visualization_msgs::msg::Marker;
using PointMsg = geometry_msgs::msg::Point;
using MapMsg = custom_interfaces::msg::PointCloud3;

class Data {
public:
    Data() = default;
    Data(long timestamp) : mpTimestamp(timestamp) {}
    double getTimestampMSec() const { return mpTimestamp * 1e-6; }
    double getTimestampSec() const { return mpTimestamp * 1e-9; }
    long getTimestampNSec() const { return mpTimestamp; }

protected:
    long mpTimestamp = 0;
};

class Imu : public Data {
public:
    Imu() = default;
    Imu(ImuMsg::SharedPtr msg);

    Eigen::Vector3d getLinearAcceleration() const {
        return mpLinearAcceleration;
    }
    Eigen::Vector3d getAngularVelocity() const { return mpAngularVelocity; }
    Eigen::Quaterniond getOrientation() const { return mpOrientation; }
    bool hasLinearAcceleration() const { return mpHasLinearAcceleration; }
    bool hasAngularVelocity() const { return mpHasAngularVelocity; }
    bool hasOrientation() const { return mpHasOrientation; }

    Eigen::Matrix3d getLinearAccelerationCovariance() const {
        return mpLinearAccelerationCovariance;
    }
    Eigen::Matrix3d getAngularVelocityCovariance() const {
        return mpAngularVelocityCovariance;
    }
    Eigen::Matrix3d getOrientationCovariance() const {
        return mpOrientationCovariance;
    }

    Eigen::Matrix3d convertCovarianceArray(
        const std::array<double, 9>& cov_array) const;

    const basalt::ImuData<double>::Ptr toBasaltImuData() const {
        const basalt::ImuData<double>::Ptr data(new basalt::ImuData<double>);
        data->t_ns = getTimestampNSec();
        data->accel = mpLinearAcceleration;
        data->gyro = mpAngularVelocity;
        return data;
    }

private:
    Eigen::Vector3d mpLinearAcceleration;
    Eigen::Vector3d mpAngularVelocity;
    Eigen::Quaterniond mpOrientation;

    bool mpHasLinearAcceleration = false;
    bool mpHasAngularVelocity = false;
    bool mpHasOrientation = false;

    Eigen::Matrix3d mpLinearAccelerationCovariance;
    Eigen::Matrix3d mpAngularVelocityCovariance;
    Eigen::Matrix3d mpOrientationCovariance;
};

class Frame : public Data {
public:
    Frame() = default;
    Frame(std::shared_ptr<cv::Mat> image, long& timestamp);
    cv::Mat getImage() { return *(mpImage.get()); }
    Sophus::SE3f getTransform() { return Tcw; }

private:
    // TODO not a good idea to have nested threadsafe. this must be used for a
    // collection of data always to be used in one thread at a time.
    std::shared_ptr<cv::Mat> mpImage;

    Sophus::SE3f Tcw;
};

enum class eSLAMAlgorithm { NOT_SET = -1, ORBSLAM3 = 0, BASALT = 1 };
enum class eSLAMType { VSLAM = 0, VISLAM = 1 };

class Slam {
public:
    Slam(rclcpp::Logger logger);
    virtual cv::Mat GetCurrentFrame() = 0;
    virtual void TrackMonocular(Frame& frame, Sophus::SE3f& tcw) = 0;
    virtual void TrackMonocularIMU(Frame& frame,
                                   std::vector<std::shared_ptr<Imu>>& imuVec,
                                   Sophus::SE3f& tcw) {
        RCLCPP_ERROR(
            mpLogger,
            "TrackMonocularIMU not implmented for this SLAM algorithm");
    }
    virtual void Shutdown() = 0;
#ifdef USE_ORBSLAM3
    virtual void SetFrameMapPointUpdateCallback(
        std::function<void(std::vector<ORB_SLAM3::MapPoint*>&,
                           const Sophus::SE3<float>&)>
            callback) = 0;
#endif
    rclcpp::Logger mpLogger;

private:
    // Getter Methods
    virtual int GetTrackingState() = 0;
};

#endif
