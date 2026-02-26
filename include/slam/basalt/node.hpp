#pragma once

#include "slam/node.hpp"
#include "slam/basalt/slam.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class BasaltSLAMNode : public SlamNode {
  public:
    BasaltSLAMNode();
    ~BasaltSLAMNode();

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  protected:
    std::string mpCameraTopicName;
    std::string mpCalibrationFilePath;
    std::string mpConfigurationFilePath;

  private:
    Frame mpCurrentFrame;
    std::unique_ptr<BasaltSLAM> mpSlam = nullptr;
    Eigen::Matrix3d mpBasaltToROSTransform;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr mpFrameSubscriber;
    rclcpp::Publisher<ImageMsg>::SharedPtr mpAnnotatedFramePublisher;

    void Update();
    void GrabImage(const ImageMsg::SharedPtr msg);
    void PublishFrame();
};
