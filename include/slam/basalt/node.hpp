#pragma once

#include "slam/basalt/slam.h"
#include "slam/node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class BasaltSLAMNode : public SlamNode {
  public:
    BasaltSLAMNode();
    ~BasaltSLAMNode();

    CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  protected:
    std::string mpCameraTopicName;
    std::string mpCalibrationFilePath;
    std::string mpConfigurationFilePath;
    eSLAMType mpSLAMType;

  private:
    Frame mpCurrentFrame;
    std::unique_ptr<BasaltSLAM> mpSlam = nullptr;
    Eigen::Matrix3d mpBasaltToROSTransform;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr mpFrameSubscriber;
    rclcpp::Publisher<ImageMsg>::SharedPtr mpAnnotatedFramePublisher;

    void Update();
    void GrabImage(const ImageMsg::SharedPtr msg);
    void GrabIMU(const ImuMsg::SharedPtr msg);
    void PublishFrame();
};
