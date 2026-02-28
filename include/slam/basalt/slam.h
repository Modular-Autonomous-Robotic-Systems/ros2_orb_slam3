#include "basalt/controller.h"
#include "basalt/io/rt_io.h"
#include "rclcpp/rclcpp.hpp"
#include "slam/slam.hpp"
#include <memory>
#include <string>

class BasaltSLAM : public Slam {
  public:
    BasaltSLAM(rclcpp::Logger logger, std::string configurationFilePath,
               std::string calibrationFilePath, std::string setupCameraType);
    BasaltSLAM(rclcpp::Logger logger);
    ~BasaltSLAM();

    bool InitialiseSlam(std::string calibrationFilePath,
                        std::string setupCameraType);
    void TrackMonocular(Frame &frame, Sophus::SE3f &tcw) override;
    void TrackMonocularIMU(Frame &frame,
                           std::vector<std::shared_ptr<Imu>> &imuVec,
                           Sophus::SE3f &tcw) override;
    cv::Mat GetCurrentFrame() override;
    void Shutdown() override;
    int GetTrackingState() override;

  private:
    rclcpp::Logger mpLogger;
    std::unique_ptr<basalt::Controller> mpController;
    std::string mpConfigurationFilePath;
    std::string mpCalibrationFilePath;
    basalt::SlamMode mpSlamMode;
};
