#include <future>
#include <memory>
#include <string>
#include <thread>

#include "basalt/controller.h"
#include "basalt/io/rt_io.h"
#include "basalt/visualisation/visualiser.h"
#include "rclcpp/rclcpp.hpp"
#include "slam/slam.hpp"

class BasaltSLAM : public Slam {
public:
    BasaltSLAM(rclcpp::Logger logger, std::string configurationFilePath,
               std::string calibrationFilePath, std::string setupCameraType,
               bool useVisualisation);
    BasaltSLAM(rclcpp::Logger logger);
    ~BasaltSLAM();

    bool InitialiseSlam(std::string calibrationFilePath,
                        std::string setupCameraType, bool useVisualisation);
    void TrackMonocular(Frame& frame, Sophus::SE3f& tcw) override;
    cv::Mat GetCurrentFrame() override;
    void Shutdown() override;
    int GetTrackingState() override;
    void GrabIMU(std::shared_ptr<Imu>& data);

    void RunSlamVisualiser(std::promise<void> startedPromise);

    void StopSlamVisualiser();

private:
    rclcpp::Logger mpLogger;
    std::unique_ptr<basalt::Controller> mpController;
    std::string mpConfigurationFilePath;
    std::string mpCalibrationFilePath;
    basalt::SlamMode mpSlamMode;

    std::unique_ptr<basalt::SlamVisualiser> mpSlamVisualiser;
    std::thread mpVisualiserThread;
};
