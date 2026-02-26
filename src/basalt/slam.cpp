#include "slam/basalt/slam.h"

BasaltSLAM::BasaltSLAM(rclcpp::Logger logger, std::string configurationFilePath,
                       std::string calibrationFilePath,
                       std::string setupCameraType)
    : Slam(logger), mpLogger(logger),
      mpConfigurationFilePath(configurationFilePath),
      mpCalibrationFilePath(calibrationFilePath) {

    this->InitialiseSlam(calibrationFilePath, setupCameraType);
}

BasaltSLAM::BasaltSLAM(rclcpp::Logger logger) : Slam(logger), mpLogger(logger) {
    RCLCPP_INFO(mpLogger, "Created BasaltSLAM object, initialisation required");
}

BasaltSLAM::~BasaltSLAM() { Shutdown(); }

bool BasaltSLAM::InitialiseSlam(std::string calibrationFilePath,
                                std::string setupCameraType) {
    if (setupCameraType == "monocular-only") {
        mpSlamMode = basalt::SlamMode::VO;
    }
    mpController = std::make_unique<basalt::Controller>(
        mpConfigurationFilePath, mpCalibrationFilePath, mpSlamMode);
    return true;
}

void BasaltSLAM::TrackMonocular(Frame &frame, Sophus::SE3f &tcw) {
    cv::Mat img = frame.getImage();
    basalt::ManagedImage<uint16_t>::Ptr image = basalt::readOpenCVImage(img);
    std::vector<basalt::ImageData> data;
    data.push_back(basalt::ImageData());
    data[0].img = image;
    basalt::OpticalFlowInput::Ptr input;
    input->t_ns = frame.getTimestampNSec();
    input->img_data = data;
    mpController->TrackMonocular(input, tcw);
}

cv::Mat BasaltSLAM::GetCurrentFrame() {
    return cv::Mat(); // Empty body
}

void BasaltSLAM::Shutdown() {
    if (mpController) {
        mpController.reset();
    }
}

int BasaltSLAM::GetTrackingState() {
    return 0; // Empty body
}
