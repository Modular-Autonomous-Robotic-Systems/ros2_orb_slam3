#include "slam/basalt/slam.h"

#include <memory>

#include "basalt/visualisation/visualiser.h"

BasaltSLAM::BasaltSLAM(rclcpp::Logger logger, std::string configurationFilePath,
                       std::string calibrationFilePath,
                       std::string setupCameraType, bool useVisualisation)
    : Slam(logger),
      mpLogger(logger),
      mpConfigurationFilePath(configurationFilePath),
      mpCalibrationFilePath(calibrationFilePath) {
    this->InitialiseSlam(calibrationFilePath, setupCameraType,
                         useVisualisation);
}

BasaltSLAM::BasaltSLAM(rclcpp::Logger logger) : Slam(logger), mpLogger(logger) {
    RCLCPP_INFO(mpLogger, "Created BasaltSLAM object, initialisation required");
}

BasaltSLAM::~BasaltSLAM() {
    if (mpVisualiserThread.joinable()) {
        StopSlamVisualiser();
    }
    Shutdown();
}

bool BasaltSLAM::InitialiseSlam(std::string calibrationFilePath,
                                std::string setupCameraType,
                                bool useVisualisation) {
    if (setupCameraType == "monocular-only") {
        mpSlamMode = basalt::SlamMode::VO;
    } else if (setupCameraType == "monocular-inertial") {
        mpSlamMode = basalt::SlamMode::VIO;
    }
    mpController = std::make_unique<basalt::Controller>(
        mpConfigurationFilePath, mpCalibrationFilePath, mpSlamMode);
    mpController->load_config();
    basalt::Calibration<double>& calib = mpController->GetCalibration();
    Eigen::Matrix<double, 3, 1> bg;
    Eigen::Matrix<double, 3, 3> sg;
    Eigen::Matrix<double, 3, 1> ba;
    Eigen::Matrix<double, 3, 3> sa;
    calib.calib_gyro_bias.getBiasAndScale(bg, sg);
    calib.calib_accel_bias.getBiasAndScale(ba, sa);

    mpController->initialize(0,                        // t_ns: start at origin
                             Sophus::SE3d(),           // T_w_i: identity pose
                             Eigen::Vector3d::Zero(),  // vel_w_i: zero velocity
                             bg.cast<double>(),  // gyro bias from calibration
                             ba.cast<double>(),  // accel bias from calibration
                             false,  // useProducerConsumerArchitecture = false
                             useVisualisation  // enableVisualisation
    );
    if (useVisualisation) {
        mpSlamVisualiser =
            std::make_unique<basalt::SlamVisualiser>(*mpController);

        // Start() must run on the same thread as Run() as OpenGL context is not
        // shared between threads
        std::promise<void> startedPromise;
        std::future<void> started = startedPromise.get_future();
        mpVisualiserThread = std::thread(&BasaltSLAM::RunSlamVisualiser, this,
                                         std::move(startedPromise));
        // error thrown here if promise fails
        started.get();
    }
    return true;
}

void BasaltSLAM::RunSlamVisualiser(std::promise<void> startedPromise) {
    try {
        mpSlamVisualiser->Start();
    } catch (...) {
        startedPromise.set_exception(std::current_exception());
        return;
    }
    startedPromise.set_value();
    mpSlamVisualiser->Run();
}

void BasaltSLAM::StopSlamVisualiser() {
    if (!mpSlamVisualiser) {
        return;
    }
    mpSlamVisualiser->Stop();
    if (mpVisualiserThread.joinable()) {
        mpVisualiserThread.join();
    }
}

bool BasaltSLAM::TrackMonocular(Frame& frame, Sophus::SE3f& tcw) {
    cv::Mat img = frame.getImage();
    basalt::ManagedImage<uint16_t>::Ptr image = basalt::readOpenCVImage(img);
    std::vector<basalt::ImageData> data;
    data.push_back(basalt::ImageData());
    data[0].img = image;
    basalt::OpticalFlowInput::Ptr input =
        std::make_shared<basalt::OpticalFlowInput>();
    input->t_ns = frame.getTimestampNSec();
    input->img_data = data;
    bool status = mpController->TrackMonocular(input, tcw);
    return status;
}

void BasaltSLAM::GrabIMU(std::shared_ptr<Imu>& data) {
    const basalt::ImuData<double>::Ptr imuData = data->toBasaltImuData();
    mpController->GrabIMU(imuData);
}

cv::Mat BasaltSLAM::GetCurrentFrame() {
    return cv::Mat();  // Empty body
}

void BasaltSLAM::Shutdown() {
    if (mpController) {
        mpController.reset();
    }
}

int BasaltSLAM::GetTrackingState() {
    return 0;  // Empty body
}
