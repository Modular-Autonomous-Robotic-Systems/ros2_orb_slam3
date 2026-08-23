#ifndef MONOCULAR_SLAM_NODE_HPP
#define MONOCULAR_SLAM_NODE_HPP

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include "slam/node.hpp"
// MORB_SLAM related includes
#include <MORB_SLAM/Frame.h>
#include <MORB_SLAM/Map.h>
#include <MORB_SLAM/MapPoint.h>
#include <MORB_SLAM/System.h>
#include <MORB_SLAM/Tracking.h>

#include <MORB_SLAM/ImprovedTypes.hpp>
#include <MORB_SLAM/Packet.hpp>

class MonoMORBSLAM : public Slam {
public:
    MonoMORBSLAM(rclcpp::Logger logger);
    ~MonoMORBSLAM() {
        if (mpMORBSLAM) {
            mpMORBSLAM->Shutdown();
        }
        RCLCPP_INFO(mpLogger, "Destroying Slam3 object");
    }

    void Shutdown() {
        if (mpMORBSLAM) {
            mpMORBSLAM->Shutdown();
        }
    }
    cv::Mat GetCurrentFrame() {
        // if(mpMORBSLAM){
        // 	return mpMORBSLAM->GetCurrentFrame();
        // }
        // else{
        // 	return cv::Mat();
        // }
    };
    bool TrackMonocular(Frame& frame, Sophus::SE3f& tcw);
    void InitialiseSlam(
        std::shared_ptr<custom_interfaces::srv::StartupSlam::Request> request,
        std::shared_ptr<custom_interfaces::srv::StartupSlam::Response>
            response);

private:
    // ORBSLAM3 Related pointers
    std::string mpVocabFilePath = "";
    std::string mpSettingsFilePath = "";
    std::unique_ptr<MORB_SLAM::System> mpMORBSLAM = nullptr;

    int GetTrackingState() {
        // if(mpMORBSLAM){
        // 	return mpMORBSLAM->GetTrackingState();
        // }
        // else{
        // 	return -1;
        // }
    };
};
// typedef MonoMORBSLAM MonoMORBSLAM;
#endif
