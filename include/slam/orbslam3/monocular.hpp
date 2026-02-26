#include "slam/node.hpp"
#ifdef USE_ORBSLAM3
#include "slam/orbslam3/monocular.hpp"
#endif

#ifdef  USE_MORBSLAM
#include "slam/morbslam/monocular.hpp"
#endif

class VisualSlamNode : public SlamNode
{
	public:
		VisualSlamNode();
		~VisualSlamNode();
		/**
		 * @brief Lifecycle callback for the 'configuring' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful configuration.
		 * @return CallbackReturn::FAILURE on configuration failure.
		 */
		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Lifecycle callback for the 'activating' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful activation.
		 * @return CallbackReturn::FAILURE on activation failure.
		 */
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Lifecycle callback for the 'deactivating' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful deactivation.
		 * @return CallbackReturn::FAILURE on deactivation failure.
		 */
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

		/**
		 * @brief Lifecycle callback for the 'cleaning up' transition.
		 * @param previous_state The previous lifecycle state.
		 * @return CallbackReturn::SUCCESS on successful cleanup.
		 * @return CallbackReturn::FAILURE on cleanup failure.
		 */
		CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);

	protected:
		std::string mpCameraTopicName;
		std::string mpVocabFilePath;

	private:
		// Node Configurations
		Frame mpCurrentFrame;
		std::unique_ptr<Slam> mpSlam = nullptr;
		Eigen::Matrix3d mpOrbToROSTransform;
		// ORB_SLAM3 related attributes
#ifdef USE_ORBSLAM3
		ORB_SLAM3::Tracking::eTrackingState mpState;
#endif
#ifdef USE_MORBSLAM
		MORB_SLAM::TrackingState mpState = MORB_SLAM::TrackingState::SYSTEM_NOT_READY;
#endif
		// Image pointer for receiving and passing images to SLAM
		cv_bridge::CvImagePtr m_cvImPtr;
		// Publishers, Subscribers, Services and Actions
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpFrameSubscriber;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mpAnnotatedFramePublisher;
		rclcpp::Publisher<MapMsg>::SharedPtr mpMapPointPublisher;
		rclcpp::Service<custom_interfaces::srv::StartupSlam>::SharedPtr mpSlamStartupService;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mpSlamShutdownService;
		// Callbacks and Methods
		void Update();
		void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
		void PublishFrame();
#ifdef USE_ORBSLAM3
		void PublishMapPointsCallback(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw);
		void MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> &mapPoints, const Sophus::SE3<float> &tcw, sensor_msgs::msg::PointCloud2 &cloud);
		void SophusToGeometryMsgTransform(const Sophus::SE3<float>& se3, geometry_msgs::msg::Transform &pose);
		// TODO need to make this a parameter
		int mpNumMinObsPerPoint = 2;
#endif
};
