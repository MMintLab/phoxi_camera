//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_ROSINTERFACE_H
#define PROJECT_ROSINTERFACE_H

#define OKRESPONSE "Ok"

//ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <phoxi_camera/phoxi_cameraConfig.h>

//diagnostic updater
#include <boost/thread/mutex.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

//messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <std_srvs/Empty.h>
#include <phoxi_camera/PhoXiInterface.h>
#include <phoxi_camera/GetDeviceList.h>
#include <phoxi_camera/ConnectCamera.h>
#include <phoxi_camera/IsConnected.h>
#include <phoxi_camera/IsAcquiring.h>
#include <phoxi_camera/GetBool.h>
#include <phoxi_camera/Empty.h>
#include <phoxi_camera/TriggerImage.h>
#include <phoxi_camera/GetFrame.h>
#include <phoxi_camera/SaveFrame.h>
#include <phoxi_camera/SaveLastFrame.h>
#include <phoxi_camera/GetHardwareIdentification.h>
#include <phoxi_camera/GetSupportedCapturingModes.h>
#include <phoxi_camera/SetCoordinatesSpace.h>
#include <phoxi_camera/SetTransformationMatrix.h>
#include <phoxi_camera/GetString.h>

//std
#include <vector>
#include <algorithm>

//others
#include <phoxi_camera/PhoXiException.h>
#include <phoxi_camera/RosConversions.h>


namespace phoxi_camera {
    class RosInterface : protected PhoXiInterface {
    public:
        RosInterface();

    protected:
        void publishFrame(pho::api::PFrame frame);

        pho::api::PFrame getPFrame(int id = -1);

        int triggerImage();

        void connectCamera(std::string HWIdentification,
                           pho::api::PhoXiTriggerMode mode = pho::api::PhoXiTriggerMode::Software,
                           bool startAcquisition = true);

        std::string getTriggerMode(pho::api::PhoXiTriggerMode mode);

        void getDefaultDynamicReconfigureConfig(phoxi_camera::phoxi_cameraConfig& config);

        std::string frameId;
    private:
        bool getDeviceList(phoxi_camera::GetDeviceList::Request& req, phoxi_camera::GetDeviceList::Response& res);

        bool connectCamera(phoxi_camera::ConnectCamera::Request& req, phoxi_camera::ConnectCamera::Response& res);

        bool isConnected(phoxi_camera::IsConnected::Request& req, phoxi_camera::IsConnected::Response& res);

        bool isAcquiring(phoxi_camera::IsAcquiring::Request& req, phoxi_camera::IsAcquiring::Response& res);

        bool isConnected(phoxi_camera::GetBool::Request& req, phoxi_camera::GetBool::Response& res);

        bool isAcquiring(phoxi_camera::GetBool::Request& req, phoxi_camera::GetBool::Response& res);

        bool startAcquisition(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool stopAcquisition(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool startAcquisition(phoxi_camera::Empty::Request& req, phoxi_camera::Empty::Response& res);

        bool stopAcquisition(phoxi_camera::Empty::Request& req, phoxi_camera::Empty::Response& res);

        bool triggerImage(phoxi_camera::TriggerImage::Request& req, phoxi_camera::TriggerImage::Response& res);

        bool getFrame(phoxi_camera::GetFrame::Request& req, phoxi_camera::GetFrame::Response& res);

        bool saveFrame(phoxi_camera::SaveFrame::Request& req, phoxi_camera::SaveFrame::Response& res);

        bool disconnectCamera(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool getHardwareIdentification(phoxi_camera::GetHardwareIdentification::Request& req,
                                       phoxi_camera::GetHardwareIdentification::Response& res);

        bool getSupportedCapturingModes(phoxi_camera::GetSupportedCapturingModes::Request& req,
                                        phoxi_camera::GetSupportedCapturingModes::Response& res);

        bool getApiVersion(phoxi_camera::GetString::Request& req, phoxi_camera::GetString::Response& res);

        bool getFirmwareVersion(phoxi_camera::GetString::Request& req, phoxi_camera::GetString::Response& res);

        void dynamicReconfigureCallback(phoxi_camera::phoxi_cameraConfig& config, uint32_t level);

        void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper& status);

        void diagnosticTimerCallback(const ros::TimerEvent&);

        void initFromPhoXi();

#ifndef PHOXI_API_v1_1

        bool setCoordianteSpace(phoxi_camera::SetCoordinatesSpace::Request& req,
                                phoxi_camera::SetCoordinatesSpace::Response& res);

        bool setTransformation(phoxi_camera::SetTransformationMatrix::Request& req,
                               phoxi_camera::SetTransformationMatrix::Response& res);

        bool saveLastFrame(phoxi_camera::SaveLastFrame::Request& req, phoxi_camera::SaveLastFrame::Response& res);

#endif
        //node handle
        ros::NodeHandle nh;

        //ros service servers
        ros::ServiceServer getDeviceListService;
        ros::ServiceServer connectCameraService;
        ros::ServiceServer isConnectedService;
        ros::ServiceServer isAcquiringService;
        ros::ServiceServer isAcquiringServiceV2;
        ros::ServiceServer isConnectedServiceV2;
        ros::ServiceServer startAcquisitionService;
        ros::ServiceServer stopAcquisitionService;
        ros::ServiceServer startAcquisitionServiceV2;
        ros::ServiceServer stopAcquisitionServiceV2;
        ros::ServiceServer triggerImageService;
        ros::ServiceServer getFrameService;
        ros::ServiceServer saveFrameService;
        ros::ServiceServer saveLastFrameService;
        ros::ServiceServer disconnectCameraService;
        ros::ServiceServer getHardwareIdentificationService;
        ros::ServiceServer getSupportedCapturingModesService;
        ros::ServiceServer setCoordianteSpaceService;
        ros::ServiceServer setTransformationService;
        ros::ServiceServer getApiVersionService;
        ros::ServiceServer getFirmwareVersionService;

        //ros publishers
        ros::Publisher cloudPub;
        ros::Publisher normalMapPub;
        ros::Publisher confidenceMapPub;
        ros::Publisher rawTexturePub;
        ros::Publisher rgbTexturePub;
        ros::Publisher depthMapPub;
        
        // camera info
        image_transport::ImageTransport mono8ImageTransport;
        image_transport::CameraPublisher mono8CameraPublisher;
        camera_info_manager::CameraInfoManager mono8CameraInfoManager;
        

        //dynamic reconfigure
        boost::recursive_mutex dynamicReconfigureMutex;
        dynamic_reconfigure::Server<phoxi_camera::phoxi_cameraConfig> dynamicReconfigureServer;
        phoxi_camera::phoxi_cameraConfig dynamicReconfigureConfig;

        //diagnostic
        diagnostic_updater::Updater diagnosticUpdater;
        diagnostic_updater::FunctionDiagnosticTask PhoXi3DscannerDiagnosticTask;
        ros::Timer diagnosticTimer;

    };
}


#endif //PROJECT_ROSINTERFACE_H
