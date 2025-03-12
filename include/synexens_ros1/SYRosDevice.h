#ifndef SYRosDevice_H
#define SYRosDevice_H
#include <iostream>
#include "SYSDKInterface.h"
#include "SYDataDefine.h"
#include "SYCalibrationTransformData.h"
#include "SYRosDeviceParmas.h"
#include "SYRosTypes.h"

#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>

using namespace Synexens;

enum PUBLISHER_TYPE
{
    // DEPTH
    DEPTH,
    // ir
    IR,
    // RGB
    RGB,
    // points
    POINTS
};

class SYRosDevice
{
private:
    /* data */
public:
    SYRosDevice(const ros::NodeHandle &n = ros::NodeHandle(), const ros::NodeHandle &p = ros::NodeHandle("~"));
    ~SYRosDevice();

    // start cameras
    SYErrorCode startCameras();
    // stop cameras
    void stopCameras();
    void PrintErrorCode(std::string strFunc, Synexens::SYErrorCode errorCode);
    std::map<unsigned int, Synexens::SYStreamType> m_mapStreamType;
    void ProcessFrameData(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData = nullptr);

    // fill point cloud
    void FillPointCloud(int nWidth, int nHeight, SYPointCloudData *pPCLData, sensor_msgs::PointCloud2Ptr &point_cloud);

    // set option
    void SetOption(unsigned int nDeviceID, SYDeviceType deviceType);

    // ros publisher map
    std::map<int, std::map<PUBLISHER_TYPE, ros::Publisher>> m_mapRosPublisher;
    // image publisher map
    std::map<int, std::map<PUBLISHER_TYPE, image_transport::Publisher>> m_mapImagePublisher;

    // ros node
    ros::NodeHandle m_node;
    // ros private node
    ros::NodeHandle m_privateNode;
    // image transport
    image_transport::ImageTransport m_imageTranspoort;
    // calibration data
    SYCalibrationTransformData m_calibrationData;

    image_transport::Publisher m_testImageP;

    // depth camera info
    sensor_msgs::CameraInfo m_depthCameraInfo;
    // rgb camera info
    sensor_msgs::CameraInfo m_rgbCameraInfo;
    // ir camera info
    sensor_msgs::CameraInfo m_irCameraInfo;

    // camera config
    SYCameraConfig m_cameraConfig;
    // ros config params
    SYRosDeviceParmas m_parmas;

    // setOpton status
    bool m_bSetOptionStatus = true;
};

#endif