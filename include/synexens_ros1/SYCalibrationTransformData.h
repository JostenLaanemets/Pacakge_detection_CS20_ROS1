#ifndef SYCalibrationTransformData_H
#define SYCalibrationTransformData_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <angles/angles.h>

#include "SYDataDefine.h"

using namespace Synexens;
class SYCalibrationTransformData
{
private:
    /* data */
public:
    void initialize();

    // set depth intrinsics params
    void setDepthCameraCalib(const SYIntrinsics &intrinsics);
    // set rgb intrinsics params
    void setColorCameraCalib(const SYIntrinsics &intrinsics);
    // get depth width
    int getDepthWidth();
    // get depth height
    int getDepthHeight();
    // get rgb width
    int getColorWidth();
    // get rgb height
    int getColorHeight();
    // get depth camera info
    void getDepthCameraInfo(sensor_msgs::CameraInfo &camera_info, SYIntrinsics *intrinsics = nullptr);
    // get rgb camera info
    void getRgbCameraInfo(sensor_msgs::CameraInfo &camera_info, SYIntrinsics *intrinsics = nullptr);

    // set camera info
    static void setCameraInfo(const SYIntrinsics &parameters, sensor_msgs::CameraInfo &camera_info);

    SYIntrinsics m_rgbCameraIntrinsics;
    SYIntrinsics m_depthCameraIntrinsics;

    std::string m_tfPrefix = "";
    std::string m_cameraBaseFrame = "camera_base";
    std::string m_rgbCameraFrame = "rgb_camera_link";
    std::string m_depthCameraFrame = "depth_camera_link";

private:
    void printCameraCalibration(SYIntrinsics &calibration);
    void publishDepthToBaseTf();

    tf2::Quaternion getDepthToBaseRotationCorrection();
    tf2::Vector3 getDepthToBaseTranslationCorrection();

    tf2_ros::StaticTransformBroadcaster m_staticBroadcaster;
};

#endif