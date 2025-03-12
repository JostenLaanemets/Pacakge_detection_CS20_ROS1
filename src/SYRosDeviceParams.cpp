#include "synexens_ros1/SYRosDeviceParmas.h"

void SYRosDeviceParmas::Help()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  ROS_INFO("#param_variable - #param_type : param_help_string (#param_default_val)");
  ROS_PARAM_LIST
#undef LIST_ENTRY
}

void SYRosDeviceParmas::Print()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);
  ROS_PARAM_LIST
#undef LIST_ENTRY
}

void SYRosDeviceParmas::GetCameraConfig(SYCameraConfig *cameraConfig)
{
  // CS30 RGB
  if (CS30_color_resolution == "1080P")
  {
    cameraConfig->CS30RGBResolution = Synexens::SYRESOLUTION_1920_1080;
  }
  else if (CS30_color_resolution == "540P")
  {
    cameraConfig->CS30RGBResolution = Synexens::SYRESOLUTION_960_540;
  }
  else
  {
    cameraConfig->CS30RGBResolution = Synexens::SYRESOLUTION_1920_1080;
  }
  // CS30 Depth
  if (CS30_depth_resolution == "240P")
  {
    cameraConfig->CS30DepthResolution = Synexens::SYRESOLUTION_320_240;
  }
  else if (CS30_depth_resolution == "480P")
  {
    cameraConfig->CS30DepthResolution = Synexens::SYRESOLUTION_640_480;
  }
  else
  {
    cameraConfig->CS30DepthResolution = Synexens::SYRESOLUTION_320_240;
  }
  
  // CS30 stream type
  if (CS30_color_enabled && CS30_depth_enabled && CS30_ir_enabled)
  {
    cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTHIRRGB;
  }
  else if (CS30_depth_enabled && CS30_ir_enabled)
  {
    cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }
  else if (CS30_depth_enabled && CS30_color_enabled)
  {
    cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTHRGB;
  }
  else if (CS30_depth_enabled)
  {
    cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTH;
  }

  if(CS30_mapping_enabled)
  {
    cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_RGBD;
  }

  // CS20 depth
  if (CS20_depth_resolution == "240P")
  {
    cameraConfig->CS20DepthResolution = Synexens::SYRESOLUTION_320_240;
  }
  else if (CS20_depth_resolution == "480P")
  {
    cameraConfig->CS20DepthResolution = Synexens::SYRESOLUTION_640_480;
  }
  else
  {
    cameraConfig->CS20DepthResolution = Synexens::SYRESOLUTION_320_240;
  }

  // CS20 stream type
  if (CS20_depth_enabled && CS20_ir_enabled)
  {
    cameraConfig->CS20StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }
  else if (CS20_depth_enabled)
  {
    cameraConfig->CS20StreamType = Synexens::SYSTREAMTYPE_DEPTH;
  }
  else
  {
    cameraConfig->CS20StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }

  // CS20P depth
  if (CS20P_depth_resolution == "240P")
  {
    cameraConfig->CS20PDepthResolution = Synexens::SYRESOLUTION_320_240;
  }
  else
  {
    cameraConfig->CS20PDepthResolution = Synexens::SYRESOLUTION_320_240;
  }

  // CS20P stream type
  if (CS20P_depth_enabled && CS20P_ir_enabled)
  {
    cameraConfig->CS20PStreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }
  else if (CS20P_depth_enabled)
  {
    cameraConfig->CS20PStreamType = Synexens::SYSTREAMTYPE_DEPTH;
  }
  else
  {
    cameraConfig->CS20PStreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }

  // CS40 depth
  if (CS40_depth_resolution == "480P")
  {
    cameraConfig->CS40DepthResolution = Synexens::SYRESOLUTION_640_480;
  }
  else
  {
    cameraConfig->CS40DepthResolution = Synexens::SYRESOLUTION_640_480;
  }

  // CS40 stream type
  if (CS40_depth_enabled && CS40_ir_enabled)
  {
    cameraConfig->CS40StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }
  else if (CS40_depth_enabled)
  {
    cameraConfig->CS40StreamType = Synexens::SYSTREAMTYPE_DEPTH;
  }
  else
  {
    cameraConfig->CS40StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
  }
}