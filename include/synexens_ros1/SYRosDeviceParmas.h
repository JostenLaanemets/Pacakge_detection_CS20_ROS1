#ifndef SYRosDeviceParmas_H
#define SYRosDeviceParmas_H
#include <string>
#include <ros/ros.h>
#include "SYRosTypes.h"
#define ROS_PARAM_LIST                                                                                                                                                 \
    LIST_ENTRY(tf_prefix, "Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.", std::string, std::string())                              \
    LIST_ENTRY(fps, "The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30", int, 7)                                                                         \
    LIST_ENTRY(point_cloud_enabled, "Generate a point cloud from depth data. Requires depth_enabled", bool, true)                                                      \
    LIST_ENTRY(CS30_depth_enabled, "True if depth stream should be enabled", bool, true)                                                                               \
    LIST_ENTRY(CS30_depth_resolution, "The resolution of the depth frame. Options are: 240P, 480P", std::string, std::string("480P"))                                  \
    LIST_ENTRY(CS30_ir_enabled, "Enable or disable the ir camera", bool, true)                                                                                         \
    LIST_ENTRY(CS30_color_enabled, "Enable or disable the color camera", bool, true)                                                                                   \
    LIST_ENTRY(CS30_color_resolution, "Resolution at which to run the color camera. Valid options: 1080P 540P 480P", std::string, std::string("540P"))                      \
    LIST_ENTRY(CS30_mapping_enabled, "True if mapped depth in color space should be enabled mapping depth resolution 480P rgb resolution 1080P 540P", bool, false)     \
    LIST_ENTRY(CS30_exposure, "The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000", int, 3000)                               \
    LIST_ENTRY(CS30_exposure_range_min, "The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0", int, 0)           \
    LIST_ENTRY(CS30_exposure_range_max, "The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000", int, 3000)     \
    LIST_ENTRY(CS30_depth_image_filter, "0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1", int, -1)                                      \
    LIST_ENTRY(CS30_filter_amplititud_value, "AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6", float, -1.0f)   \
    LIST_ENTRY(CS30_filter_median_value_01, "MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3", float, -1.0f)        \
    LIST_ENTRY(CS30_filter_median_value_02, "MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)        \
    LIST_ENTRY(CS30_filter_gauss_value_01, "GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1", float, -1.0f)          \
    LIST_ENTRY(CS30_filter_gauss_value_02, "GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)          \
    LIST_ENTRY(CS30_filter_edge_value, "EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50", float, -1.0f)              \
    LIST_ENTRY(CS30_filter_speckle_value_01, "SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40", float, -1.0f)  \
    LIST_ENTRY(CS30_filter_speckle_value_02, "SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200", float, -1.0f)                      \
    LIST_ENTRY(CS30_filter_sobel_value, "SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150", float, -1.0f)           \
    LIST_ENTRY(CS30_filter_edge_mad_value, "EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15", float, -1.0f)       \
    LIST_ENTRY(CS30_filter_okada_value, "OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10", float, -1.0f)            \
    LIST_ENTRY(CS20_depth_enabled, "True if depth stream should be enabled", bool, true)                                                                               \
    LIST_ENTRY(CS20_depth_resolution, "The resolution of the depth frame. Options are: 240P, 480P. SINGLE not supported 480P", std::string, std::string("240P"))       \
    LIST_ENTRY(CS20_ir_enabled, "Enable or disable the ir camera", bool, true)                                                                                         \
    LIST_ENTRY(CS20_exposure, "The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000", int, 3000)                               \
    LIST_ENTRY(CS20_exposure_range_min, "The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0", int, 0)           \
    LIST_ENTRY(CS20_exposure_range_max, "The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000", int, 3000)     \
    LIST_ENTRY(CS20_depth_image_filter, "0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1", int, -1)                                      \
    LIST_ENTRY(CS20_filter_amplititud_value, "AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6", float, -1.0f)   \
    LIST_ENTRY(CS20_filter_median_value_01, "MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3", float, -1.0f)        \
    LIST_ENTRY(CS20_filter_median_value_02, "MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)        \
    LIST_ENTRY(CS20_filter_gauss_value_01, "GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1", float, -1.0f)          \
    LIST_ENTRY(CS20_filter_gauss_value_02, "GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)          \
    LIST_ENTRY(CS20_filter_edge_value, "EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50", float, -1.0f)              \
    LIST_ENTRY(CS20_filter_speckle_value_01, "SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40", float, -1.0f)  \
    LIST_ENTRY(CS20_filter_speckle_value_02, "SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200", float, -1.0f)                      \
    LIST_ENTRY(CS20_filter_sobel_value, "SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150", float, -1.0f)           \
    LIST_ENTRY(CS20_filter_edge_mad_value, "EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15", float, -1.0f)       \
    LIST_ENTRY(CS20_filter_okada_value, "OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10", float, -1.0f)            \
    LIST_ENTRY(CS20P_depth_enabled, "True if depth stream should be enabled", bool, true)                                                                              \
    LIST_ENTRY(CS20P_depth_resolution, "The resolution of the depth frame. Options are: 240P", std::string, std::string("240P"))                                       \
    LIST_ENTRY(CS20P_ir_enabled, "Enable or disable the ir camera", bool, true)                                                                                        \
    LIST_ENTRY(CS20P_exposure, "The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000", int, 3000)                              \
    LIST_ENTRY(CS20P_exposure_range_min, "The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0", int, 0)          \
    LIST_ENTRY(CS20P_exposure_range_max, "The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000", int, 3000)    \
    LIST_ENTRY(CS20P_depth_image_filter, "0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1", int, -1)                                     \
    LIST_ENTRY(CS20P_filter_amplititud_value, "AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6", float, -1.0f)  \
    LIST_ENTRY(CS20P_filter_median_value_01, "MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3", float, -1.0f)       \
    LIST_ENTRY(CS20P_filter_median_value_02, "MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)       \
    LIST_ENTRY(CS20P_filter_gauss_value_01, "GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1", float, -1.0f)         \
    LIST_ENTRY(CS20P_filter_gauss_value_02, "GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)         \
    LIST_ENTRY(CS20P_filter_edge_value, "EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50", float, -1.0f)             \
    LIST_ENTRY(CS20P_filter_speckle_value_01, "SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40", float, -1.0f) \
    LIST_ENTRY(CS20P_filter_speckle_value_02, "SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200", float, -1.0f)                     \
    LIST_ENTRY(CS20P_filter_sobel_value, "SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150", float, -1.0f)          \
    LIST_ENTRY(CS20P_filter_edge_mad_value, "EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15", float, -1.0f)      \
    LIST_ENTRY(CS20P_filter_okada_value, "OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10", float, -1.0f)           \
    LIST_ENTRY(CS40_depth_enabled, "True if depth stream should be enabled", bool, true)                                                                               \
    LIST_ENTRY(CS40_depth_resolution, "The resolution of the depth frame. Options are: 480P", std::string, std::string("480P"))                                        \
    LIST_ENTRY(CS40_ir_enabled, "Enable or disable the ir camera", bool, true)                                                                                         \
    LIST_ENTRY(CS40_exposure, "The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=1510", int, 1510)                               \
    LIST_ENTRY(CS40_exposure_range_min, "The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0", int, 0)           \
    LIST_ENTRY(CS40_exposure_range_max, "The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000", int, 3000)     \
    LIST_ENTRY(CS40_depth_image_filter, "0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1", int, -1)                                      \
    LIST_ENTRY(CS40_filter_amplititud_value, "AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6", float, -1.0f)   \
    LIST_ENTRY(CS40_filter_median_value_01, "MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3", float, -1.0f)        \
    LIST_ENTRY(CS40_filter_median_value_02, "MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)        \
    LIST_ENTRY(CS40_filter_gauss_value_01, "GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1", float, -1.0f)          \
    LIST_ENTRY(CS40_filter_gauss_value_02, "GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1", float, -1.0f)          \
    LIST_ENTRY(CS40_filter_edge_value, "EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50", float, -1.0f)              \
    LIST_ENTRY(CS40_filter_speckle_value_01, "SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40", float, -1.0f)  \
    LIST_ENTRY(CS40_filter_speckle_value_02, "SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200", float, -1.0f)                      \
    LIST_ENTRY(CS40_filter_sobel_value, "SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150", float, -1.0f)           \
    LIST_ENTRY(CS40_filter_edge_mad_value, "EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15", float, -1.0f)       \
    LIST_ENTRY(CS40_filter_okada_value, "OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10", float, -1.0f)

class SYRosDeviceParmas
{
public:
    // get camera config
    void GetCameraConfig(SYCameraConfig *cameraConfig);
    // Print help messages to the console
    void Help();

    // Print the value of all parameters
    void Print();
// Parameters
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
    ROS_PARAM_LIST
#undef LIST_ENTRY
};

#endif // SYRosDeviceParmas_H