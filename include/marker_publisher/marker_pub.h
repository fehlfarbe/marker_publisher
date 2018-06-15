#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <aruco.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <marker_publisher/MarkerArray.h>
#include <tf/transform_listener.h>

class MarkerPosePublisher {
    aruco::CameraParameters TheCameraParameters;
    aruco::MarkerDetector TheMarkerDetector;
    std::string dict_type;
    float markerSizeMin;
    std::string detectionMode;
    aruco::DetectionMode detectionModeEnum;
    float markerSizeMeters;
    float marker_id_temp = -1;
    std::string TheCameraParameters_path;
    bool useCamInfo;
    bool useRectifiedImages;
    std::string camera_frame;
    bool invertImage;

public:
    MarkerPosePublisher();

    void callBackColor(const sensor_msgs::ImageConstPtr &);

    tf::Transform arucoMarker2Tf(const aruco::Marker &);

    void publish_marker(geometry_msgs::Pose, int);

    aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                         bool useRectifiedParameters);

    bool hasSubscribers();

protected:
    ros::NodeHandle nh_node;
    ros::Subscriber sub;
    ros::Publisher markers_pub_tf;
    ros::Publisher markers_pub_array;
    ros::Publisher markers_pub_debug;
    marker_publisher::MarkerArray::Ptr marker_msg_pub;
};

