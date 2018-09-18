#include "marker_publisher/marker_pub.h"

MarkerPosePublisher::MarkerPosePublisher() : nh_node("~") {
    nh_node.param<std::string>("dict_type", dict_type, "ALL_DICTS");
    TheMarkerDetector.setDictionary(dict_type,
                                    0.6f); //errorCorrectionRate error correction rate respect to the maximun error correction capability for each dictionary. (default 0.6).

    // set detection mode and minimum marker size
    nh_node.param<float>("marker_min", markerSizeMin, 0);
    nh_node.param<std::string>("detection_mode", detectionMode, "DM_NORMAL");
    aruco::MarkerDetector::Params p;
    p.setCornerRefinementMethod(p.getCornerRefinementMethodFromString(detectionMode));
    p.setCornerRefinementMethod(aruco::CORNER_SUBPIX);
    //p.detectEnclosedMarkers(true); // doesn't detect anything
    TheMarkerDetector.setParameters(p);

    nh_node.param<std::string>("camera_frame", camera_frame, "");

    // load camera parameters from camerainfo or YAML file
    nh_node.param<bool>("use_camera_info", useCamInfo, true);
    if(useCamInfo)
    {
        sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", nh_node);//, 10.0);
        nh_node.param<bool>("image_is_rectified", useRectifiedImages, true);
        TheCameraParameters = rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages);
    }
    else {
        try {
            nh_node.param<std::string>("TheCameraParameters_path", TheCameraParameters_path, "");
            TheCameraParameters.readFromXMLFile(TheCameraParameters_path);
        } catch (cv::Exception&) {
            ROS_ERROR("Cannot load camera parameters!");
        }
    }

    nh_node.param<float>("markerSizeMeters", markerSizeMeters, -1);
    nh_node.param<bool>("invert_image", invertImage, false);
    nh_node.param<bool>("equalize", equalize, true);


    sub = nh_node.subscribe("image_raw", 1, &MarkerPosePublisher::callBackColor, this);

    markers_pub_tf = nh_node.advertise<visualization_msgs::Marker>("Estimated_marker", 1);
    markers_pub_array = nh_node.advertise<marker_publisher::MarkerArray>("MarkerArray", 1);
    markers_pub_debug = nh_node.advertise<sensor_msgs::Image>("debug", 1);

    marker_msg_pub = marker_publisher::MarkerArray::Ptr(new marker_publisher::MarkerArray());
    //The same frame_id & seq for every marker in each frame
    marker_msg_pub->header.frame_id = camera_frame;
    marker_msg_pub->header.seq = 0;
}

void MarkerPosePublisher::callBackColor(const sensor_msgs::ImageConstPtr &msg) {
    if(!hasSubscribers()){
        return;
    }

    cv_bridge::CvImagePtr cv_ptr, cv_preview;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_preview = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(invertImage){
        bitwise_not ( cv_ptr->image, cv_ptr->image );
    }

    if(equalize){
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);
        cv::cvtColor(gray, cv_ptr->image, cv::COLOR_GRAY2BGR);
    }

    cv_preview->image = cv_ptr->image;

    ros::Time curr_stamp(ros::Time::now());

    static tf::TransformBroadcaster br;
    std::vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(
            cv_ptr->image); //, TheCameraParameters, markerSizeMeters);

    marker_msg_pub->markers.clear();
    marker_msg_pub->markers.resize(detected_markers.size());
    marker_msg_pub->header.stamp = curr_stamp;
    marker_msg_pub->header.seq++;

    for (size_t i = 0; i < detected_markers.size(); ++i) {
        marker_publisher::Marker &marker_i = marker_msg_pub->markers.at(i);
        marker_i.idx = detected_markers[i].id;
    }


    tf::StampedTransform cameraToReference;
    cameraToReference.setIdentity();

    for (size_t i = 0; i < detected_markers.size(); ++i) {
        std::ostringstream o;
        auto markerId = detected_markers[i].id;
        o << "marker_" << markerId;
        std::string o_str = o.str();

        marker_id_temp = -1.0f;
        nh_node.param<float>(o_str, marker_id_temp, -1);

        if (marker_id_temp != -1)
            detected_markers[i].calculateExtrinsics(marker_id_temp, TheCameraParameters.CameraMatrix,
                                                    TheCameraParameters.Distorsion, false);
        else
            detected_markers[i].calculateExtrinsics(markerSizeMeters, TheCameraParameters.CameraMatrix,
                                                    TheCameraParameters.Distorsion, false);

        detected_markers[i].draw(cv_preview->image, cv::Scalar(0, 0, 255), 3, true, true);
//        aruco::CvDrawingUtils::draw3dCube(cv_ptr->image, detected_markers[i], TheCameraParameters);
        aruco::CvDrawingUtils::draw3dAxis(cv_preview->image, detected_markers[i], TheCameraParameters);

        tf::Transform object_transform = arucoMarker2Tf(detected_markers[i]);

        br.sendTransform(tf::StampedTransform(object_transform, ros::Time::now(), camera_frame, o_str));

        geometry_msgs::Pose marker_pose_data;

        const tf::Vector3 marker_origin = object_transform.getOrigin();
        marker_pose_data.position.x = marker_origin.getX();
        marker_pose_data.position.y = marker_origin.getY();
        marker_pose_data.position.z = marker_origin.getZ();

        tf::Quaternion marker_quaternion = object_transform.getRotation();
        marker_pose_data.orientation.x = marker_quaternion.getX();
        marker_pose_data.orientation.y = marker_quaternion.getY();
        marker_pose_data.orientation.z = marker_quaternion.getZ();
        marker_pose_data.orientation.w = marker_quaternion.getW();

        publish_marker(marker_pose_data, markerId);

        //Publish markers
        marker_publisher::Marker &marker_i = marker_msg_pub->markers.at(i);
        tf::Transform transform = arucoMarker2Tf(detected_markers[i]);
        tf::poseTFToMsg(transform, marker_i.pose.pose);
    }

    // only publish if markers detected
    if(!detected_markers.empty()) {
        markers_pub_array.publish(marker_msg_pub);
    }

    // publish debug image with markers
    sensor_msgs::ImagePtr debug;
    try {
        debug = cv_preview->toImageMsg();
    }

    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    debug->header = marker_msg_pub->header;
    markers_pub_debug.publish(debug);
}


tf::Transform MarkerPosePublisher::arucoMarker2Tf(const aruco::Marker &marker) {
    cv::Mat marker_rotation(3, 3, CV_32FC1);
    cv::Rodrigues(marker.Rvec, marker_rotation);
    cv::Mat marker_translation = marker.Tvec;

    tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0, 0), marker_rotation.at<float>(0, 1),
                                marker_rotation.at<float>(0, 2),
                                marker_rotation.at<float>(1, 0), marker_rotation.at<float>(1, 1),
                                marker_rotation.at<float>(1, 2),
                                marker_rotation.at<float>(2, 0), marker_rotation.at<float>(2, 1),
                                marker_rotation.at<float>(2, 2));

    tf::Vector3 marker_tf_tran(marker_translation.at<float>(0, 0),
                               marker_translation.at<float>(1, 0),
                               marker_translation.at<float>(2, 0));

    return tf::Transform(marker_tf_rot, marker_tf_tran);
}

void MarkerPosePublisher::publish_marker(geometry_msgs::Pose marker_pose, int marker_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = camera_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = marker_pose;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.01;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.1);

    markers_pub_tf.publish(marker);
}

aruco::CameraParameters MarkerPosePublisher::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                bool useRectifiedParameters)
{
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if ( useRectifiedParameters )
    {
        cameraMatrix.setTo(0);
        cameraMatrix.at<double>(0,0) = cam_info.P[0];   cameraMatrix.at<double>(0,1) = cam_info.P[1];   cameraMatrix.at<double>(0,2) = cam_info.P[2];
        cameraMatrix.at<double>(1,0) = cam_info.P[4];   cameraMatrix.at<double>(1,1) = cam_info.P[5];   cameraMatrix.at<double>(1,2) = cam_info.P[6];
        cameraMatrix.at<double>(2,0) = cam_info.P[8];   cameraMatrix.at<double>(2,1) = cam_info.P[9];   cameraMatrix.at<double>(2,2) = cam_info.P[10];

        for(int i=0; i<4; ++i)
            distorsionCoeff.at<double>(i, 0) = 0;
    }
    else
    {
        for(int i=0; i<9; ++i)
            cameraMatrix.at<double>(i%3, i-(i%3)*3) = cam_info.K[i];

        if(cam_info.D.size() >= 4)
        {
            for(int i=0; i<4; ++i)
                distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
        }
        else
        {
            ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...\n"
                     "distortion model of calibration file is %s", cam_info.distortion_model.c_str());

            for(int i=0; i<4; ++i)
                distorsionCoeff.at<double>(i, 0) = 0;
        }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

bool MarkerPosePublisher::hasSubscribers(){
    return markers_pub_array.getNumSubscribers() > 0 || markers_pub_debug.getNumSubscribers() > 0;
}





















