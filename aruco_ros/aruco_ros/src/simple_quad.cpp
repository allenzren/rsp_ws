#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

using namespace cv;
using namespace aruco;

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
MarkerDetector mDetector;
vector<Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;

ros::Publisher pose_pub1;
ros::Publisher pose_pub2;
ros::Publisher pose_pub3;
ros::Publisher pose_pub4;

std::string parent_name;
std::string child_name1;
std::string child_name2;
std::string child_name3;
std::string child_name4;

double marker_size;
int marker_id1;
int marker_id2;
int marker_id3;
int marker_id4;

tf::Transform cam_cub1_tf;
tf::Transform cam_cub2_tf;
tf::Transform cam_cub3_tf;
tf::Transform cam_dest_tf;

tf::Transform base_cub1_tf;
tf::Transform base_cub2_tf;
tf::Transform base_cub3_tf;
tf::Transform base_dest_tf;

tf::StampedTransform tf_base_camera;

// void image_callback(const sensor_msgs::ImageConstPtr& msg)
void image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg_ptr)
{
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;
  if(cam_info_received)
  {
    // ros::Time curr_stamp(ros::Time::now());
    ros::Time curr_stamp = info_msg_ptr->header.stamp;

    static tf::TransformListener listener;
    tf::StampedTransform transform;

    try{
      listener.waitForTransform("/base_link", "/camera_rgb_optical_frame", curr_stamp, ros::Duration(0.1));
      listener.lookupTransform("/base_link", "camera_rgb_optical_frame", curr_stamp, transform);
    }

    catch(tf::TransformException &ex) {
      ROS_INFO("No base to camera tf.");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      if(normalizeImageIllumination)
      {
        ROS_WARN("normalizeImageIllumination is unimplemented!");
        //cv::Mat inImageNorm;
        //pal_vision_util::dctNormalization(inImage, inImageNorm, dctComponentsToRemove);
        //inImage = inImageNorm;
      }

      //detection results will go into "markers"
      markers.clear();
      //Ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size);
      //for each marker, draw info and its boundaries in the image
      for(unsigned int i=0; i<markers.size(); ++i)
      {
        // only publishing the selected marker
        if ( markers[i].id == marker_id1 )
        {
          cam_cub1_tf = aruco_ros::arucoMarker2Tf(markers[i]);

          tf::Transform tmp_tf = transform*cam_cub1_tf;
          tf::Quaternion tmp_quat = tmp_tf.getRotation();
          double tmp = 1.0 - 2*tmp_quat.x()*tmp_quat.x() - 2*tmp_quat.y()*tmp_quat.y();
          if(tmp >= 0.995 && 
            ((tmp_tf.getOrigin() - base_cub1_tf.getOrigin()).length() < 0.005 ||
              base_cub1_tf.getOrigin().y() == 0)) base_cub1_tf = transform*cam_cub1_tf;
        }
        else if ( markers[i].id == marker_id2 )
        {
          cam_cub2_tf = aruco_ros::arucoMarker2Tf(markers[i]);

          tf::Transform tmp_tf = transform*cam_cub2_tf;
          tf::Quaternion tmp_quat = tmp_tf.getRotation();
          double tmp = 1.0 - 2*tmp_quat.x()*tmp_quat.x() - 2*tmp_quat.y()*tmp_quat.y();
          if(tmp >= 0.9995)  base_cub2_tf = transform*cam_cub2_tf;
        }
        else if ( markers[i].id == marker_id3 )
        {
          cam_cub3_tf = aruco_ros::arucoMarker2Tf(markers[i]);

          tf::Transform tmp_tf = transform*cam_cub3_tf;
          tf::Quaternion tmp_quat = tmp_tf.getRotation();
          double tmp = 1.0 - 2*tmp_quat.x()*tmp_quat.x() - 2*tmp_quat.y()*tmp_quat.y();
          if(tmp >= 0.9995) base_cub3_tf = transform*cam_cub3_tf;
        }
        else if ( markers[i].id == marker_id4 )
        {
          cam_dest_tf = aruco_ros::arucoMarker2Tf(markers[i]);

          tf::Transform tmp_tf = transform*cam_dest_tf;
          tf::Quaternion tmp_quat = tmp_tf.getRotation();
          double tmp = 1.0 - 2*tmp_quat.x()*tmp_quat.x() - 2*tmp_quat.y()*tmp_quat.y();
          if(tmp >= 0.995 && 
            ((tmp_tf.getOrigin() - base_dest_tf.getOrigin()).length() < 0.005 ||
              base_dest_tf.getOrigin().y() == 0)) base_dest_tf = transform*cam_dest_tf;
        }

        // but drawing all the detected markers
        markers[i].draw(inImage,Scalar(0,0,255),2);
      }

      if(base_cub1_tf.getOrigin().x() != 0)
      {
        br.sendTransform(tf::StampedTransform(base_cub1_tf, curr_stamp,
                                              parent_name, child_name1));
      }

      if(base_cub2_tf.getOrigin().x() != 0)
      {
      br.sendTransform(tf::StampedTransform(base_cub2_tf, curr_stamp,
                                              parent_name, child_name2));
      }

      if(base_cub3_tf.getOrigin().x() != 0)
      {
      br.sendTransform(tf::StampedTransform(base_cub3_tf, curr_stamp,
                                              parent_name, child_name3));
      }

      if(base_dest_tf.getOrigin().x() != 0)
      {
      br.sendTransform(tf::StampedTransform(base_dest_tf, curr_stamp,
                                              parent_name, child_name4));
      }

      //paint a circle in the center of the image
      cv::circle(inImage, cv::Point(inImage.cols/2, inImage.rows/2), 4, cv::Scalar(0,255,0), 1);

      //draw a 3d cube in each marker if there is 3d info
      if(camParam.isValid() && marker_size!=-1)
      {
        for(unsigned int i=0; i<markers.size(); ++i)
        {
          CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
        }
      }

      if(image_pub.getNumSubscribers() > 0)
      {
        //show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if(debug_pub.getNumSubscribers() > 0)
      {
        //show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }

      ROS_DEBUG("runtime: %f ms",
                1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
{
  mDetector.setThresholdParams(config.param1,config.param2);
  normalizeImageIllumination = config.normalizeImage;
  dctComponentsToRemove      = config.dctComponentsToRemove;
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
  f_ = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(f_);

  normalizeImageIllumination = false;

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

  // image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  image_transport::CameraSubscriber image_sub = it.subscribeCamera("/image", 1, &image_callback);
  cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

  cam_info_received = false;
  image_pub = it.advertise("result", 1);
  debug_pub = it.advertise("debug", 1);

  nh.param<double>("marker_size", marker_size, 0.03);

  nh.param<int>("marker_id1", marker_id1, 1);
  nh.param<int>("marker_id2", marker_id2, 2);
  nh.param<int>("marker_id3", marker_id3, 3);
  nh.param<int>("marker_id4", marker_id4, 4);

  nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
  nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if(dctComponentsToRemove == 0)
    normalizeImageIllumination = false;
  nh.param<std::string>("parent_name", parent_name, "");
  nh.param<std::string>("child_name1", child_name1, "");
  nh.param<std::string>("child_name2", child_name2, "");
  nh.param<std::string>("child_name3", child_name3, "");
  nh.param<std::string>("child_name4", child_name4, "");


  if(parent_name == "" || child_name1 == "" || child_name2 == "" || child_name3 == "" || child_name4 == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }

  ROS_INFO("Aruco node started with marker size of %f meters and marker ids to track: %d, %d, %d",
           marker_size, marker_id1, marker_id2, marker_id3);
  ROS_INFO("Aruco node will publish pose to TF with (%s, %s, %s, %s) and (%s, %s, %s, %s) as (parent,child).",
           parent_name.c_str(), child_name1.c_str(), parent_name.c_str(), child_name2.c_str(),
           parent_name.c_str(), child_name3.c_str(), parent_name.c_str(), child_name4.c_str());

  ros::spin();
}
