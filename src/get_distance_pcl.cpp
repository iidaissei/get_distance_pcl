// ROS core
#include <ros/ros.h>
//ros msg
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
//Image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//stl stuff
#include <string>
//opencv
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//original msg
#include <get_distance_pcl/Coordinate_xyz.h>
//tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
//nav_msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
class PointCloudToImage
{
public:
  get_distance_pcl::Coordinate_xyz getCoordinate(int pixel_width,int pixel_height){
    get_distance_pcl::Coordinate_xyz c;
    c.frame_id = "camrera_point_frame";
    uint8_t r = 0, g = 0, b = 0;    //  Black color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    if(point_cloud_.height == 0 && point_cloud_.width == 0){
      std::cout<<"cloud is empty!"<<std::endl;
      c.x = 999.0;
      c.y = 999.0;
      c.z = 999.0;
      c.world_x = 999.0;
      c.world_y = 999.0;
      c.world_z = 999.0;
      //c.xy_angle_radian = 999.0;
      c.data_enable = false;
      return c;
    }
    if(debag == true){
      std::cout<<"point_cloud.height :"<<point_cloud_.height<<std::endl;
      std::cout<<"point_cloud.width  :"<<point_cloud_.width<<std::endl;
    }
    image_.height = 480;//めんどくさいので固定
    image_.width = 640;//これもだるいので固定 エラー吐くかも
    image_.encoding = "bgr8";
    image_.step = image_.width * sizeof(uint8_t)*3;
    image_.data.resize(image_.step * image_.height);
    c.x = point_cloud_(pixel_width,pixel_height).z;
    c.y = point_cloud_(pixel_width,pixel_height).x;
    c.z = point_cloud_(pixel_width,pixel_height).y;
    c.data_enable = true;
    if(debag ==true){
      std::cout<<"cloud_("<<pixel_width<<","<<pixel_height<<").x  :"<<point_cloud_(pixel_width,pixel_height).x<<std::endl;
      std::cout<<"cloud_("<<pixel_width<<","<<pixel_height<<").y  :"<<point_cloud_(pixel_width,pixel_height).y<<std::endl;
   std::cout<<"cloud_("<<pixel_width<<","<<pixel_height<<").z  :"<<point_cloud_(pixel_width,pixel_height).z<<std::endl;
      std::cout<<"(x,y,z) = ("<<c.x<<","<<c.y<<","<<c.z<<")"<<std::endl;
    }
    //using tf to create world coordinate xyz
    /*https://ppdr.softether.net/ros-pointcloud2-transform*/
    
    for(size_t y = 0; y < image_.height; y++){
      for(size_t x = 0; x < image_.width; x++){
	uint8_t* pixel = &(image_.data[y * image_.step + x * 3]);
	if(point_cloud_(x, y).z < 5.0){
	  memcpy(pixel, &point_cloud_(x, y).rgb, 3 * sizeof(uint8_t));
	}
	else{
	  memcpy(pixel, &rgb, 3 * sizeof(uint8_t));
	}
      }
    }
    showImage(pixel_width,pixel_height);// default 
    return c;
  }

  void showImage(int pixel_width_,int pixel_height_){
    cv_bridge::CvImagePtr img_ptr;
    cv::Mat image_rgb;
    int minR = 0;
    int minG = 0;
    int minB = 0;
    int maxR = 255;
    int maxG = 255;
    int maxB = 255;
    
    try
      {
	img_ptr = cv_bridge::toCvCopy(image_,sensor_msgs::image_encodings::BGR8);
	image_rgb = img_ptr->image;
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: \%s", e.what());
	return;
      }
    cv::Mat img_binary;
    CvMoments colorMoment;
    cv::Scalar min_vals(minR, minG, minB);
    cv::Scalar max_vals(maxR, maxG, maxB);
    //cv::cvtColor(img_rgb, img_hsv, CV_BGR2HSV);
    cv::inRange(image_rgb, min_vals, max_vals, img_binary);
    dilate( img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1)) );
    colorMoment = cv::moments(img_binary);
    double moment10 = cvGetSpatialMoment(&colorMoment, 1, 0);
    double moment01 = cvGetSpatialMoment(&colorMoment, 0, 1);
    double area = cvGetCentralMoment(&colorMoment, 0, 0);
    float posX = (moment10/area);
    float posY = (moment01/area);
    //printf("1. x-Axis  %f  y-Axis  %f  Area  %f\n", moment10, moment01, area);
    //printf("2. x  %f  y %f \n\n", posX , posY);
    
    //cv::imshow("TRACKING COLOR", img_binary);

    cv::line(image_rgb,cv::Point(0,pixel_height_),cv::Point(639,pixel_height_),2,CV_AA);
    cv::line(image_rgb,cv::Point(pixel_width_,0),cv::Point(pixel_width_,479),2,CV_AA);
    cv::imshow("RGB image", image_rgb);
    cv::waitKey(3);
    
  }
  get_distance_pcl::Coordinate_xyz getWorldCoordinate(get_distance_pcl::Coordinate_xyz c_){
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform transform;
    //tf::Transform transform;
    tf::Transform bro_transform;//for broadcast
    tf::Quaternion q;
    geometry_msgs::PoseStamped source_pose;
    geometry_msgs::PoseStamped target_pose;
    
    double roll_radian,pitch_radian,yaw_radian;
    double roll_deg,pitch_deg,yaw_deg;
    double qx,qy,qz,qw;
    
    //fake pose
    odom.child_frame_id = "object_position_frame";
    odom.pose.pose.position.x = c_.x;
    odom.pose.pose.position.y = c_.y*-1;
    odom.pose.pose.position.z = c_.z;
    std::cout<<"tan is "<<atan(c_.y/c_.x)<<std::endl;
    EulerAnglesToQuaternion(0.0,0.0,atan(c_.y/c_.x),qx,qy,qz,qw);
    //EulerAnglesToQuaternion(0.0,atan(c_.y/c_.x),0.0,qx,qy,qz,qw);
    
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy; 
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom_pub_.publish(odom);
    //EulerAnglesToQuaternion(0.0,0,0,atan(c_y/c_x),qx,qy,qz,qw);
    
    for(int i;i<5;i++){
      broadcaster.sendTransform(tf::StampedTransform(bro_transform, ros::Time(0), "/camera_rgb_frame", "/object_position_frame"));
      
      try{
	listener.waitForTransform("/map","/object_position_frame",ros::Time(0),ros::Duration(3.0));//default /map & /camera_rgb_frame
	//listener.transformPose("/object_position_frame",source_pose,target_pose);
	listener.lookupTransform("/map", "/object_position_frame",ros::Time(0), transform);
      }
      catch(...){
	ROS_INFO("time error");
      }
    }
    if(transform.getOrigin().getX()<0.0001 && 0 < transform.getOrigin().getX()){
      c_.data_enable = false;
    }
    c_.world_x = transform.getOrigin().getX();
    c_.world_y = transform.getOrigin().getY();
    c_.world_z = transform.getOrigin().getZ();
    c_.quaternion_x = transform.getRotation().getX();
    c_.quaternion_y = transform.getRotation().getY();
    c_.quaternion_z = transform.getRotation().getZ();
    c_.quaternion_w = transform.getRotation().getW();
    
    c_.xy_angle_radian = 0.0;
    c_.xy_angle_deg = 0.0;
    if(debag == true){
    std::cout<<"roll       :"<<roll_radian
	     <<"\npitch      :"<<pitch_radian
             <<"\nyaw        :"<<yaw_radian
	     <<"\nroll_deg   :"<<roll_deg
	     <<"\npitch_deg  :"<<pitch_deg
	     <<"\nyaw_deg    :"<<yaw_deg
	     <<"\ncos(pitch) :"<<cos(pitch_radian)
	     <<"\nsin(pitch) :"<<sin(pitch_radian)
	     <<"\norigin_x   :"<<transform.getOrigin().x()
	     <<"\norigin_y   :"<<transform.getOrigin().y()
	     <<"\norigin_z   :"<<transform.getOrigin().z()
	     <<"\norigin_w   :"<<transform.getOrigin().w()<<std::endl;
    }
    return c_;
  }
  void EulerAnglesToQuaternion(double roll, double pitch, double yaw,
			       double& q0, double& q1, double& q2, double& q3)
  {
    double cosRoll = cos(roll / 2.0);
    double sinRoll = sin(roll / 2.0);
    double cosPitch = cos(pitch / 2.0);
    double sinPitch = sin(pitch / 2.0);
    double cosYaw = cos(yaw / 2.0);
    double sinYaw = sin(yaw / 2.0);
    
    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  }

  
  void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
        try
	  {
	    pcl::fromROSMsg(*cloud,point_cloud_);
	    //createRangeImage();
	  }
	catch (std::runtime_error e)
	  {
	    ROS_ERROR_STREAM("Error in converting cloud to image message: "
			     << e.what());
	  }
	//image_pub_.publish (image_); //publish our cloud image
  }
  
  void pixel_cb(const std_msgs::Int16MultiArray msg_){
    get_distance_pcl::Coordinate_xyz coordinate;
    coordinate.frame_id ="camera_point_frame";
    int num=msg_.data.size();
    if(msg_.data[0]<0 || 640<msg_.data[0] || msg_.data[1]<0 || 480<msg_.data[1] || num != 2){
      coordinate.x = 999.0;
      coordinate.y = 999.0;
      coordinate.z = 999.0;
      coordinate.world_x = 999.0;
      coordinate.world_y = 999.0;
      coordinate.world_z = 999.0;
      //coordinate.xy_angle = 999.0;
      coordinate.data_enable = false;
    }
    else{
      coordinate = getCoordinate(msg_.data[0],msg_.data[1]);
      if(world_coordinate_enable == true){
	coordinate = getWorldCoordinate(coordinate);
      }
      else{
	coordinate.world_x = 999.0;
	coordinate.world_y = 999.0;
	coordinate.world_z = 999.0;
	coordinate.quaternion_x = 999.0;
	coordinate.quaternion_y = 999.0;
	coordinate.quaternion_z = 999.0;
	coordinate.quaternion_w = 999.0;

	}
    }
    if(debag == true){
      std::cout<<"coordinate.x           "<< coordinate.x 
	       <<"\ncoordinate.y           "<< coordinate.y 
	       <<"\ncoordinate.z           "<< coordinate.z 
	       <<"\ncoordinate.world_x     "<< coordinate.world_x 
	       <<"\ncoordinate.world_y     "<< coordinate.world_y 
	       <<"\ncoordinate.world_z     "<< coordinate.world_z 
	       <<"\ncoordinate.xy_angle    "<< coordinate.xy_angle_deg 
	       <<"\ncoordinate.data_enable "<< (coordinate.data_enable ? true:false) <<std::endl;
    }
    xyz_pub_.publish(coordinate);
  }
  
  PointCloudToImage () : cloud_topic_("/camera/depth_registered/points"),image_topic_("output")
  {
    sub_ = nh_.subscribe (cloud_topic_, 100,&PointCloudToImage::cloud_cb, this);
    pixel_sub_ = nh_.subscribe("pixel_xy",10,&PointCloudToImage::pixel_cb,this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_,30);
    xyz_pub_ = nh_.advertise<get_distance_pcl::Coordinate_xyz>("get_distance_pcl/Coordinate_xyz",10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/get_distance_pcl/fake_odom",10);
    debag = true;
    world_coordinate_enable = true;
    
    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic" << r_it );
  }
private:
  ros::NodeHandle nh_;
  //sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Subscriber pixel_sub_;
  ros::Publisher image_pub_; //image message publisher
  ros::Publisher xyz_pub_;
  ros::Publisher pcl_pub_;//world coordinate for test(for rviz)
  ros::Publisher odom_pub_;
  pcl::PointCloud<pcl::PointXYZRGB> point_cloud_;
  sensor_msgs::Image image_;
  nav_msgs::Odometry odom;
  bool debag;
  bool world_coordinate_enable;
};


int main (int argc, char **argv)
{
  ros::init (argc, argv,"get_distance_pcl");
  PointCloudToImage pci; //this loads up the node
  ros::spin(); //where she stops nobody knows
  return 0;
}
