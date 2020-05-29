#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <math.h>

static double init_offset_odom_;
static double interval_odom_;
static std::string save_path_;

bool first = true;
bool init_offset = true;
double first_x;
double first_y;
double prev_x;
double prev_y;
int img_count = 0;

void SyncCallBack(const sensor_msgs::CompressedImageConstPtr& event_img_msg, const sensor_msgs::CompressedImageConstPtr& rgb_img_msg,
                    const sensor_msgs::ImageConstPtr& depth_img_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    ROS_INFO("sync callback");
    // first time
    if (first)
    {
        ROS_INFO("first");
        first_x = odom_msg->pose.pose.position.x;
        first_y = odom_msg->pose.pose.position.y;
        first = false;
        return;
    }

    double cur_x = odom_msg->pose.pose.position.x;
    double cur_y = odom_msg->pose.pose.position.y;


    if (init_offset)
    {
        double init_odom = sqrt((cur_x-first_x)*(cur_x-first_x)+(cur_y-first_y)*(cur_y-first_y));
        ROS_INFO("init odom %f", init_odom);
        if (init_odom >= init_offset_odom_)
        {
            prev_x = cur_x;
            prev_y = cur_y;
            init_offset = false;
        }
        return;
    }


    double odom = sqrt((cur_x-prev_x)*(cur_x-prev_x)+(cur_y-prev_y)*(cur_y-prev_y));
    ROS_INFO("odom %f", odom);

    if (odom < interval_odom_)
        return;

    // extract images
    char buf[100];
    
    // event camera
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(event_img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat event_img = cv_ptr->image;
    sprintf(buf, (save_path_+"event/%06d.jpg").c_str(), img_count);
    cv::imwrite(buf, event_img);

    // rgb image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb_img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rgb_img = cv_ptr->image;
    sprintf(buf, (save_path_+"rgb/%06d.jpg").c_str(), img_count);
    cv::imwrite(buf, rgb_img);

    // depth image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat depth_img = cv_ptr->image;
    sprintf(buf, (save_path_+"depth/%06d.jpg").c_str(), img_count);
    cv::imwrite(buf, depth_img);

    ROS_INFO("Save %06d.jpg", img_count);
    prev_x = cur_x;
    prev_y = cur_y;
    img_count++;
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_sync_images");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    priv_nh.param("init_offset_odom", init_offset_odom_, 3.);
    priv_nh.param("interval_odom", interval_odom_, 5.);
    priv_nh.param("save_path", save_path_, std::string("/media/khg/HDD1TB/bagfiles/tram_dataset/"));

    message_filters::Subscriber<sensor_msgs::CompressedImage> event_image_sync(nh, "/prophesee/camera/cd_events_image/compressed", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_image_sync(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sync(nh, "/camera/depth/image_rect_raw", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sync(nh, "/Odometry/ekf_estimated", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                            sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), event_image_sync, rgb_image_sync, depth_image_sync, odom_sync);
    sync.registerCallback(boost::bind(&SyncCallBack, _1, _2, _3, _4));

    ros::spin();
    return 0;
}