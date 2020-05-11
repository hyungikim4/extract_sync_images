#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;

void callback(const CompressedImageConstPtr& image1, const ImageConstPtr& image3, const OdometryConstPtr& odom)
{
  // Solve all of perception here...
  ROS_INFO("callback");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<CompressedImage> image1_sub(nh, "/camera/color/image_raw/compressed", 1);
  // message_filters::Subscriber<CompressedImage> image2_sub(nh, "/dvs_rendering/compressed", 1);
  message_filters::Subscriber<Image> image3_sub(nh, "/camera/depth/image_rect_raw", 1);
  message_filters::Subscriber<Odometry> odom_sub(nh, "/Odometry/wheel", 1);

  typedef sync_policies::ApproximateTime<CompressedImage, Image, Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image3_sub, odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}