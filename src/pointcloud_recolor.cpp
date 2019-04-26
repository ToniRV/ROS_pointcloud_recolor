#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <glog/logging.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& cam_info) {
  // Solve all of perception here...
  //
  // Loop over each entry in the pointcloud and change the rgb data field for
  // the corresponding one in the given image.

  // Check that timestamps agree.

  //
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_recolor");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_sub(nh, "image_in", 10);
  message_filters::Subscriber<PointCloud2> pointcloud_sub(nh, "pointcloud_in", 10);
  TimeSynchronizer<Image, PointCloud2> sync(image_sub, pointcloud_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
