#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace sensor_msgs;
using namespace message_filters;


class PointcloudRecolor {
 public:
  PointcloudRecolor(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      image_sub_(nh_, "image_in", 1),
      pointcloud_sub_(nh_, "pointcloud_in", 1),
      sync_(image_sub_, pointcloud_sub_, 10) {
    setupRos();
  }

void setupRos() {
  // Set up publisher.
  pointcloud_recolor_pub_ = nh_.advertise<PointCloud2>(
                              "pointcloud_recolor_out", 1, true);

  // Set up subscriber.
  sync_.registerCallback(boost::bind(&PointcloudRecolor::syncCallback,
                                    this, _1, _2));
}

  void syncCallback(const ImageConstPtr& image,
                    const PointCloud2ConstPtr& pointcloud) {
    // Solve all of perception here...
    ROS_ERROR_STREAM("Received image and pointcloud.");

    // Loop over each entry in the pointcloud and change the rgb data field for
    // the corresponding one in the given image.

    // Check that timestamps agree.
    CHECK_EQ(image->header.stamp, pointcloud->header.stamp)
        << "Image and Pointcloud are not synchronized! If that is ok, you should"
           "make sure that the image and the pointcloud are at least registered.";

    // Check image encoding: for now we need BGR8.
    // Alternatively, modify the for loop to accept others such as BGR8.
    CHECK_EQ(image->encoding, sensor_msgs::image_encodings::BGR8)
        << "Incorrect image encoding, expected BGR8.";

    // Do we require bigendian?
    CHECK_EQ(image->is_bigendian, 0u);

    // Convert the image to OpenCv.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Process cv_ptr->image using OpenCV.
    // TODO do we have to clone the image maybe? It is still a pointer to
    // something at this point.
    recolorPointcloudWithImage(image, pointcloud);
  }

  // This copies the pointcloud but this is ok as we will need to copy it anyway
  // for modification.
  void recolorPointcloudWithImage(ImageConstPtr image_msg,
                                  PointCloud2ConstPtr pointcloud) {
    cv::Mat_<cv::Vec3b> color(image_msg->height, image_msg->width,
                              (cv::Vec3b*)&image_msg->data[0], image_msg->step);

    // Check that sizes agree, aka that the pointcloud given is ordered
    if (image_msg->width != pointcloud->width ||
        image_msg->height != pointcloud->height) {
      ROS_WARN_STREAM("Pointcloud width does not match with image width. "
           "The pointcloud is assumed to be ordered.\n"
           "image.width =" << image_msg->width << '\n' <<
           "image.height =" << image_msg->height <<'\n'<<
           "pointcloud.width =" << pointcloud->width <<'\n'<<
           "pointcloud.height =" << pointcloud->height <<'\n'<<
           "But should have Width Image == Width pointcloud && "
           "Height Image == Height pointcloud.\n"
           "RESIZING IMAGE INSTEAD.");
      cv::resize(color, color,
                 cv::Size(pointcloud->width, pointcloud->height),
                 0, 0, CV_INTER_LINEAR);
    }
    CHECK(color.cols == pointcloud->width &&
          color.rows == pointcloud->height);


  // Fill in new PointCloud2 message (2D image-like layout)
  PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
  points_msg->header = pointcloud->header;
  points_msg->height = pointcloud->height;
  points_msg->width  = pointcloud->width;
  points_msg->is_bigendian = false;
  points_msg->is_dense = false; // there may be invalid points

  sensor_msgs::PointCloud2Modifier out_modifier(*points_msg);
  out_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> out_x(*points_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*points_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*points_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*points_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*points_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*points_msg, "b");

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int i = 0;
  for (int v = 0; v < pointcloud->height; ++v) {
    for (int u = 0; u < pointcloud->width; ++u,
         ++i,
         ++out_x, ++out_y, ++out_z,
         ++out_r, ++out_g, ++out_b) {
      float in_x = 0;
      float in_y = 0;
      float in_z = 0;

      memcpy (&in_x, &pointcloud->data[i * pointcloud->point_step + 0], sizeof (float));
      memcpy (&in_y, &pointcloud->data[i * pointcloud->point_step + 4], sizeof (float));
      memcpy (&in_z, &pointcloud->data[i * pointcloud->point_step + 8], sizeof (float));

      // x,y,z
      *out_x = in_x;
      *out_y = in_y;
      *out_z = in_z;
      // Change colors
      const cv::Vec3b& bgr = color(v,u);
      *out_r = bgr[2];
      *out_g = bgr[1];
      *out_b = bgr[0];
    }
  }

    // Fill in sparse point cloud message
    //sensor_msgs::PointCloud2 points = pointcloud;
    //points.data.clear();
    //ROS_ERROR_STREAM("PCL SIZXE: " << pointcloud.data.size());
    //points.data.resize(pointcloud.data.size());

    //int i = 0;
    //for (int u = 0; u < image.rows; ++u) {
    //  for (int v = 0; v < image.cols; ++v, ++i) {

    //    memcpy (&points.data[i * points.point_step + 0], &pointcloud.data[i * pointcloud.point_step + 0], sizeof (float));
    //    memcpy (&points.data[i * points.point_step + 4], &pointcloud.data[i * pointcloud.point_step + 4], sizeof (float));
    //    memcpy (&points.data[i * points.point_step + 8], &pointcloud.data[i * pointcloud.point_step + 8], sizeof (float));

    //    // Fill in r, g, b local variables with image data.
    //    // image data is in BGR8
    //    const cv::Vec3b& bgr = image.at<cv::Vec3b>(u, v);
    //    int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
    //    memcpy(&points.data[i * points.point_step + 12],
    //           //&pointcloud.data[i * pointcloud.point_step + 12],
    //          &rgb_packed,
    //          sizeof (int32_t));
    //  }
    //}

    // Publish to network.
    pointcloud_recolor_pub_.publish(points_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pointcloud_recolor_pub_;
  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<PointCloud2> pointcloud_sub_;
  TimeSynchronizer<Image, PointCloud2> sync_;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "pointcloud_recolor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");

  PointcloudRecolor pointcloud_recolor (nh, nh_private_);

  ros::spin();

  return 0;
}
