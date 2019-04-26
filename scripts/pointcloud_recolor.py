#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

class Params():
  def __init__(self):
    print("Parsing Params")

  def parseParams(self):
    # Model name
    self.model = rospy.get_param("~model", "ESPNetv2")
    # Data directory
    self.data_dir = rospy.get_param("~data_dir", './data')
    # RGB Image format
    self.img_extn = rospy.get_param("~img_extn", "png")
    # Width of RGB image
    self.inWidth = rospy.get_param("~inWidth", 1024)
    # Height of RGB image
    self.inHeight = rospy.get_param("~inHeight", 512)
    # Whether to write segmented images to save_dir or not
    self.save_imgs = rospy.get_param('~save_imgs', False)
    # Directory to save the results (ignored if save_img is False)
    self.save_dir = rospy.get_param("~save_dir", "./results")
    # Run on CPU or GPU. If TRUE, then GPU.
    self.gpu = rospy.get_param('~gpu', True)
    # Pretrained weights directory.
    self.pretrained = rospy.get_param('~pretrained', './pretrained')
    # Scale
    self.scale = rospy.get_param('~scale', 0.5)
    # If you want to convert to cityscape original label ids')
    self.cityFormat = rospy.get_param('~cityFormat', True)
    #If you want to visualize the segmentation masks in color')
    self.colored = rospy.get_param('~colored', True)
    # If you want to visualize the segmentation masks overlayed on top of RGB image
    self.overlay = rospy.get_param('~overlay', True)
    # Number of classes in the dataset. 20 for Cityscapes
    self.classes = rospy.get_param('~classes', 20)

    if self.overlay:
      self.colored = True # This has to be true if you want to overlay

class ImageConverter:
  def __init__(self, params):
    # Store params for semantic segmentation
    self.params = params

    # Create ROSMITNET for semantic segmentation
    print("Using Model: {}".format(params.model))
    self.rosmitnet = gc.ROSMITNET(params)

    # Setup ROS interface
    self.image_pub = rospy.Publisher("image_out", Image)
    self.bridge = CvBridge()

    # Use synchronized image and pointcloud!
    self.image_sub = rospy.Subscriber("image_in", Image, self.imageCallback)
    self.image_sub = rospy.Subscriber("pointcloud_in", PointCloud2, self.pointcloudcallback)

  def pointcloudCallback(self, pointcloud):
    """
        Args:
          pointcloud(PointCloud2): ROS PointCloud2 message
        Returns:
    """



  def imageCallback(self, data):
    print("Got an Image!")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.semanticSegmentation(cv_image)
    except CvBridgeError as e:
      print(e)

  def semanticSegmentation(self, cv_image):
    # Run semantic segmentation on a single image.
    cv_image = self.rosmitnet.segmentImage(self.params, cv_image)

    # Visualize in Opencv and ROS
    self.publishToROS(cv_image)

  def publishToROS(self, cv_image):
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main():
    # Init ROS node, do this before parsing params.
    rospy.init_node('pointcloud_recolor', anonymous=True)

    # Parse ROS params.
    params = Params()
    params.parseParams()

    image_converter = ImageConverter(params)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
