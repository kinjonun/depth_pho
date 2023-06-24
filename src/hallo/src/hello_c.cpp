#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

ros::Time start_time; // 用于计算相对时间戳

cv::VideoWriter color_video_writer;
cv::VideoWriter depth_video_writer;

void imageCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
  // 将ROS图像消息转换为OpenCV图像（彩色图像和深度图像）
  cv_bridge::CvImagePtr cv_color_ptr, cv_depth_ptr;
  try
  {
    cv_color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
    cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // 录制彩色图像
  cv::Mat color_image = cv_color_ptr->image;
  color_video_writer.write(color_image);

  // 录制深度图像
  cv::Mat depth_image = cv_depth_ptr->image;
  depth_video_writer.write(depth_image);

  // 可以在这里进行其他对彩色图像和深度图像的处理或分析操作

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_recorder");
  ros::NodeHandle nh;

  // 创建彩色图像和深度图像的订阅器
  message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, "/camera_front/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera_front/depth/image_rect_raw", 1);

  // 定义同步策略（近似时间同步）
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), color_sub, depth_sub);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));

  // 获取当前时间作为起始时间
  start_time = ros::Time::now();

  // 设置视频文件名和参数
  std::string color_video_filename = "/path/to/save/color_video.avi"; // 替换为实际的文件路径和名称
  std::string depth_video_filename = "/path/to/save/depth_video.avi"; // 替换为实际的文件路径和名称
  int fps = 30; // 录制的视频帧率
  cv::Size frame_size; // 彩色图像和深度图像的尺寸
  frame_size.width = 1280; // 替换为彩色图像的宽度
  frame_size.height = 720; // 替换为彩色图像的高度

  // 创建视频编写器
  color_video_writer.open(color_video_filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frame_size);
  depth_video_writer.open(depth_video_filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frame_size);

  ros::spin();

  // 关闭视频编写器
  color_video_writer.release();
  depth_video_writer.release();

  return 0;
}
