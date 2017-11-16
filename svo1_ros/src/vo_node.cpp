// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include <ros/package.h>
#include <string>
#include <svo1/frame_handler_mono.h>
#include <svo1/map.h>
#include <svo1/config.h>
#include <svo1_ros/visualizer.h>
#include <vikit1/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit1/abstract_camera.h>
#include <vikit1/camera_loader.h>
#include <vikit1/user_input_thread.h>

namespace svo1 {

/// SVO Interface
class VoNode
{
public:
  svo1::FrameHandlerMono* vo_;
  svo1::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;
  ros::Time last_frame_t_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo1/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo1/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false),
  last_frame_t_(ros::Time::now())
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("svo1/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo1", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3d(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo1/init_rx", 0.0),
                           vk::getParam<double>("svo1/init_ry", 0.0),
                           vk::getParam<double>("svo1/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo1/init_tx", 0.0),
                      vk::getParam<double>("svo1/init_ty", 0.0),
                      vk::getParam<double>("svo1/init_tz", 0.0)));

  // Init VO and start
  vo_ = new svo1::FrameHandlerMono(cam_);
  vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  last_frame_t_ = (msg->header.stamp != ros::Time(0)) ? msg->header.stamp : ros::Time::now();
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();
  vo_->addImage(img, last_frame_t_.toSec());
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, last_frame_t_.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo1");
  ros::NodeHandle nh;
  std::cout << "Creating vo_node" << std::endl;
  svo1::VoNode vo_node;

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo1/cam_topic", "camera/image_raw"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo1::VoNode::imgCb, &vo_node);

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("svo1/remote_key", 5, &svo1::VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    if (ros::Time::now() - vo_node.last_frame_t_ > ros::Duration(5)) {
        ROS_WARN_DELAYED_THROTTLE(1.0, "Not receiving any image callbacks, hung camera?");
    }
  }

  printf("SVO terminated.\n");
  return 0;
}
