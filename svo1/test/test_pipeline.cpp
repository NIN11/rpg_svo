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

#include <svo1/config.h>
#include <svo1/frame_handler_mono.h>
#include <svo1/map.h>
#include <svo1/frame.h>
#include <vector>
#include <string>
#include <vikit1/math_utils.h>
#include <vikit1/vision.h>
#include <vikit1/abstract_camera.h>
#include <vikit1/atan_camera.h>
#include <vikit1/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <iostream>
#include "test_utils.h"

namespace svo1 {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo1::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
  cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
  vo_ = new svo1::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder()
{
  for(int img_id = 2; img_id < 188; ++img_id)
  {
    // load image
    std::stringstream ss;
    ss << svo1::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
       << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
    if(img_id == 2)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }
}

} // namespace svo1

int main(int argc, char** argv)
{
  {
    svo1::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

