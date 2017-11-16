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

#include <string.h>
#include <svo1/global.h>
#include <svo1/config.h>
#include <svo1/frame.h>
#include <svo1/feature_detection.h>
#include <svo1/depth_filter.h>
#include <svo1/feature.h>
#include <vikit1/timer.h>
#include <vikit1/vision.h>
#include <vikit1/abstract_camera.h>
#include <vikit1/atan_camera.h>
#include "test_utils.h"

namespace {

using namespace Eigen;
using namespace std;

void testCornerDetector()
{
  std::string img_name(svo1::test_utils::getDatasetDir() + "/sin2_tex2_h1_v8_d/img/frame_000002_0.png");
  printf("Loading image '%s'\n", img_name.c_str());
  cv::Mat img(cv::imread(img_name, 0));
  assert(img.type() == CV_8UC1 && !img.empty());

  vk::AbstractCamera* cam = new vk::ATANCamera(752, 480, 0.511496, 0.802603, 0.530199, 0.496011, 0.934092);
  svo1::FramePtr frame(new svo1::Frame(cam, img, 0.0));

  // Corner detection
  vk::Timer t;
  svo1::Features fts;
  svo1::feature_detection::FastDetector fast_detector(
      img.cols, img.rows, svo1::Config::gridSize(), svo1::Config::nPyrLevels());
  for(int i=0; i<100; ++i)
  {
    fast_detector.detect(frame.get(), frame->img_pyr_, svo1::Config::triangMinCornerScore(), fts);
  }
  printf("Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 7.166360ms, 40000)\n", t.stop()*10, fts.size());
  printf("Note, in this case, feature detection also contains the cam2world projection of the feature.\n");
  cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
  cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
  std::for_each(fts.begin(), fts.end(), [&](svo1::Feature* i){
    cv::circle(img_rgb, cv::Point2f(i->px[0], i->px[1]), 4*(i->level+1), cv::Scalar(0,255,0), 1);
  });
  cv::imshow("ref_img", img_rgb);
  cv::waitKey(0);

  std::for_each(fts.begin(), fts.end(), [&](svo1::Feature* i){ delete i; });
}

} // namespace


int main(int argc, char **argv)
{
  testCornerDetector();
  return 0;
}
