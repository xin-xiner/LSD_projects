/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once
#undef Success
#include "../LSD/IOWrapper/displaySetting.h"

namespace lsd_slam
{
 typedef unsigned char uchar;
 class Frame;
 struct TrajecVertex
 {
  float point[3];
  uchar color[4];
 };

 class TrajectoryDisplay
 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   TrajectoryDisplay(int r,int g,int b);
  ~TrajectoryDisplay();

  void pushback(Eigen::Vector3f& currentCam);
  void drawTrac();//wx-更新点云的绘制效果
  void refreshPC();

  int r, g, b;
  int pointSize;
  int id;
  double time;
  int totalPoints, displayedPoints;
 

  // buffer & how many
  unsigned int vertexBufferId;
  int vertexBufferNumPoints;

  bool vertexBufferIdValid; // true if the vertixBufferID is valid (doesnt mean the data in there is still valid)
  bool glBuffersValid;  // true if the vertexBufferID contains valid data
  std::vector<Eigen::Vector3f> trajectory;
 };
}
