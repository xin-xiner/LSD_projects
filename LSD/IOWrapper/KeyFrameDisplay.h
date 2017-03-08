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
#include "../preprocessHeaders.h"
#include "displaySetting.h"

namespace lsd_slam
{
 typedef unsigned char uchar;
 class Frame;
 struct MyVertex
 {
  float point[3];
  uchar color[4];
 };
 struct InputPointDense
 {
  float idepth;
  float idepth_var;
  uchar color[4];
 };
 // stores a pointcloud associated to a Keyframe.
 //wx-对每个关键帧保存生成点云所需要的数据深度和颜色信息，并提供由关键帧创建点云，显示点云、保存点云、显示关键帧camera的操作
 //注意该类中并不直接存储点云信息，仅提供在显示和输出到文件时生成点云的操作 （这样做的原因可能是全局优化会随时的更新关键帧里的深度，在每次输出点云和显示点云前再计算点云是一个安全的操作）
 //在算法的运算中并没有用到点云数据，所以将构建点云的过程放置在显示模块中
 class KeyFrameDisplay
 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   KeyFrameDisplay();
  ~KeyFrameDisplay();

  void setFrom(Frame *frame, bool isKeyFrame);
  void drawCam(float lineWidth = 1, float* color = 0);
  void drawPC(float pointSize = 1, float alpha = 1);//wx-更新点云的绘制效果
  void refreshPC();//wx-重新计算点云，并更新openGL缓冲中的点云信息
  int  refreshPC(float* pointCloud, float* colors,int vertexNumberInPointCloud);//create-by-wx 2015-11-6 用来在添加关键帧之后向界面输出一次新计算的点云
  int flushPC(std::ofstream* f);//wx-重新计算点云并输出到文件f
  //输出的格式为三维坐标加三维颜色(255形式)
  //在keyframeGraphDisplay中被调用时生成ply格式的点云文件


  int id;
  double time;
  int totalPoints, displayedPoints;

  // camera pose
  // may be updated by kf-graph.
  Sophus::Sim3f camToWorld;
  float savedCamToWorld[7];
  // pointcloud data & respective buffer
  InputPointDense* originalInput;
  // camera parameter
  // fixed.
  float fx, fy, cx, cy;
  float fxi, fyi, cxi, cyi;
  int width, height;
  float my_scaledTH, my_absTH, my_scale;
  int my_minNearSupport;
  int my_sparsifyFactor;



  // buffer & how many
  unsigned int vertexBufferId;
  int vertexBufferNumPoints;

  bool vertexBufferIdValid; // true if the vertixBufferID is valid (doesnt mean the data in there is still valid)
  bool glBuffersValid;  // true if the vertexBufferID contains valid data
 };
}


