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
 //wx-��ÿ���ؼ�֡�������ɵ�������Ҫ��������Ⱥ���ɫ��Ϣ�����ṩ�ɹؼ�֡�������ƣ���ʾ���ơ�������ơ���ʾ�ؼ�֡camera�Ĳ���
 //ע������в���ֱ�Ӵ洢������Ϣ�����ṩ����ʾ��������ļ�ʱ���ɵ��ƵĲ��� ����������ԭ�������ȫ���Ż�����ʱ�ĸ��¹ؼ�֡�����ȣ���ÿ��������ƺ���ʾ����ǰ�ټ��������һ����ȫ�Ĳ�����
 //���㷨�������в�û���õ��������ݣ����Խ��������ƵĹ��̷�������ʾģ����
 class KeyFrameDisplay
 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   KeyFrameDisplay();
  ~KeyFrameDisplay();

  void setFrom(Frame *frame, bool isKeyFrame);
  void drawCam(float lineWidth = 1, float* color = 0);
  void drawPC(float pointSize = 1, float alpha = 1);//wx-���µ��ƵĻ���Ч��
  void refreshPC();//wx-���¼�����ƣ�������openGL�����еĵ�����Ϣ
  int  refreshPC(float* pointCloud, float* colors,int vertexNumberInPointCloud);//create-by-wx 2015-11-6 ��������ӹؼ�֮֡����������һ���¼���ĵ���
  int flushPC(std::ofstream* f);//wx-���¼�����Ʋ�������ļ�f
  //����ĸ�ʽΪ��ά�������ά��ɫ(255��ʽ)
  //��keyframeGraphDisplay�б�����ʱ����ply��ʽ�ĵ����ļ�


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


