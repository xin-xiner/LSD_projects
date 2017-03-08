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

#ifndef KEYFRAMEGRAPHDISPLAY_H_
#define KEYFRAMEGRAPHDISPLAY_H_
#include "../preprocessHeaders.h"



namespace lsd_slam
{
 class KeyFrameDisplay;
 class Frame;
 struct GraphMsg;
 struct GraphConstraint
 {
  int from;
  int to;
  float err;
 };

 struct GraphConstraintPt
 {
  KeyFrameDisplay* from;
  KeyFrameDisplay* to;
  float err;
 };
 struct GraphFramePose
 {
  int id;
  float camToWorld[7];
 };

 //wx-封装了keyframGraph的显示、保存、读取等操作
 class KeyFrameGraphDisplay {
 public:
  KeyFrameGraphDisplay();
  KeyFrameGraphDisplay(const std::string _keyFramePath);
  virtual ~KeyFrameGraphDisplay();
  void draw();//显示点云、关键帧摄像机、keyframeGraph的边(可选)、输出点云的ply文件(可选)
  void saveAll();//将keyframGraph中所有关键帧的逆深度图和图片输出到文件
  void savePointCloud();//create-by-wx 2015-10-28 将当前keyframeGraph对应的点云输出到文件
  void readAll(std::string kfPath, int cnt);
  int  refreshPointCloud(float* pointCloud, float* colors);//create-by-wx 2015-11-6 用来在添加关键帧之后向界面输出一次新计算的点云
  int  keyframeNumber();//create-by-wx 2015-11-6 返回keyframe的大小
  void addMsg(Frame *frame);
  void addGraphMsg(GraphMsg *msg);


  bool flushPointcloud;
  bool printNumbers;
  std::string keyFramePath;
 private:
  std::map<int, KeyFrameDisplay*> keyframesByID;
  std::vector<KeyFrameDisplay*> keyframes;
  std::vector<GraphConstraintPt> constraints;
  boost::mutex dataMutex;
 };
}
#endif /* KEYFRAMEGRAPHDISPLAY_H_ */

