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
#include "preprocessHeaders.h"
#include "KeyFrameGraphDisplay.h"
#include "KeyFrameDisplay.h"
#include "../DataStructures/Frame.h"
#include "../IOWrapper/OpenCV/CVOutput3DWarpper.h"
#include "displaySetting.h"


//#include "ros/package.h"
//using namespace lsd_slam;
namespace lsd_slam
{
#define KEYFRAME_PATH "/keyframe/"

 KeyFrameGraphDisplay::KeyFrameGraphDisplay()
 {
  flushPointcloud = false;
  printNumbers = false;
  keyFramePath = KEYFRAME_PATH;
 }
 KeyFrameGraphDisplay::KeyFrameGraphDisplay(const std::string _keyFramePath)
 {
  flushPointcloud = false;
  printNumbers = false;
  keyFramePath = _keyFramePath;
 }
 KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
 {
  for (unsigned int i = 0; i < keyframes.size(); i++)
   delete keyframes[i];
 }

 void KeyFrameGraphDisplay::draw()
 {
  dataMutex.lock();
  numRefreshedAlready = 0;
  // draw keyframes
  float color[3] = { 0, 0, 1 };
  for (unsigned int i = 0; i<keyframes.size(); i++)
  {
   if (showKFCameras)
    keyframes[i]->drawCam(lineTesselation, color);
   if ((showKFPointclouds && (int)i > cutFirstNKf) || i == keyframes.size() - 1)
    keyframes[i]->drawPC(pointTesselation, 1);
  }
#ifdef OPENGL_USE
  if (flushPointcloud)
  {
   //  printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
   //  std::ofstream f((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
   std::ofstream f("lsd_slam_viewer/pc_tmp.ply");//wx-先将点云的坐标和颜色信息输出到tmp文件中
   int numpts = 0;
   for (unsigned int i = 0; i<keyframes.size(); i++)
   {
    if ((int)i > cutFirstNKf)
     numpts += keyframes[i]->flushPC(&f);
   }
   f.flush();
   f.close();
   std::ofstream f2("lsd_slam_viewer/pc.ply");//wx-构建ply文件的文件头
   f2 << std::string("ply\n");
   f2 << std::string("format binary_little_endian 1.0\n");
   f2 << std::string("element vertex ") << numpts << std::string("\n");
   f2 << std::string("property float x\n");
   f2 << std::string("property float y\n");
   f2 << std::string("property float z\n");
   f2 << std::string("property float intensity\n");
   f2 << std::string("end_header\n");
   std::ifstream f3("lsd_slam_viewer/pc_tmp.ply");//wx-将包含点云坐标和颜色信息的tmp文件内容复制到ply文件中(为什么感觉没有必要要tmp文件就可以完成，读写文件本来就很慢)
   while (!f3.eof()) f2.put(f3.get());
   f2.close();
   f3.close();
   //   system(("rm "+ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
   flushPointcloud = false;
   printf("Done Flushing Pointcloud with %d points!\n", numpts);
  }
  if (printNumbers)
  {
   int totalPoint = 0;
   int visPoints = 0;
   for (unsigned int i = 0; i < keyframes.size(); i++)
   {
    totalPoint += keyframes[i]->totalPoints;
    visPoints += keyframes[i]->displayedPoints;
   }
   printf("Have %d points, %d keyframes, %d constraints. Displaying %d points.\n",
    totalPoint, (int)keyframes.size(), (int)constraints.size(), visPoints);
   printNumbers = false;
  }
  if (showConstraints)
  {
   // draw constraints
   glLineWidth(lineTesselation);
   glBegin(GL_LINES);
   for (unsigned int i = 0; i < constraints.size(); i++)
   {
    if (constraints[i].from == 0 || constraints[i].to == 0)
     continue;
    double colorScalar = ((std::max))(0.0, ((std::min))(1.0, constraints[i].err / 0.05));
    glColor3f(colorScalar, 1 - colorScalar, 0);

    Sophus::Vector3f t = constraints[i].from->camToWorld.translation();
    glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
    t = constraints[i].to->camToWorld.translation();
    glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
   }
   glEnd();
  }
#define SHOW_TRACK_CALIBRATION
#ifdef SHOW_TRACK_CALIBRATION
  //glLineWidth(2);
  //glBegin(GL_LINES);
  //for (int i = 0; i < (int)(keyframes.size() - 1); i++)
  //{
  // glColor3f(1, 1, 0);
  // Sophus::Vector3f t = keyframes[i]->camToWorld.translation();
  // glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
  // t = keyframes[i + 1]->camToWorld.translation();
  // glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
  //}
  //glEnd();
  //cv::FileStorage fs("result.yml", cv::FileStorage::READ);
  //cv::Mat extrinsic_mat;
  //fs["extrinsic_parameters"] >> extrinsic_mat;
  //std::vector<cv::Mat> R_gt, t_gt;
  //R_gt.resize(extrinsic_mat.rows);
  //t_gt.resize(extrinsic_mat.rows);
  //for (int i = 0; i < extrinsic_mat.rows; i++)
  //{
  // cv::Mat r = extrinsic_mat(cv::Range(i, i + 1), cv::Range(0, 3));
  // cv::Mat t = extrinsic_mat(cv::Range(i, i + 1), cv::Range(3, 6));
  // Rodrigues(r, R_gt[i]);
  // t.copyTo(t_gt[i]);
  //}
  //glLineWidth(2);
  //glBegin(GL_LINES);
  //glColor3f(1, 1, 1);
  ////int sizeTmep__ = ;
  //for (int i = 0; (i<(int)(t_gt.size() - 1)); i++)
  //{
  // cv::Mat t1 = t_gt[i];
  // cv::Mat t2 = t_gt[i + 1];
  // glVertex3f((GLfloat)t1.at<double>(0, 0), (GLfloat)t1.at<double>(0, 1), (GLfloat)t1.at<double>(0, 2));
  // glVertex3f((GLfloat)t2.at<double>(0, 0), (GLfloat)t2.at<double>(0, 1), (GLfloat)t2.at<double>(0, 2));
  //}
  glEnd();

#endif

  
  
#endif
  dataMutex.unlock();
 }

 void KeyFrameGraphDisplay::saveAll()
 {
  std::ofstream keyfs;
  keyfs.open(keyFramePath + (std::string)("keyframes.txt"), std::ios::out);
  for (unsigned int i = 0; i < keyframes.size(); i++)
  {
   std::stringstream sstr;
   std::string str;//wx-保存关键帧文件的地址
   std::string str_img;//wx-保存关键帧图片的地址
   sstr << keyFramePath  << i << "d.kf";
   sstr >> str;
   sstr.clear();
   sstr << keyFramePath  << i << "_" << keyframes[i]->id << ".bmp";
   sstr >> str_img;
   std::ofstream f;
   f.open(str.c_str(), std::ios::binary);
   
   memcpy(keyframes[i]->savedCamToWorld, keyframes[i]->camToWorld.data(), 7 * sizeof(float));
   //f.write((const char*)keyframes[i], sizeof(KeyFrameDisplay));
   /*f.write((const char*)(keyframes[i]->originalInput), keyframes[i]->width*keyframes[i]->height*sizeof(InputPointDense));*///eidt-by-wx-try 2015-11-4
 
   for (int ii = 0; ii < keyframes[i]->height; ii++)
   {
    for (int jj = 0; jj < keyframes[i]->width; jj++)
    {
     f << keyframes[i]->originalInput[ii*keyframes[i]->height + jj].idepth << " ";
    }
    f << std::endl;
   }

   f.close();
   float *ptmp = keyframes[i]->savedCamToWorld;//wx-保存keyframe的摄像机信息
   keyfs << ptmp[0] << "," << ptmp[1] << "," << ptmp[2] << ","  \
    << ptmp[3] << "," << ptmp[4] << "," << ptmp[5] << ","  \
    << ptmp[6] << "\n";

  }
  keyfs.close();
 }

 void KeyFrameGraphDisplay::savePointCloud()//create-by-wx 2015-10-28 将当前keyframeGraph对应的点云输出到文件
 {
  //std::ofstream f(keyFramePath+"pc_tmp.ply");//wx-先将点云的坐标和颜色信息输出到tmp文件中
  //int numpts = 0;
  //for (unsigned int i = 0; i<keyframes.size(); i++)
  //{
  // //if ((int)i > cutFirstNKf) 
  //  numpts += keyframes[i]->flushPC(&f);
  //}
  //f.flush();
  //f.close();
  std::ofstream f2(keyFramePath + "pc.xyz");//wx-构建ply文件的文件头
  //f2 << std::string("ply\n");
  //f2 << std::string("format binary_little_endian 1.0\n");
  //f2 << std::string("element vertex ") << numpts << std::string("\n");
  //f2 << std::string("property float x\n");
  //f2 << std::string("property float y\n");
  //f2 << std::string("property float z\n");
  //f2 << std::string("property float intensity\n");
  //f2 << std::string("end_header\n");
  for (unsigned int i = 0; i<keyframes.size(); i++)
  {
   if ((int)i > cutFirstNKf)
    keyframes[i]->flushPC(&f2);
  }
  f2.flush();
  f2.close();
 }

 int KeyFrameGraphDisplay::refreshPointCloud(float* pointCloud, float* colors)//create-by-wx 2015-11-06 在加入关键帧时，给界面输出更新过的点云
 {
  int count = 0;
  for (unsigned int i = 0; i<keyframes.size(); i++)
  {
   if ((int)i > cutFirstNKf)
   count += keyframes[i]->refreshPC(pointCloud, colors, count);
   //std::cout << "count" << count << std::endl;
  }
  return count;
 }

 void KeyFrameGraphDisplay::readAll(std::string kfPath, int cnt)
 {
  int i = 0;
  keyframes.clear();//guo-这里要保证keyframes大小为0;
  while (true)
  {
   std::stringstream sstr;
   std::string str;
   sstr << kfPath << "/" << i << "d.kf";
   sstr >> str;//keyframe文件路径
   //   char name[30];
   //   sprintf_s(name, 30, "keyframe/%d.kf", i);
   std::ifstream f;
   f.open(str.c_str(), std::ios::binary);
   keyframes.push_back(new KeyFrameDisplay());
   f.read((char*)keyframes[i], sizeof(KeyFrameDisplay));
   keyframes[i]->glBuffersValid = false;
   memcpy(keyframes[i]->camToWorld.data(), keyframes[i]->savedCamToWorld, 7 * sizeof(float));
   keyframes[i]->originalInput = new InputPointDense[keyframes[i]->width*keyframes[i]->height];
   f.read((char*)(keyframes[i]->originalInput), keyframes[i]->width*keyframes[i]->height*sizeof(InputPointDense));
   i++;
   if (i == cnt)break;
  }
 }

 void KeyFrameGraphDisplay::addMsg(Frame *frame)
 {
  dataMutex.lock();
  if (keyframesByID.count(frame->id()) == 0)
  {
   KeyFrameDisplay* disp = new KeyFrameDisplay();
   keyframesByID[frame->id()] = disp;
   keyframes.push_back(disp);
   // printf("added new KF, now there are %d!\n", (int)keyframes.size());
  }
  keyframesByID[frame->id()]->setFrom(frame, true);
  dataMutex.unlock();
 }
 void KeyFrameGraphDisplay::addGraphMsg(GraphMsg *msg)
 {
  dataMutex.lock();
  constraints.resize(msg->numConstraints);
  
  assert(msg->constraintsData.size() == msg->numConstraints);//edit-by-wx 2015-10-28 之前这里的判断的是一个内存大小和vector元素个数，会报错，怀疑原程序有问题，之前的写法是assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
  GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
  for (int i = 0; i < msg->numConstraints; i++)
  {
   constraints[i].err = constraintsIn[i].err;
   constraints[i].from = 0;
   constraints[i].to = 0;
   if (keyframesByID.count(constraintsIn[i].from) != 0)
    constraints[i].from = keyframesByID[constraintsIn[i].from];
   //  else
   //   printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
   //     constraintsIn[i].from,
   //     constraintsIn[i].to,
   //     constraintsIn[i].from);

   if (keyframesByID.count(constraintsIn[i].to) != 0)
    constraints[i].to = keyframesByID[constraintsIn[i].to];
   //  else
   //   printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
   //     constraintsIn[i].from,
   //     constraintsIn[i].to,
   //     constraintsIn[i].to);
  }


  GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
  int numGraphPoses = msg->numFrames;
  
  assert(msg->frameData.size() == msg->numFrames);//edit-by-wx 2015-10-28 之前这里的判断的是一个内存大小和vector元素个数，会报错，怀疑原程序有问题，之前的写法是//assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);
  for (int i = 0; i < numGraphPoses; i++)
  {
   if (keyframesByID.count(graphPoses[i].id) == 0)
   {
    // printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
   }
   else
    memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7 * sizeof(float));
  }
  dataMutex.unlock();
  // printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
 }

 int  KeyFrameGraphDisplay::keyframeNumber()
 {
  return keyframes.size();
 }


}//namespace lsd_slam


