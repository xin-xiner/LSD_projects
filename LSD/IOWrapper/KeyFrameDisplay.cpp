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
#define GL_GLEXT_PROTOTYPES 1
#include "preprocessHeaders.h"
#include "KeyFrameDisplay.h"
#include "displaySetting.h"
#include "DataStructures/Frame.h"
#ifdef ANDROID
#include <GLES3/gl3.h>
#endif // ANDROID


//using namespace lsd_slam;
namespace lsd_slam
{

KeyFrameDisplay::KeyFrameDisplay()
{
 
 originalInput = 0;
 id = 0;
 vertexBufferIdValid = false;
 glBuffersValid = false;

 camToWorld = Sophus::Sim3f();
 width=height=0;
 my_scaledTH = my_absTH = 0;
 totalPoints = displayedPoints = 0;
 my_scaledTH = scaledDepthVarTH;
 my_absTH = absDepthVarTH;
 my_scale = camToWorld.scale();
 my_minNearSupport = minNearSupport;
 my_sparsifyFactor = sparsifyFactor;
 vertexBufferNumPoints = 0;
 
}

KeyFrameDisplay::~KeyFrameDisplay()
{
 
 if(vertexBufferIdValid)
 {
#ifdef OPENGL_USE//edit-by-wx  2015-10-30 增加了条件编译
  glDeleteBuffers(1, &vertexBufferId);
#endif
  vertexBufferIdValid = false;
 }
 if(originalInput != 0)
  delete[] originalInput;
 
}

void KeyFrameDisplay::setFrom(Frame *frame,bool isKeyFrame)
{
 
 // copy over campose.
 memcpy(camToWorld.data(), frame->getScaledCamToWorld().cast<float>().data(), 7 * sizeof(float));
 fx = frame->fx(0);
 fy = frame->fy(0);
 cx = frame->cx(0);
 cy = frame->cy(0);
 fxi = 1/fx;
 fyi = 1/fy;
 cxi = -cx / fx;
 cyi = -cy / fy;
 width = frame->width(0);
 height = frame->height(0);
 id = frame->id();
// time = frame->time;
 if(originalInput != 0)
  delete[] originalInput;
 originalInput=0;
#if 0
 if(msg->pointcloud.size() != width*height*sizeof(InputPointDense))
 {
  if(msg->pointcloud.size() != 0)
  {
   printf("WARNING: PC with points, but number of points not right! (is %zu, should be %u*%dx%d=%u)\n",
     msg->pointcloud.size(), sizeof(InputPointDense), width, height, width*height*sizeof(InputPointDense));
  }
 }
 else
 {
  originalInput = new InputPointDense[width*height];
  memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
 }
#endif
 if (isKeyFrame)
 {
  originalInput = new InputPointDense[width*height];
  const float* idepth = frame->idepth(0);
  const float* idepthVar = frame->idepthVar(0);
  const float* color = frame->image(0);
  const unsigned char *imageColor = frame->imageColor();
  for (int idx = 0; idx < width*height; idx++)
  {
   if (originalInput == 0)
    return;
   originalInput[idx].idepth = idepth[idx];
   originalInput[idx].idepth_var = idepthVar[idx];
   if (imageColor != 0)
   {
    originalInput[idx].color[0] = *imageColor;
    originalInput[idx].color[1] = *(imageColor+1);
    originalInput[idx].color[2] = *(imageColor+2);
    imageColor +=3;
   }
   else
   {
    originalInput[idx].color[0] = color[idx];
    originalInput[idx].color[1] = color[idx];
    originalInput[idx].color[2] = color[idx];
   }
   originalInput[idx].color[3] = color[idx];
  }
 }
 
 glBuffersValid = false;
 
}
void KeyFrameDisplay::refreshPC()
{
 
 //minNearSupport = 9;
 bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
   my_absTH == absDepthVarTH &&
   my_scale*1.2 > camToWorld.scale() &&
   my_scale < camToWorld.scale()*1.2 &&
   my_minNearSupport == minNearSupport &&
   my_sparsifyFactor == sparsifyFactor;


 if(glBuffersValid && (paramsStillGood || numRefreshedAlready > 10)) return;
#ifdef OPENGL_USE
 numRefreshedAlready++;
 glBuffersValid = true;

 // delete old vertex buffer
 if(vertexBufferIdValid)
 {
  glDeleteBuffers(1, &vertexBufferId);
  vertexBufferIdValid = false;
 }

#endif
 // if there are no vertices, done!
 if(originalInput == 0)
  return;
#ifdef OPENGL_USE
  //make data
 MyVertex* tmpBuffer = new MyVertex[width*height];
#endif
 my_scaledTH =scaledDepthVarTH;
 my_absTH = absDepthVarTH;
 my_scale = camToWorld.scale();
 my_minNearSupport = minNearSupport;
 my_sparsifyFactor = sparsifyFactor;
 // data is directly in ros message, in correct format.
 vertexBufferNumPoints = 0;
#ifdef OPENGL_USE
 
 int total = 0, displayed = 0;//wx-outlier removal 处理(论文Semi-Dense visual Odometry for a monocular camera 第2.3节)(与flushPC中的操作重复,不太合理)
 for(int y=1;y<height-1;y++)
  for(int x=1;x<width-1;x++)
  {
   if(originalInput[x+y*width].idepth <= 0) continue;
   total++;

   if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;
   float depth = 1 / originalInput[x+y*width].idepth;
   float depth4 = depth*depth; depth4*= depth4;

   if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
    continue;
   if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
    continue;
   if(my_minNearSupport > 1)
   {
    int nearSupport = 0;
    for(int dx=-1;dx<2;dx++)
     for(int dy=-1;dy<2;dy++)
     {
      int idx = x+dx+(y+dy)*width;
      if(originalInput[idx].idepth > 0)
      {
       float diff = originalInput[idx].idepth - 1.0f / depth;
       if(diff*diff < 2*originalInput[x+y*width].idepth_var)
        nearSupport++;
      }
     }
    if(nearSupport < my_minNearSupport)
     continue;
   }
   tmpBuffer[vertexBufferNumPoints].point[0] = (x*fxi + cxi) * depth;
   tmpBuffer[vertexBufferNumPoints].point[1] = (y*fyi + cyi) * depth;//edit-by-wx 2015-11-10 为了使openGL的显示结果是正的这里给y轴和z轴取了负数
   tmpBuffer[vertexBufferNumPoints].point[2] = depth;
   tmpBuffer[vertexBufferNumPoints].color[3] = 100;
   tmpBuffer[vertexBufferNumPoints].color[2] = originalInput[x+y*width].color[0];
   tmpBuffer[vertexBufferNumPoints].color[1] = originalInput[x+y*width].color[1];
   tmpBuffer[vertexBufferNumPoints].color[0] = originalInput[x+y*width].color[2];
   vertexBufferNumPoints++;
   displayed++;
  }
 totalPoints = total;
 displayedPoints = displayed;
 // create new ones, static
 vertexBufferId=0;
 glGenBuffers(1, &vertexBufferId);
 
 glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
 glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);//wx-将创建好的点云数据传递给openGL的数据结构中
 vertexBufferIdValid = true;


 if(!keepInMemory)
 {
  delete[] originalInput;
  originalInput = 0;
 }



 delete[] tmpBuffer;
 
#endif
}

int  KeyFrameDisplay::refreshPC(float* pointCloud, float* colors, int vertexNumberInPointCloud)//create-by-wx 2015-11-6 用来在添加关键帧之后向界面输出一次新计算的点云
{

 //minNearSupport = 9;
 bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
  my_absTH == absDepthVarTH &&
  my_scale*1.2 > camToWorld.scale() &&
  my_scale < camToWorld.scale()*1.2 &&
  my_minNearSupport == minNearSupport &&
  my_sparsifyFactor == sparsifyFactor;



 //std::cout <<"originalInput"<< originalInput << std::endl;
 if (originalInput == 0)
  return 0;
 my_scaledTH = scaledDepthVarTH;
 my_absTH = absDepthVarTH;
 my_scale = camToWorld.scale();
 my_minNearSupport = minNearSupport;
 my_sparsifyFactor = sparsifyFactor;

 vertexBufferNumPoints = 0;
 int startPositionPC = vertexNumberInPointCloud * 3;
 int startPositionColor = vertexNumberInPointCloud * 4;
 int total = 0, displayed = 0;//wx-outlier removal 处理(论文Semi-Dense visual Odometry for a monocular camera 第2.3节)(与flushPC中的操作重复,不太合理)
 for (int y = 1; y<height - 1; y++)
  for (int x = 1; x<width - 1; x++)
  {
   if (originalInput[x + y*width].idepth <= 0) continue;

   if (my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0) continue;
   float depth = 1 / originalInput[x + y*width].idepth;
   float depth4 = depth*depth; depth4 *= depth4;

   if (originalInput[x + y*width].idepth_var * depth4 > my_scaledTH)
    continue;
   if (originalInput[x + y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
    continue;
   if (my_minNearSupport > 1)
   {
    int nearSupport = 0;
    for (int dx = -1; dx<2; dx++)
     for (int dy = -1; dy<2; dy++)
     {
      int idx = x + dx + (y + dy)*width;
      if (originalInput[idx].idepth > 0)
      {
       float diff = originalInput[idx].idepth - 1.0f / depth;
       if (diff*diff < 2 * originalInput[x + y*width].idepth_var)
        nearSupport++;
      }
     }
    if (nearSupport < my_minNearSupport)
     continue;
   }
   //Sophus::Vector3f pt = (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
   Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);//wx-将深度图的像素值转换为三维空间中的点
   pointCloud[startPositionPC + vertexBufferNumPoints * 3] = pt[0];//(x*fxi + cxi) * depth;
   pointCloud[startPositionPC + vertexBufferNumPoints * 3 + 1] = pt[1];//(y*fyi + cyi) * depth;
   pointCloud[startPositionPC + vertexBufferNumPoints * 3 + 2] = pt[2];//depth;
   colors[startPositionColor + vertexBufferNumPoints * 4 + 3] = 255;
   colors[startPositionColor + vertexBufferNumPoints * 4] = originalInput[x + y*width].color[0];
   colors[startPositionColor + vertexBufferNumPoints * 4 + 1] = originalInput[x + y*width].color[1];
   colors[startPositionColor + vertexBufferNumPoints * 4 + 2] = originalInput[x + y*width].color[2];
   vertexBufferNumPoints++;
  }
 
 //std::cout << "vertexBufferNumPoints" << vertexBufferNumPoints << std::endl;
 return vertexBufferNumPoints;
}


void KeyFrameDisplay::drawCam(float lineWidth, float* color)
{
#ifdef OPENGL_USE 
 if(width == 0)
  return;

 glPushMatrix();
  Sophus::Matrix4f m = camToWorld.matrix();
  glMultMatrixf((GLfloat*)m.data());
  if(color == 0)
   glColor3f(1,0,0);
  else
   glColor3f(color[0],color[1],color[2]);
  glLineWidth(lineWidth);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
  glVertex3f(0,0,0);
  glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
  glVertex3f(0,0,0);
  glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
  glVertex3f(0,0,0);
  glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
  glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
  glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
  glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
  glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
  glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
  glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
  glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
  glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
  glEnd();
 glPopMatrix();
#endif
}
int KeyFrameDisplay::flushPC(std::ofstream* f)
{
 
 MyVertex* tmpBuffer = new MyVertex[width*height];
 int num = 0;
 //wx-outlier removal 处理(论文Semi-Dense visual Odometry for a monocular camera 第2.3节)
 for(int y=1;y<height-1;y++)
  for(int x=1;x<width-1;x++)
  {
   if(originalInput[x+y*width].idepth <= 0) continue;
   if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;
   float depth = 1 / originalInput[x+y*width].idepth;
   float depth4 = depth*depth; depth4*= depth4;
   if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
    continue;
   if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
    continue;
   if(my_minNearSupport > 1)
   {
    int nearSupport = 0;
    for(int dx=-1;dx<2;dx++)
     for(int dy=-1;dy<2;dy++)
     {
      int idx = x+dx+(y+dy)*width;
      if(originalInput[idx].idepth > 0)
      {
       float diff = originalInput[idx].idepth - 1.0f / depth;
       if(diff*diff < 2*originalInput[x+y*width].idepth_var)
        nearSupport++;
      }
     }
    if(nearSupport < my_minNearSupport)
     continue;
   }

   Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);//wx-将深度图的像素值转换为三维空间中的点
   tmpBuffer[num].point[0] = pt[0];
   tmpBuffer[num].point[1] = pt[1];
   tmpBuffer[num].point[2] = pt[2];


   tmpBuffer[num].color[3] = 100;
   tmpBuffer[num].color[2] = originalInput[x+y*width].color[0];
   tmpBuffer[num].color[1] = originalInput[x+y*width].color[1];
   tmpBuffer[num].color[0] = originalInput[x+y*width].color[2];
   num++;
  }



 for(int i=0;i<num;i++)
 {
  //f->write((const char *)tmpBuffer[i].point,3*sizeof(float));
  //float color = tmpBuffer[i].color[0] / 255.0;
  //f->write((const char *)&color,sizeof(float));
  *f << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " " << tmpBuffer[i].point[2] << " ";
  *f << tmpBuffer[i].color[0] / 255.0 << " " << tmpBuffer[i].color[1] / 255.0 << " " << tmpBuffer[i].color[2] / 255.0 << std::endl;
 }
 // *f << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " " << tmpBuffer[i].point[2] << " " << (tmpBuffer[i].color[0] / 255.0) << "\n";
 delete tmpBuffer;
 printf("Done flushing frame %d (%d points)!\n", this->id, num);
 return num;
}
void KeyFrameDisplay::drawPC(float pointSize, float alpha)
{
 refreshPC();
#ifdef OPENGL_USE
 
 
 if(!vertexBufferIdValid)
 {
  return;
 }
 GLfloat LightColor[] = {1, 1, 1, 1};
 if(alpha < 1)
 {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  LightColor[0] = LightColor[1] = 0;
  glEnable(GL_LIGHTING);
  glDisable(GL_LIGHT1);
  glLightfv (GL_LIGHT0, GL_AMBIENT, LightColor);
 }
 else
 {
  glDisable(GL_LIGHTING);
 }

 glPushMatrix();
  Sophus::Matrix4f m = camToWorld.matrix();
  glMultMatrixf((GLfloat*)m.data());
  glPointSize(pointSize);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
  glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
  glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints);
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
 glPopMatrix();



 if(alpha < 1)
 {
  glDisable(GL_BLEND);
  glDisable(GL_LIGHTING);
  LightColor[2] = LightColor[1] = LightColor[0] = 1;
  glLightfv (GL_LIGHT0, GL_AMBIENT_AND_DIFFUSE, LightColor);
 }
#endif
}


}//namepsace lsd_slam
