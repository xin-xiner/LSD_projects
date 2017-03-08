#include "preprocessHeaders.h"
#include "TrajectoryDisplay.h"
#define GL_GLEXT_PROTOTYPES 1
#include "../LSD/IOWrapper/displaySetting.h"
#include "DataStructures/Frame.h"



//using namespace lsd_slam;
namespace lsd_slam
{

 TrajectoryDisplay::TrajectoryDisplay(int r_, int g_, int b_)
  :r(r_), g(g_), b(b_)
 {
  id = 0;
  vertexBufferIdValid = false;
  glBuffersValid = false;
  totalPoints = displayedPoints = 0;
  vertexBufferNumPoints = 0;
  pointSize = 3;
 }

 TrajectoryDisplay::~TrajectoryDisplay()
 {
  if (vertexBufferIdValid)
  {
#ifdef OPENGL_USE//edit-by-wx  2015-10-30 增加了条件编译
   glDeleteBuffers(1, &vertexBufferId);
#endif
   vertexBufferIdValid = false;
  }
 }

 void TrajectoryDisplay::pushback(Eigen::Vector3f& frame)
 {
  trajectory.push_back(frame);
  glBuffersValid = false;
 }
 void TrajectoryDisplay::refreshPC()
 {
  if (glBuffersValid && ( numRefreshedAlready > 10)) return;
#ifdef OPENGL_USE
  numRefreshedAlready++;
  glBuffersValid = true;

  // delete old vertex buffer
  if (vertexBufferIdValid)
  {
   glDeleteBuffers(1, &vertexBufferId);
   vertexBufferIdValid = false;
  }

#endif
  // if there are no vertices, done!
  if (trajectory.size() == 0)
   return;
#ifdef OPENGL_USE
  //make data
  TrajecVertex* tmpBuffer = new TrajecVertex[trajectory.size()];
#endif
  // data is directly in ros message, in correct format.
  vertexBufferNumPoints = 0;
#ifdef OPENGL_USE
  int total = 0, displayed = 0;//wx-outlier removal 处理(论文Semi-Dense visual Odometry for a monocular camera 第2.3节)(与flushPC中的操作重复,不太合理)
  for (int i = 1; i<trajectory.size(); i++)
  {
    total++;
    tmpBuffer[vertexBufferNumPoints].point[0] = trajectory[i].x();
    tmpBuffer[vertexBufferNumPoints].point[1] = trajectory[i].y();//edit-by-wx 2015-11-10 为了使openGL的显示结果是正的这里给y轴和z轴取了负数
    tmpBuffer[vertexBufferNumPoints].point[2] = trajectory[i].z();
    tmpBuffer[vertexBufferNumPoints].color[3] = 255;
    tmpBuffer[vertexBufferNumPoints].color[2] = b;
    tmpBuffer[vertexBufferNumPoints].color[1] = g;
    tmpBuffer[vertexBufferNumPoints].color[0] = r;
    vertexBufferNumPoints++;
    displayed++;
  }
  totalPoints = total;
  displayedPoints = displayed;
  // create new ones, static
  vertexBufferId = 0;
  glGenBuffers(1, &vertexBufferId);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
  glBufferData(GL_ARRAY_BUFFER, sizeof(TrajecVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);//wx-将创建好的点云数据传递给openGL的数据结构中
  vertexBufferIdValid = true;
  delete[] tmpBuffer;
#endif
 }

 void TrajectoryDisplay::drawTrac()
 {
  refreshPC();
#ifdef OPENGL_USE
  if (!vertexBufferIdValid)
  {
   return;
  }
  GLfloat LightColor[] = { 1, 1, 1, 1 };
  glDisable(GL_LIGHTING);
  glPushMatrix();
  glPointSize(pointSize);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
  glVertexPointer(3, GL_FLOAT, sizeof(TrajecVertex), 0);
  glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(TrajecVertex), (const void*)(3 * sizeof(float)));
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints);
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glPopMatrix();
#endif
 }


}//namepsace lsd_slam