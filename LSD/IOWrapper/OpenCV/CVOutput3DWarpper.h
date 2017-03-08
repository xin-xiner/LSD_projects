#pragma once
#include "preprocessHeaders.h"
#include "../Output3DWrapper.h"
#include "../KeyFrameDisplay.h"
#include "../KeyFrameGraphDisplay.h"
#include "../displaySetting.h"
#include "../TrajectoryDisplay.h"
namespace lsd_slam
{
 class Frame;
 class KeyFrameGraph;
 class KeyFrameGraphDisplay;
 class KeyFrameDisplay;
 class Frame;
 struct GraphConstraint;
 struct GraphFramePose;
 struct GraphMsg
 {
  int numConstraints;
  int numFrames;
  std::vector<GraphConstraint> constraintsData;
  std::vector<GraphFramePose> frameData;
 };
 struct FrameMsg
 {
  float fx, fy, cx, cy;
  int width, height, id;
 };
 //wx-有着单独线程，调用keyframeGraphDisplay，负责显示、输出、读取关键帧文件，三维点云等
 //使用publish方法，将需要显示的信息传递给显示线程进行显示
 class CVOutput3DWrapper : public Output3DWrapper
 {
 public:
  CVOutput3DWrapper(int width, int height, bool bOnlyDisplay = false, std::string kfPath = "", int cnt = 0);//edit-by-wx 2015-11-06 在构造函数中增加了长宽的信息，为向界面更新点云提供信息
  ~CVOutput3DWrapper();
  friend void display(CVOutput3DWrapper* out);


  void publishKeyframeGraph(KeyFrameGraph* graph);
  // publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
  void publishKeyframe(Frame* kf);
  // published a tracked frame that did not become a keyframe (yet; i.e. has no depth data)
  void publishTrackedFrame(Frame* kf);
  // publishes graph and all constraints, as well as updated KF poses.
  void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier);
  void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier);
  void publishTrajectoryGTIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier);
  void publishDebugInfo(Eigen::Matrix<float, 20, 1>& data);
  //create-by-wx 2015-10-28 为了保存关键帧和点云增加的函数，
  void saveKeyframeGraph();
  void savePointCloud();
  void savePointThread();//create-by-wx-try 2015-11-4 
  void refreshPoint();
  //guo-wrapper thread by OpenGL
#ifdef OPENGL_USE
   // void openglThread();
   // int DrawGLScene(GLvoid);         // Here's Where We Do All The Drawing
   // LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM); // Declaration For WndProc
   // GLvoid ReSizeGLScene(GLsizei width, GLsizei height);
   // int InitGL(GLvoid);
   // void DrawPoints();
   // void DrawCameras();
   //
   //
   //private:
   // HDC   hDC ;  // Private GDI Device Context
   // HGLRC  hRC ;  // Permanent Rendering Context
   // HWND  hWnd ;  // Holds Our Window Handle
   // HINSTANCE hInstance;  // Holds The Instance Of The Application
   //
   // bool    _initialized ;
   // bool    willquit;
   //
   // // key control
   // bool keys[256];   // Array Used For The Keyboard Routine
   // GLfloat xtran , ytran , ztran ;
   // GLfloat move_step ;
   //
   // // mouse control
   // POINT mouse_position;
   // int   mouse_status ;
   // GLfloat xrot ;
   // GLfloat yrot ;
   // GLfloat zrot ;
   //
   //
   // // 3D data to be visualized
   // WGLViewerPoint*  g_point_cloud;
   // int     g_point_num;
   // WGLViewerCameraPose* g_cam_trajectory;
   // int     g_cam_num;
   // WGLViewerJoints  g_joints;
   //
   //
   //
   // // displays kf-graph
   // KeyFrameGraphDisplay* graphDisplay;
   //
   // // displays only current keyframe (which is not yet in the graph).
   // KeyFrameDisplay* currentCamDisplay;
   //
   // // meddle mutex
   // boost::mutex meddleMutex;
#endif
 private:
  // displays kf-graph
  KeyFrameGraphDisplay *graphDisplay;
  // displays only current keyframe (which is not yet in the graph).
  KeyFrameDisplay* currentCamDisplay;
  TrajectoryDisplay* trajectoryDisplay;
  TrajectoryDisplay* trajectoryGTDisplay;
  // meddle mutex
  boost::mutex meddleMutex;
  bool resetRequested;
  bool bSaveDisplay;
  // for saving stuff
  std::string save_folder;
  double localMsBetweenSaves;
  double simMsBetweenSaves;
  double lastSaveTime;
  double lastCamTime;
  int lastCamID;


  //graph data
  GraphMsg graphMsg;
  FrameMsg frameMsg;

  boost::thread thread_opengl;
  void openGLThread();
  void draw(void);
  void saveAllGraphDisplay();
  void operator()(void){ draw(); };
  //create-by-wx 2015-11-06 加入关键帧时向界面输出点云
  //show cloud
 public:
  float* pointCloud;//按照x，y，z顺序存放的点云的位置信息，这个内存由CVoutputWarpper管理，用new新建，每次更新后delete
  float* pointCloudColor; //按照B，G，R，A的顺序存放的颜色信息
     int numberOfpoints;//点云中总共有的点
  int width;
  int height;//每帧图片的长和宽，用来在refreshPointCloud时给pointCloud申请空间，这里假设了每帧的长宽不变
  bool pointCloudrefreshed;
  void refreshPointCloud();//edit-by-wx 2015-11-10 用来更新点云数据，给手机输出
  boost::mutex pointCloudMutex;
 };
}

extern float moveSpeed;