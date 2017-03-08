#include "preprocessHeaders.h"
#include "CVOutput3DWarpper.h"
#include "util/SophusUtil.h"
#include "util/settings.h"
#include "../../DataStructures/Frame.h"
#include "../../GlobalMapping/KeyFrameGraph.h"
#include "../../GlobalMapping/g2oTypeSim3Sophus.h"
#include "../KeyFrameGraphDisplay.h"
#include "../KeyFrameDisplay.h"


/////////////////////////////OPENGL PARAM////////////////////////////////
float moveSpeed = 0.001f;
#ifdef OPENGL_USE//edit-by-wx  2015-10-30 增加了条件编译
HDC   hDC = NULL;  // Private GDI Device Context//
HGLRC  hRC = NULL;  // Permanent Rendering Context
HWND  hWnd = NULL;  // Holds Our Window Handle
HINSTANCE hInstance;  // Holds The Instance Of The Application



// key control
bool keys[256];   // Array Used For The Keyboard Routine
GLfloat xtran = 0, ytran = 0, ztran = -2000*moveSpeed;
GLfloat move_step = 0.1f;
// mouse control
POINT mouse_position;
int   mouse_status = 0;
GLfloat xrot = 0;
GLfloat yrot = 0;
GLfloat zrot = 0;
#endif
////////////////////////////////////////////////////////////////////////////
bool    _initialized = false;
bool    willquit = false;

#ifdef OPENGL_USE//edit-by-wx   2015-10-30 增加了条件编译
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM); // Declaration For WndProc
GLvoid ReSizeGLScene(GLsizei width, GLsizei height)  // Resize And Initialize The GL Window
{
 if (height == 0)          // Prevent A Divide By Zero By
 {
  height = 1;          // Making Height Equal One
 }
 glViewport(0, 0, width, height);      // Reset The Current Viewport
 glMatrixMode(GL_PROJECTION);      // Select The Projection Matrix
 glLoadIdentity();         // Reset The Projection Matrix
 // Calculate The Aspect Ratio Of The Window
 gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
 glMatrixMode(GL_MODELVIEW);       // Select The Modelview Matrix
 glLoadIdentity();         // Reset The Modelview Matrix
}
int InitGL(GLvoid)          // All Setup For OpenGL Goes Here
{
 glShadeModel(GL_SMOOTH);       // Enable Smooth Shading
 glClearColor(0.0f, 0.0f, 0.0f, 0.5f);    // Black Background
 glClearDepth(1.0f);         // Depth Buffer Setup
 glEnable(GL_DEPTH_TEST);       // Enables Depth Testing
 glDepthFunc(GL_LEQUAL);        // The Type Of Depth Testing To Do
 glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations
 GLenum err = glewInit();//guo-使用glgenbuffer初始化glew
 if (GLEW_OK != err)
 {
  /* Problem: glewInit failed, something is seriously wrong. */
  printf("Error: %s\n", glewGetErrorString(err));
  return FALSE;
 }
 return TRUE;          // Initialization Went OK
}
GLvoid KillGLWindow(GLvoid)        // Properly Kill The Window
{
 if (hRC)           // Do We Have A Rendering Context?
 {
  if (!wglMakeCurrent(NULL, NULL))     // Are We Able To Release The DC And RC Contexts?
  {
   MessageBox(NULL, "Release Of DC And RC Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
  }
  if (!wglDeleteContext(hRC))      // Are We Able To Delete The RC?
  {
   MessageBox(NULL, "Release Rendering Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
  }
  hRC = NULL;          // Set RC To NULL
 }
 if (hDC && !ReleaseDC(hWnd, hDC))     // Are We Able To Release The DC
 {
  MessageBox(NULL, "Release Device Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
  hDC = NULL;          // Set DC To NULL
 }
 if (hWnd && !DestroyWindow(hWnd))     // Are We Able To Destroy The Window?
 {
  MessageBox(NULL, "Could Not Release hWnd.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
  hWnd = NULL;          // Set hWnd To NULL
 }
 if (!UnregisterClass("OpenGL", hInstance))   // Are We Able To Unregister Class
 {
  MessageBox(NULL, "Could Not Unregister Class.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
  hInstance = NULL;         // Set hInstance To NULL
 }
}
BOOL CreateGLWindow(char* title, int width, int height, int bits)
{
 GLuint  PixelFormat;   // Holds The Results After Searching For A Match
 WNDCLASS wc;      // Windows Class Structure
 DWORD  dwExStyle;    // Window Extended Style
 DWORD  dwStyle;    // Window Style
 RECT  WindowRect;    // Grabs Rectangle Upper Left / Lower Right Values
 WindowRect.left = (long)0;   // Set Left Value To 0
 WindowRect.right = (long)width;  // Set Right Value To Requested Width
 WindowRect.top = (long)0;    // Set Top Value To 0
 WindowRect.bottom = (long)height;  // Set Bottom Value To Requested Height
 hInstance = GetModuleHandle(NULL);    // Grab An Instance For Our Window
 wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC; // Redraw On Size, And Own DC For Window.
 wc.lpfnWndProc = (WNDPROC)WndProc;     // WndProc Handles Messages
 wc.cbClsExtra = 0;         // No Extra Window Data
 wc.cbWndExtra = 0;         // No Extra Window Data
 wc.hInstance = hInstance;       // Set The Instance
 wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);   // Load The Default Icon
 wc.hCursor = LoadCursor(NULL, IDC_ARROW);   // Load The Arrow Pointer
 wc.hbrBackground = NULL;         // No Background Required For GL
 wc.lpszMenuName = NULL;         // We Don't Want A Menu
 wc.lpszClassName = "OpenGL";        // Set The Class Name
 if (!RegisterClass(&wc))         // Attempt To Register The Window Class
 {
  MessageBox(NULL, "Failed To Register The Window Class.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;           // Return FALSE
 }
 dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;   // Window Extended Style
 dwStyle = WS_OVERLAPPEDWINDOW;       // Windows Style
 AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);  // Adjust Window To True Requested Size
 // Create The Window
 if (!(hWnd = CreateWindowEx(dwExStyle,       // Extended Style For The Window
  "OpenGL",       // Class Name
  title,        // Window Title
  dwStyle |       // Defined Window Style
  WS_CLIPSIBLINGS |     // Required Window Style
  WS_CLIPCHILDREN,     // Required Window Style
  0, 0,        // Window Position
  WindowRect.right - WindowRect.left, // Calculate Window Width
  WindowRect.bottom - WindowRect.top, // Calculate Window Height
  NULL,        // No Parent Window
  NULL,        // No Menu
  hInstance,       // Instance
  NULL)))        // Dont Pass Anything To WM_CREATE
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Window Creation Error.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 static PIXELFORMATDESCRIPTOR pfd =    // pfd Tells Windows How We Want Things To Be
 {
  sizeof(PIXELFORMATDESCRIPTOR),    // Size Of This Pixel Format Descriptor
  1,           // Version Number
  PFD_DRAW_TO_WINDOW |      // Format Must Support Window
  PFD_SUPPORT_OPENGL |      // Format Must Support OpenGL
  PFD_DOUBLEBUFFER,       // Must Support Double Buffering
  PFD_TYPE_RGBA,        // Request An RGBA Format
  bits,          // Select Our Color Depth
  0, 0, 0, 0, 0, 0,       // Color Bits Ignored
  0,           // No Alpha Buffer
  0,           // Shift Bit Ignored
  0,           // No Accumulation Buffer
  0, 0, 0, 0,         // Accumulation Bits Ignored
  16,           // 16Bit Z-Buffer (Depth Buffer)  
  0,           // No Stencil Buffer
  0,           // No Auxiliary Buffer
  PFD_MAIN_PLANE,        // Main Drawing Layer
  0,           // Reserved
  0, 0, 0          // Layer Masks Ignored
 };
 if (!(hDC = GetDC(hWnd)))       // Did We Get A Device Context?
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Can't Create A GL Device Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 if (!(PixelFormat = ChoosePixelFormat(hDC, &pfd))) // Did Windows Find A Matching Pixel Format?
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Can't Find A Suitable PixelFormat.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 if (!SetPixelFormat(hDC, PixelFormat, &pfd))  // Are We Able To Set The Pixel Format?
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Can't Set The PixelFormat.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 if (!(hRC = wglCreateContext(hDC)))    // Are We Able To Get A Rendering Context?
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Can't Create A GL Rendering Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 if (!wglMakeCurrent(hDC, hRC))     // Try To Activate The Rendering Context
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Can't Activate The GL Rendering Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 ShowWindow(hWnd, SW_SHOW);      // Show The Window
 SetForegroundWindow(hWnd);      // Slightly Higher Priority
 SetFocus(hWnd);         // Sets Keyboard Focus To The Window
 ReSizeGLScene(width, height);     // Set Up Our Perspective GL Screen
 if (!InitGL())         // Initialize Our Newly Created GL Window
 {
  KillGLWindow();        // Reset The Display
  MessageBox(NULL, "Initialization Failed.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
  return FALSE;        // Return FALSE
 }
 return TRUE;         // Success
}

LRESULT CALLBACK WndProc(HWND hWnd,   // Handle For This Window
 UINT uMsg,   // Message For This Window
 WPARAM wParam,   // Additional Message Information
 LPARAM lParam)   // Additional Message Information
{
 double speed;
 static long width = 640;
 static long height = 480;
 switch (uMsg)         // Check For Windows Messages
 {
 case WM_CLOSE:        // Did We Receive A Close Message?
 {
  willquit = true;
  return 0;        // Jump Back
 }
 case WM_KEYDOWN:       // Is A Key Being Held Down?
 {
  keys[wParam] = TRUE;     // If So, Mark It As TRUE
  return 0;        // Jump Back
 }
 case WM_KEYUP:        // Has A Key Been Released?
 {
  keys[wParam] = FALSE;     // If So, Mark It As FALSE
  return 0;        // Jump Back
 }
 case WM_SIZE:        // Resize The OpenGL Window
 {
  ReSizeGLScene(LOWORD(lParam), HIWORD(lParam));  // LoWord=Width, HiWord=Height
  width = LOWORD(lParam);
  height = HIWORD(lParam);
  return 0;        // Jump Back
 }
 case WM_RBUTTONDOWN:
  mouse_status = 3;
  mouse_position.x = LOWORD(lParam);
  mouse_position.y = HIWORD(lParam);
  return 0;
 case WM_LBUTTONDOWN:
  mouse_status = 1;
  mouse_position.x = LOWORD(lParam);
  mouse_position.y = HIWORD(lParam);
  return 0;
 case WM_LBUTTONUP:
  mouse_status = 0;
  return 0;
 case WM_RBUTTONUP:
  mouse_status = 0;
  return 0;
 case WM_MOUSEMOVE:
  if (mouse_status == 0) 
   return 0;
  if (mouse_status == 1)//视角表换
  {
   POINT newpt;
   newpt.x = LOWORD(lParam);
   newpt.y = HIWORD(lParam);
   double speed = 0.005f;
   if (keys[VK_SHIFT])
    speed = 0.001;
   if ( (newpt.y - mouse_position.y)== 0 || abs((newpt.x - mouse_position.x) / (newpt.y - mouse_position.y))>0.8 )//edit-by-wx 2015-11-13 增加拖动改变z轴的操作
   {
    yrot += (newpt.x - mouse_position.x) * speed;
   }
   if ((newpt.x - mouse_position.x) == 0 || abs((newpt.y - mouse_position.y) / (newpt.x - mouse_position.x)) > 0.8)
   {
    xrot -= (newpt.y - mouse_position.y) * speed;
   }
   mouse_position = newpt;
  }
  if (mouse_status == 2)//平移
  {
   POINT newpt;
   newpt.x = LOWORD(lParam);
   newpt.y = HIWORD(lParam);
   double speed = moveSpeed;
   if (keys[VK_SHIFT])
    speed = moveSpeed/20;
   xtran += (newpt.x - mouse_position.x) * speed;
   ytran += (newpt.y - mouse_position.y) * speed;
   mouse_position = newpt;
  }
  if (mouse_status == 3)//围绕光线旋转
  {
   POINT newpt;
   newpt.x = LOWORD(lParam);
   newpt.y = HIWORD(lParam);
   double speed = 0.005f;
   if (keys[VK_SHIFT])
    speed = 0.001;
   float rotateDis = 0;
   if (newpt.x<0.3*width || newpt.x>0.7*width)
   {
    rotateDis = abs( newpt.y - mouse_position.y) * speed;
    if ((newpt.y - mouse_position.y) < 0)
     rotateDis = -rotateDis;
    if (newpt.x < width / 2)
     rotateDis = -rotateDis;
   }
   else
   {
    rotateDis = abs(newpt.x - mouse_position.x) * speed;
    if ((newpt.x - mouse_position.x) < 0)
     rotateDis = -rotateDis;
    if (newpt.y > height / 2)
     rotateDis = -rotateDis;
   }
   zrot += rotateDis;
   mouse_position = newpt;
  }
  return 0;
 case WM_MBUTTONDOWN:
  mouse_status = 2;
  mouse_position.x = LOWORD(lParam);
  mouse_position.y = HIWORD(lParam);
  return 0;
 case WM_MBUTTONUP:
  mouse_status = 0;
  return 0;
 case WM_MOUSEWHEEL:
  int direction = (short)HIWORD(wParam);
  double speed = 0.5f;
  if (keys[VK_SHIFT]) 
   speed = 0.03f;
  if (direction>0)
   ztran += speed;
  else
   ztran -= speed;
  return 0;
 
 }
 // Pass All Unhandled Messages To DefWindowProc
 return DefWindowProc(hWnd, uMsg, wParam, lParam);
}
int DrawGLScene(GLvoid)         // Here's Where We Do All The Drawing
{
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Screen And Depth Buffer
 glLoadIdentity();         // Reset The Current Modelview Matrix
 //glRotatef(180, 0, 1, 0);
 /*glTranslatef(0, 0, 0);
 glRotatef(180, 1, 0, 0);
 glRotatef(xrot, 1, 0, 0);
 glRotatef(yrot, 0, 1, 0);
 glRotatef(zrot, 0, 0, 1);
 glTranslatef(xtran, ytran, -ztran);*/
 //created-by-wx 2015-11-13 修改了显示的方式，之前是改变模型来改变视角，现在改变视角完成。
 //OPENGL的视角切换函数需要三组参数，分别是摄像机位置，摄像机视点中心，摄像机姿态。
 GLfloat viewDis = -2;
 static Eigen::Vector3f cameraCordinateZ(0, 0, -1);//初始的摄像机坐标系
 static Eigen::Vector3f cameraCordinateX(-1, 0, 0);
 static Eigen::Vector3f cameraCordinateY(0, -1, 0);
 static Eigen::Vector3f cameraPosition(0, 0, viewDis);//初始摄像机的位置
 static float xrotPre = xrot, yrotPre = yrot, zrotPre = zrot;//rotPre用来记录上一次的旋转角度，用来完成增量旋转
 float refXrot = xrot - xrotPre;//求本次旋转的增量
 float refYrot = yrot - yrotPre;
 float refZrot = zrot - zrotPre;
 
 Eigen::Vector3f temp = cameraCordinateX*sin(refXrot / 2);//围绕摄像机坐标系的X轴，旋转摄像机及摄像机坐标系
 Eigen::Quaternion<float> refXrotation(cos(refXrot / 2), temp(0), temp(1), temp(2));
 cameraPosition = refXrotation.toRotationMatrix()*cameraPosition;
 cameraCordinateZ = refXrotation.toRotationMatrix()*cameraCordinateZ;
 cameraCordinateY = refXrotation.toRotationMatrix()*cameraCordinateY;
 temp = cameraCordinateY*sin(refYrot / 2);//围绕摄像机坐标系的Y轴，旋转摄像机及摄像机坐标系
 Eigen::Quaternion<float> refYrotation(cos(refYrot / 2), temp(0), temp(1), temp(2));
 cameraPosition = refYrotation.toRotationMatrix()*cameraPosition;
 cameraCordinateZ = refYrotation.toRotationMatrix()*cameraCordinateZ;
 cameraCordinateX = refYrotation.toRotationMatrix()*cameraCordinateX;

                 //围绕摄像机坐标系Z轴(光线方向)，旋转摄像机及摄像机坐标系
 Eigen::Vector3f rayDirection = cameraCordinateZ*sin(refZrot / 2);
 Eigen::Quaternion<float> cameraPlaneRotate(cos(refZrot / 2), rayDirection(0), rayDirection(1), rayDirection(2));
 cameraCordinateY = cameraPlaneRotate.toRotationMatrix()*cameraCordinateY;
 cameraCordinateY.normalize();
 cameraCordinateX = cameraPlaneRotate.toRotationMatrix()*cameraCordinateX;
 cameraCordinateX.normalize();
            //摄像机的平移，平移的方向应当为摄像机坐标系下的方向
 Eigen::Vector3f viewCenter = cameraCordinateX*xtran + cameraCordinateY*ytran - cameraCordinateZ*ztran;//将摄像机坐标系下的平移转换到世界坐标系下
 GLfloat cameraPositionX = cameraPosition(0) + viewCenter(0);
 GLfloat cameraPositionY = cameraPosition(1) + viewCenter(1);
 GLfloat cameraPositionZ = cameraPosition(2) + viewCenter(2);
 GLfloat cameraDirectionX = cameraCordinateY(0);
 GLfloat cameraDirectionY = cameraCordinateY(1);
 GLfloat cameraDirectionZ = cameraCordinateY(2);
 GLfloat viewCenterX = viewCenter(0);
 GLfloat viewCenterY = viewCenter(1);
 GLfloat viewCenterZ = viewCenter(2);
 xrotPre = xrot; yrotPre = yrot; zrotPre = zrot;//更新记录增量前值的变量
 gluLookAt(cameraPositionX, cameraPositionY, cameraPositionZ, viewCenterX, viewCenterY, viewCenterZ, cameraDirectionX, cameraDirectionY, cameraDirectionZ);//对视角进行切换


//  const GLubyte* name = glGetString(GL_VENDOR); //返回负责当前OpenGL实现厂商的名字
//  const GLubyte* biaoshifu = glGetString(GL_RENDERER); //返回一个渲染器标识符，通常是个硬件平台
//  const GLubyte* OpenGLVersion = glGetString(GL_VERSION); //返回当前OpenGL实现的版本号
//  const GLubyte* gluVersion = gluGetString(GLU_VERSION); //返回当前GLU工具库版本
//  printf("OpenGL实现厂商的名字：%s\n", name);
//  printf("渲染器标识符：%s\n", biaoshifu);
//  printf("OOpenGL实现的版本号：%s\n", OpenGLVersion);
//  printf("OGLU工具库版本：%s\n", gluVersion);
//  Draw_grid();
//  DrawCameras();
//  DrawPoints();
#if 0
 glPointSize(4);
 //glTranslatef(-1.5f,0.0f,-6.0f);      // Move Left 1.5 Units And Into The Screen 6.0
 glBegin(GL_TRIANGLES);        // Drawing Using Triangles
 glColor3f(1.0f, 0.0f, 0.0f);      // Set The Color To Red
 glVertex3f(0.0f, 1.0f, 0.0f);     // Top
 glColor3f(0.0f, 1.0f, 0.0f);      // Set The Color To Green
 glVertex3f(-1.0f, -1.0f, 0.0f);     // Bottom Left
 glColor3f(0.0f, 0.0f, 1.0f);      // Set The Color To Blue
 glVertex3f(1.0f, -1.0f, 0.0f);     // Bottom Right
 glEnd();           // Finished Drawing The Triangle
 glTranslatef(3.0f, 0.0f, 0.0f);      // Move Right 3 Units
 glColor3f(0.5f, 0.5f, 1.0f);       // Set The Color To Blue One Time Only
 glBegin(GL_QUADS);         // Draw A Quad
 glVertex3f(-1.0f, 1.0f, 0.0f);     // Top Left
 glVertex3f(1.0f, 1.0f, 0.0f);     // Top Right
 glVertex3f(1.0f, -1.0f, 0.0f);     // Bottom Right
 glVertex3f(-1.0f, -1.0f, 0.0f);     // Bottom Left
 glEnd();           // Done Drawing The Quad
#endif
 return TRUE;          // Everything Went OK
}
#endif
namespace lsd_slam
{
 
 CVOutput3DWrapper::CVOutput3DWrapper(int w,int h,bool bOnlyDisplay, std::string kfPath , int cnt)
  :width(w), height(h)
 {
  bSaveDisplay = false;
  width = w;
  height = h;
  currentCamDisplay = new KeyFrameDisplay();
  graphDisplay = new KeyFrameGraphDisplay(kfPath);
  trajectoryDisplay = new TrajectoryDisplay(255,0,0);
  trajectoryGTDisplay = new TrajectoryDisplay(0, 0, 255);
  frameMsg.width = w;
  frameMsg.height = h; 
  width = w;
  height = h;
  pointCloud = 0;
  pointCloudColor = 0;
  pointCloudrefreshed = false;
  if (bOnlyDisplay)
  {
   graphDisplay->flushPointcloud = false;
   graphDisplay->printNumbers = false;
   showCurrentCamera = false;
   showCurrentPointcloud = false;
   graphDisplay->readAll(kfPath,cnt);
  }
  
  if (!_initialized)
  {
#ifdef OPENGL_USE//edit-by-wx  2015-10-30 增加了条件编译
   thread_opengl = boost::thread(&CVOutput3DWrapper::openGLThread, this);
#else
   //thread_opengl = boost::thread(&CVOutput3DWrapper::savePointThread, this);
#endif
   _initialized = true;
  }
 }
 CVOutput3DWrapper::~CVOutput3DWrapper()
 {
  delete currentCamDisplay;
  delete graphDisplay;
  delete trajectoryDisplay;
  delete trajectoryGTDisplay;
 }
 void CVOutput3DWrapper::publishKeyframe(Frame* kf)
 {
#ifdef TXT_OUTPUT
  std::ofstream f("data/publishKeyframe.txt", std::ofstream::app);
  f << kf->id() << std::endl;
  f.close();
#endif
  boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();
  meddleMutex.lock();
  graphDisplay->addMsg(kf);
#ifndef OPENGL_USE
  refreshPointCloud();//edit-by-wx-try 2015-11-10
#endif
  meddleMutex.unlock();
  
 }
 void CVOutput3DWrapper::refreshPointCloud()//edit-by-wx 2015-11-10 用来更新点云数据，给手机输出
 {
  pointCloudMutex.lock();
  if (pointCloud != 0)//edit-by-wx 2015-11-06 当publish关键帧的时候，更新一次往界面输出的点云
   delete pointCloud;
  if (pointCloudColor != 0)
   delete pointCloudColor;
  //std::cout <<"width"<< width << std::endl;
  //std::cout << "height" << height << std::endl;
  pointCloud = new float[graphDisplay->keyframeNumber() * width * height * 3];//预留了足够的空间用来存点云，但是实际用不了这么多，因为depthMap中有很多深度缺失的值，同时在生成点云时outlier Removal操作也会删除很多点
  pointCloudColor = new float[graphDisplay->keyframeNumber() * width * height * 4];
  numberOfpoints = graphDisplay->refreshPointCloud(pointCloud, pointCloudColor);
  pointCloudrefreshed = true;
  pointCloudMutex.unlock();
 }


 void CVOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
 {
#ifdef TXT_OUTPUT
  std::ofstream f("data/publishKeyframeGraph.txt", std::ofstream::app);
  for (FramePoseStruct *pose : graph->allFramePoses)
  {
   f << pose->frameID << "--";
  }
  f << std::endl;
  f.close();
#endif
  meddleMutex.lock();
  graph->edgesListsMutex.lock();
  graphMsg.numConstraints = graph->edgesAll.size();
  graphMsg.constraintsData.resize(graphMsg.numConstraints);
  for (unsigned int i = 0; i < graph->edgesAll.size(); i++)
  {
   graphMsg.constraintsData[i].from = graph->edgesAll[i]->firstFrame->id();
   graphMsg.constraintsData[i].to = graph->edgesAll[i]->secondFrame->id();
   Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
   graphMsg.constraintsData[i].err = sqrt(err.dot(err));
  }
  graph->edgesListsMutex.unlock();
  ///////////////////////////////////////////////////
  graph->keyframesAllMutex.lock_shared();
  graphMsg.numFrames = graph->keyframesAll.size();
  graphMsg.frameData.resize(graphMsg.numFrames);
  for (unsigned int i = 0; i < graph->keyframesAll.size(); i++)
  {
   graphMsg.frameData[i].id = graph->keyframesAll[i]->id();
   memcpy(graphMsg.frameData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(), sizeof(float) * 7);
  }
  graph->keyframesAllMutex.unlock_shared();

  graphDisplay->addGraphMsg(&graphMsg);
  //refreshPointCloud();
  meddleMutex.unlock();

 }
 void CVOutput3DWrapper::publishTrackedFrame(Frame* kf)
 {
#ifdef TXT_OUTPUT
  std::ofstream f("data/publishTrackedFrame.txt", std::ofstream::app);
  f << kf->id() << std::endl;
  f.close();
#endif

  meddleMutex.lock();



  if (currentCamDisplay->id > kf->id())
  {
   printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, kf->id());
   resetRequested = true;
  }
  currentCamDisplay->setFrom(kf,false);
  //lastAnimTime = lastCamTime = msg->time;
  lastCamID = kf->id();
  meddleMutex.unlock();

 }
 void CVOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
 {
  ;
 }
 void CVOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
 {
  trajectoryDisplay->pushback(pt);
 }
 void CVOutput3DWrapper::publishTrajectoryGTIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
 {
  trajectoryGTDisplay->pushback(pt);
 }
 void CVOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1>& data)
 {
#ifdef TXT_OUTPUT
  std::ofstream f("data/publishDebugInfo.txt", std::ofstream::app);
  f << data << std::endl << std::endl;
  f.close();
#endif
 }

#ifdef OPENGL_USE
 
 void CVOutput3DWrapper::openGLThread()
 {
  
  MSG  msg;         // Windows Message Structure
  // Create Our OpenGL Window
  if (!CreateGLWindow("OpenGL Framework", 640, 480, 16))
  {
   return;         // Quit If Window Was Not Created
  }

    const GLubyte* name = glGetString(GL_VENDOR); //返回负责当前OpenGL实现厂商的名字
    const GLubyte* biaoshifu = glGetString(GL_RENDERER); //返回一个渲染器标识符，通常是个硬件平台
    const GLubyte* OpenGLVersion = glGetString(GL_VERSION); //返回当前OpenGL实现的版本号
    const GLubyte* gluVersion = gluGetString(GLU_VERSION); //返回当前GLU工具库版本
    printf("OpenGL实现厂商的名字：%s\n", name);
    printf("渲染器标识符：%s\n", biaoshifu);
    printf("OOpenGL实现的版本号：%s\n", OpenGLVersion);
    printf("OGLU工具库版本：%s\n", gluVersion);

  while (!willquit)         // Loop That Runs While done=FALSE
  {
   if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) // Is There A Message Waiting?
   {
    TranslateMessage(&msg);    // Translate The Message
    DispatchMessage(&msg);    // Dispatch The Message
   }
   else          // If There Are No Messages
   {
    float dx = 0, dy = 0;
    float alpha = -yrot * 3.14 / 180;
    float speed;
    if (keys[VK_SHIFT]) speed = 1.0f;
    else speed = 5.0f;
    if (keys['W']) { dx = 0.0f; dy = move_step; }
    if (keys['S']) { dx = 0.0f; dy = -move_step; }
    if (keys['A']) { dx = move_step; dy = 0.0f; }
    if (keys['D']) { dx = -move_step; dy = 0.0f; }
    if (keys['Z']) ytran += move_step * speed;
    if (keys['X']) ytran -= move_step * speed;
    if (keys['P'])saveAllGraphDisplay();
    dx *= speed;
    dy *= speed;
    if (dx != 0 || dy != 0)
    {
     xtran += dx * cos(alpha) + dy * sin(alpha);
     ztran += -dx * sin(alpha) + dy * cos(alpha);
    }

    DrawGLScene();
    draw();     // Draw The Scene
    SwapBuffers(hDC);

   }
//     if (bSaveDisplay)
//     {
//      saveAllGraphDisplay();
//     }
  }
  KillGLWindow();
  DestroyWindow(hWnd);
  willquit = false;
  _initialized = false;
 }
 void CVOutput3DWrapper::draw(void)
 {
  meddleMutex.lock();
  if (resetRequested)
  {
//   reset();
   resetRequested = false;
  }
  glPushMatrix();

#if 0
  if (animationPlaybackEnabled)
  {
   double tm = ros::Time::now().toSec() - animationPlaybackTime;
   if (tm > kfInt->lastTime())
   {
    animationPlaybackEnabled = false;
    tm = kfInt->lastTime();
   }
   if (tm < kfInt->firstTime())
    tm = kfInt->firstTime();
   printf("anim at %.2f (%.2f to %.2f)\n", tm, kfInt->firstTime(), kfInt->lastTime());

   kfInt->interpolateAtTime(tm);
   camera()->frame()->setFromMatrix(kfInt->frame()->matrix());


   double accTime = 0;
   for (unsigned int i = 0; i < animationList.size(); i++)
   {
    if (tm >= accTime && tm < accTime + animationList[i].duration && animationList[i].isFix)
    {
     camera()->frame()->setFromMatrix(animationList[i].frame.matrix());
     printf("fixFrameto %d at %.2f (%.2f to %.2f)\n", i, tm, kfInt->firstTime(), kfInt->lastTime());
    }
    accTime += animationList[i].duration;
   }

   accTime = 0;
   AnimationObject* lastAnimObj = 0;
   for (unsigned int i = 0; i < animationList.size(); i++)
   {
    accTime += animationList[i].duration;
    if (animationList[i].isSettings && accTime <= tm)
     lastAnimObj = &(animationList[i]);
   }
   if (lastAnimObj != 0)
   {
    absDepthVarTH = lastAnimObj->absTH;
    scaledDepthVarTH = lastAnimObj->scaledTH;
    minNearSupport = lastAnimObj->neighb;
    sparsifyFactor = lastAnimObj->sparsity;
    showKFCameras = lastAnimObj->showKeyframes;
    showConstraints = lastAnimObj->showLoopClosures;
   }
  }
#endif
  //wx-由于当前的graphDisplay中还没有包含进当前帧，故单独显示当前帧
  if (showCurrentCamera)
   currentCamDisplay->drawCam(2 * lineTesselation, 0);
  if (showCurrentPointcloud)
   currentCamDisplay->drawPC(pointTesselation, 1);
  //wx-显示keyframeGraph中的点云和摄像机,并可以选择将点云输出到文件
  if (showTrajectory)
   trajectoryDisplay->drawTrac();
  if (showTrajectoryGT)
   trajectoryGTDisplay->drawTrac();
   
   
  graphDisplay->draw();
  glPopMatrix();
  meddleMutex.unlock();
 }
#endif
 void CVOutput3DWrapper::refreshPoint()//created-by-wx-try 考虑删掉这几个函数，现在用不到了
 {
  /*currentCamDisplay->refreshPC();
  graphDisplay->draw();*/
  refreshPointCloud();
 }
 void CVOutput3DWrapper::savePointThread()
 {
  while (1)         // Loop That Runs While done=FALSE
  {
   refreshPoint();
  }
 }
 void CVOutput3DWrapper::saveAllGraphDisplay()
 {
  graphDisplay->saveAll();
 }
//create-by-wx 2015-10-28 添加这两个函数，用来保存关键帧和点云

 void CVOutput3DWrapper::saveKeyframeGraph()
 {
  graphDisplay->saveAll();
 }
 void CVOutput3DWrapper::savePointCloud()
 {
  graphDisplay->savePointCloud();
 }

}
