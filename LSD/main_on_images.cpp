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
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/
//#include "LiveSLAMWrapper.h"
#include "preprocessHeaders.h"
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"
#include "IOWrapper/OpenCV/CVOutput3DWarpper.h"
#ifdef ROS
#include <dirent.h>
#endif
#include "util/Undistorter.h"
#include "IOWrapper/displaySetting.h"
#ifdef ROS
#include <ros/package.h>
#endif
#include <opencv2\video\video.hpp>
#include "../SLAM_windows/RGBDSLAM.h"
#include "tracking\DisField\disField.h"
#ifdef ROS
std::string &ltrim(std::string &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	return s;
}
std::string &rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	return s;
}
std::string &trim(std::string &s) {
	return ltrim(rtrim(s));
}
int getdir(std::string dir, std::vector<std::string> &files)
{
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL)
	{
		return -1;
	}
	while ((dirp = readdir(dp)) != NULL) {
		std::string name = std::string(dirp->d_name);
		if (name != "." && name != "..")
			files.push_back(name);
	}
	closedir(dp);

	std::sort(files.begin(), files.end());
	if (dir.at(dir.length() - 1) != '/') dir = dir + "/";
	for (unsigned int i = 0; i<files.size(); i++)
	{
		if (files[i].at(0) != '/')
			files[i] = dir + files[i];
	}
	return files.size();
}
int getFile(std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());
	if (f.good() && f.is_open())
	{
		while (!f.eof())
		{
			std::string l;
			std::getline(f, l);
			l = trim(l);
			if (l == "" || l[0] == '#')
				continue;
			files.push_back(l);
		}
		f.close();
		size_t sp = source.find_last_of('/');
		std::string prefix;
		if (sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0, sp);
		for (unsigned int i = 0; i<files.size(); i++)
		{
			if (files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}
		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}
}
#endif
using namespace lsd_slam;

#define RGBDTUM
#ifdef RGBDTUM
#include <../SLAM_windows/TUM RGBD/FileOperator.h>
#endif
enum InputType
{
	imageFile = 0,
	videoFile = 1,
	camera = 2
};
InputType inputType = InputType::imageFile;
std::string calibFile = "";
int onlyDisplay = 0;
std::string dataPath("");
std::string keyFramePath("");
int kefFrameCnt = 0;
int startFramID = 0;//wx-debug 160 for far to debug//100 for near
void parseCommand(int argc, char** argv);
int main(int argc, char** argv)
{
	parseCommand(argc, argv);
	MatVis::Vis();
#ifdef ROS
	ros::init(argc, argv, "LSD_SLAM");
	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);
	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);
	packagePath = ros::package::getPath("lsd_slam_core") + "/";


	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;
	if (ros::param::get("~calib", calibFile))
	{
		undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
		ros::param::del("~calib");
	}
	if (undistorter == 0)
	{
		printf("need camera calibration file! (set using _calib:=FILE)\n");
		exit(0);
	}
#else
	Undistorter* undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
#endif
	//int w = undistorter->getOutputWidth();
	//int h = undistorter->getOutputHeight();
	int w = undistorter->crop_width;
	int h = undistorter->crop_height;
	int w_inp = undistorter->getInputWidth();
	int h_inp = undistorter->getInputHeight();
	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
	std::cout << K;
	// make output wrapper. just set to zero if no output is required.
#ifdef ROS
	Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(w, h);
#else
	//  bool onlyDisplay = true;//true for result display with saved keyframes
	//false for tracking and display at the save 




	Output3DWrapper *outputWrapper = new CVOutput3DWrapper(w, h, onlyDisplay, keyFramePath, kefFrameCnt);
	((CVOutput3DWrapper*)outputWrapper)->width = w;
	((CVOutput3DWrapper*)outputWrapper)->height = h;
	while (onlyDisplay)
	{
		Sleep(100000);
	}
#endif

	// make slam system
	SlamSystem* system = new SlamSystem(w, h, K, doSlam, undistorter);
	system->setVisualization(outputWrapper);
#ifdef FILE
#ifdef DATATYPE_1
	double hz = 0;
	//  std::string source(argv[4]);
	std::string framelist = dataPath + "framelist.txt";
	std::ifstream framefile(framelist.c_str());
	if (!framefile.good())
	{
		framefile.close();
		printf("can't find the frame file\n");
		return 0;
	}
	printf("...found file %s\n", argv[2]);
	int frameCnt = 0;
	std::vector<std::string> files;
	std::string fileName;
	while (!framefile.eof())
	{
		getline(framefile, fileName);
		files.push_back(dataPath + fileName);
	}

#elif  DATATYPE_2
	double hz = 0;
	std::string source;
	std::vector<std::string> files;
	std::string dirc("D:/stereoPara/pano_data/5.11/25sup/framest/");
	files.resize(3000);
	char str[30];
	for (int i = 400; i < 1800; i++)
	{
		sprintf(str, "%d.jpg", i);
		files[i - 400] = dirc + str;
	}
#else
	std::string makeImageName(int frameID);
	double hz = 0;
	std::vector<std::string> files;
	for (int i = 1; i <= 2702; i++)
	{
		files.push_back(dataPath + makeImageName(i));
	}
#endif

	cv::Mat image = cv::Mat(h, w, CV_8U);
	int runningIDX = 0;
	float fakeTimeStamp = 0;
#ifdef ROS
	ros::Rate r(hz);
#endif
	for (unsigned int i = 0; i<files.size(); i++)//edit-by-wx-debug  2015-10-30 为了方便调试，只算前30帧
	{
		cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		if (imageDist.rows != h_inp || imageDist.cols != w_inp)
		{
			if (imageDist.rows * imageDist.cols == 0)
				printf("failed to load image %s! skipping.\n", files[i].c_str());
			else
				printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
				files[i].c_str(),
				w, h, imageDist.cols, imageDist.rows);
			continue;
		}
		assert(imageDist.type() == CV_8U);
		undistorter->undistort(imageDist, image);
		assert(image.type() == CV_8U);
		if (runningIDX == 0)
			system->randomInit(image.data, fakeTimeStamp, runningIDX);
		else
			system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
#ifndef OPENGL_USE
		if (((CVOutput3DWrapper*)outputWrapper)->pointCloudrefreshed)
		{

			CVOutput3DWrapper* output = (CVOutput3DWrapper*)outputWrapper;

			std::ofstream testFile("testLog.xyz");
			output->pointCloudMutex.lock();
			for (int i = 0; i<output->numberOfpoints; i++)
			{
				testFile << output->pointCloud[i * 3] << " " << output->pointCloud[i * 3 + 1] << " " << output->pointCloud[i * 3 + 2] << " ";
				testFile << output->pointCloudColor[i * 4] / 255.0 << " " << output->pointCloudColor[i * 4 + 1] / 255.0 << " " << output->pointCloudColor[i * 4 + 2] / 255.0 << std::endl;
			}
			output->pointCloudrefreshed = false;
			output->pointCloudMutex.unlock();
		}
#endif
		runningIDX++;
		fakeTimeStamp += 0.03;
		printf("//////////////////////////////\n ");
		printf("//////next frame %d/////// \n ", i);
		printf("//////////////////////////// \n ");
#ifdef ROS
		if (hz != 0)
			r.Sleep();
#endif
		if (fullResetRequested)
		{
			printf("FULL RESET!\n");
			delete system;
			system = new SlamSystem(w, h, K, doSlam);
			system->setVisualization(outputWrapper);
			fullResetRequested = false;
			runningIDX = 0;
		}
#ifdef ROS
		ros::spinOnce();
		if (!ros::ok())
			break;
#endif
	}
	system->finalize();
	system->saveKeyframeGraph();//edit-by-wx  2015-10-30 为了与手机端程序同步，增加了显式输出关键帧和点云的操作
	system->savePointCloud();

	while (true)
	{
		Sleep(100);
	}

	delete system;
	delete undistorter;
	delete outputWrapper;
#endif


	if (inputType == InputType::camera || inputType == InputType::videoFile)
	{
		int runningIDX = 0;
		float fakeTimeStamp = 0;
		double hz = 0;

		cv::VideoCapture capture;
		if (inputType == InputType::camera)
		{
			capture.open(0);
		}
		else
		{
			capture.open(dataPath);
		}
		if (!capture.isOpened())
		{
			std::cout << "camera open error" << std::endl;
			exit(0);
		}
		cv::Mat imageDist;
		cv::Mat image;
		char key = ' ';

		int frame_count = 0;

		while (key != 's')//wx 按s键终止重建循环
		{
			capture >> imageDist;
			cv::resize(imageDist, imageDist, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
			if (imageDist.empty())break;
			cv::cvtColor(imageDist, image, CV_BGR2GRAY);
			if (image.rows != h_inp || image.cols != w_inp)
			{
				std::cout << "image size error" << std::endl;
				continue;
			}
			
			//cv::Rect myROI(cv::Point(320, 60), cv::Point((320 + 1280), (60+960)));
			//cv::Rect myROI(60, 320, 960, 1280);
			//cv::imwrite("frame.jpg", imageDist);
			assert(image.type() == CV_8U);
			undistorter->undistort(image, image);
			assert(image.type() == CV_8U);

			undistorter->undistort(imageDist, imageDist);
			
			//cv::imwrite("rect_frame.jpg", imageDist);
			cv::namedWindow("frame", 0);
			cv::imshow("frame", image);
			cv::waitKey(0);
			std::stringstream sst;
			sst << frame_count << ".bmp";
			std::string rgbImgPath = sst.str();
			frame_count++;
			if (runningIDX == startFramID)
			{
				system->randomInit(image.data, fakeTimeStamp, runningIDX, imageDist.data, rgbImgPath);
			}
			else
			{
				if (runningIDX < startFramID)
				{
					runningIDX++;
					continue;
				}
				system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp, imageDist.data,0, rgbImgPath);
			}
			analyzer++;
			runningIDX++;
			fakeTimeStamp += 0.03;
			//printf("//////////////////////////////\n ");
			printf("//////next frame %d/////// \n ", runningIDX);
			//printf("//////////////////////////// \n ");

			if (fullResetRequested)
			{
				printf("FULL RESET!\n");
				delete system;
				system = new SlamSystem(w, h, K, doSlam);
				system->setVisualization(outputWrapper);
				fullResetRequested = false;
				runningIDX = 0;
			}
			key = cv::waitKey(10);
		}
	}

	if (inputType == InputType::imageFile)
	{
#ifdef RGBDTUM

		int runningIDX = 0;
		float fakeTimeStamp = 0;
		double hz = 0;
		std::string DataSetPath = dataPath;//"E:\\RGBD-dataSet\\rgbd_dataset_freiburg2_xyz\\";
		std::string rgbImgPath, depthImgPath;

		//wx- rgb图片
		cv::Mat rgbImage;
		cv::Mat image;
		//wx- 深度图和摄像机坐标真值
		cv::Mat depthImage;
		Eigen::Vector3f accelerometer;
		Eigen::Vector3f position;
		Eigen::Vector4f pose;
		char key = ' ';

		FileParser<Eigen::Vector3f, Eigen::Vector4f> parser(DataSetPath);
		analyzer.setTrajectoryFile("trajectory_lsd_xyz.txt");
		while (parser.getNextRecord(rgbImgPath, depthImgPath, accelerometer, position, pose))//Process while successfully read a frame//wx 按s键终止重建循环
		{
			analyzer.resetStartTime();
			analyzer.currentTimeStamp = parser.timeStamp;
			rgbImage = cv::imread(DataSetPath + rgbImgPath);

			//if (rgbImage.empty() || depthImage.empty())break;
			if (rgbImage.empty())break;
			cv::cvtColor(rgbImage, image, CV_BGR2GRAY);
			if (image.rows != h_inp || image.cols != w_inp)
			{
				std::cout << "image size error" << std::endl;
				continue;
			}
			assert(image.type() == CV_8U);
			undistorter->undistort(image, image);
			assert(image.type() == CV_8U);

			if (runningIDX == startFramID)
			{
				depthImage = cv::imread(DataSetPath + depthImgPath, CV_LOAD_IMAGE_ANYDEPTH);
				float factor = 5000;
				ushort* depthImageData = (ushort*)depthImage.data;
				float* depth = new float[w*h];
				double averageDepth = 0;
				unsigned int count_depth = 0;
				for (int i = 0; i < w*h; i++)
				{
					if (depthImageData[i] != 0)
					{
						depth[i] = (float)depthImageData[i] / factor;
						averageDepth += depth[i];
						count_depth++;
					}
					else
						depth[i] = -1;
				}
				averageDepth /= count_depth;
				moveSpeed = averageDepth / 1000;
				//system->randomInit(image.data, fakeTimeStamp, runningIDX, rgbImage.data, rgbImgPath);
				analyzer.setCameraBase(position, pose);
				system->gtDepthInit(image.data, depth, fakeTimeStamp, runningIDX, rgbImage.data,rgbImgPath);

				analyzer.setCurrentCamera(position, pose);
			}
			else
			{
				if (runningIDX < startFramID)
				{
					runningIDX++;
					continue;
				}
				depthImage = cv::imread(DataSetPath + depthImgPath, CV_LOAD_IMAGE_ANYDEPTH);
				float factor = 5000;
				ushort* depthImageData = (ushort*)depthImage.data;
				float* depth = new float[w*h];
				double averageDepth = 0;
				unsigned int count_depth = 0;
				for (int i = 0; i < w*h; i++)
				{
					if (depthImageData[i] != 0)
					{
						depth[i] = (float)depthImageData[i] / factor;
					}
					else
						depth[i] = -1;
				}
				//#ifdef RGBDSLAM
				//    system->trackRGBDFrame(depth, image.data, rgbImage.data, runningIDX, hz == 0, fakeTimeStamp);
				//#else
				system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp, rgbImage.data, depth,rgbImgPath);
				//#endif
			}
			analyzer.setCurrentGTCamera(position, pose);

			outputWrapper->publishTrajectoryIncrement(analyzer.position_current, "");
			outputWrapper->publishTrajectoryGTIncrement(analyzer.position_currentGT, "");
			if (system->trackingIsGood)
				analyzer++;
			runningIDX++;
			fakeTimeStamp += 0.03;
			//printf("//////////////////////////////\n ");
			//printf("//////next frame %d/////// \n ", runningIDX);
			//printf("//////////////////////////// \n ");

			if (fullResetRequested)
			{
				printf("FULL RESET!\n");
				delete system;
				system = new SlamSystem(w, h, K, doSlam);
				system->setVisualization(outputWrapper);
				fullResetRequested = false;
				runningIDX = 0;
			}
			//key = cv::waitKey(0);
		}
#endif
	}
	analyzer.logToFile("perform.txt");
	system->finalize();
	//system->saveKeyframeGraph();//edit-by-wx  2015-10-30 为了与手机端程序同步，增加了显式输出关键帧和点云的操作
	//system->savePointCloud();
	while (true)
	{
		Sleep(100);
	}

	delete system;
	delete undistorter;
	delete outputWrapper;

	return 0;
}





std::string makeImageName(int frameID)
{
	char buf[20];
	std::string frameIDstring(itoa(frameID, buf, 10));
	std::string imageName;
	for (int i = 0; i < 5 - frameIDstring.size(); i++)
	{
		imageName += "0";
	}
	imageName += frameIDstring + ".png";
	return imageName;
}



void parseCommand(int argc, char** argv)
{
	for (int i = 1; i < argc; i++)
	{
		if (std::string(argv[i]) == "-camera")
		{
			inputType = InputType::camera;
			std::cout << "input from camera" << std::endl;
		}
		else if (std::string(argv[i]) == "-video")
		{
			inputType = InputType::videoFile;
			std::cout << "input from video" << std::endl;
		}
		else if (std::string(argv[i]) == "-calibFile")
		{
			calibFile = std::string(argv[++i]);
			std::cout << "calib File: " << calibFile << std::endl;
		}
		else if (std::string(argv[i]) == "-onlyDisplay")
		{
			onlyDisplay = 1;
			std::cout << "only display: " << dataPath << std::endl;
		}
		else if (std::string(argv[i]) == "-dataPath")
		{
			dataPath = std::string(argv[++i]);
			std::cout << "data path: " << dataPath << std::endl;
		}
		else if (std::string(argv[i]) == "-keyFramePath")
		{
			keyFramePath = std::string(argv[++i]);
			std::cout << "key frame path: " << keyFramePath << std::endl;
		}
		else if (std::string(argv[i]) == "-keyFrameCount")
		{
			kefFrameCnt = onlyDisplay ? atoi(argv[++i]) : 0;
		}
		else if (std::string(argv[i]) == "-startFrame")
		{
			startFramID = atoi(argv[++i]);
		}
		else if (std::string(argv[i]) == "-disWeight")
		{
			disWeight = atof(argv[++i]);
		}
		else if (std::string(argv[i]) == "-ifUseEdge")
		{
			ifUseEdge = atoi(argv[++i]);
		}
		else if (std::string(argv[i]) == "-printTrackInfo")
		{
			printTrackingIterationInfo = atoi(argv[++i]);
		}
		else if (std::string(argv[i]) == "-edgeThr")
		{
			edgeThr = atoi(argv[++i]);
		}
		else if (std::string(argv[i]) == "-disType")
		{
			disType = atoi(argv[++i]);
			//CV_DIST_L1 = 1,CV_DIST_L2=2,CV_DIST_C = 3,CV_DIST_HUBER=7
		}
		else if (std::string(argv[i]) == "-startLayer")
		{
			startLayer = atoi(argv[++i]);
			//CV_DIST_L1 = 1,CV_DIST_L2=2,CV_DIST_C = 3,CV_DIST_HUBER=7
		}
		else if (argv[i][0] == '-')
		{
			std::cout << "invalid option " << argv[i] << std::endl;
			++i;
		}
	}

	std::cout << std::endl;
	if ((inputType == InputType::videoFile || inputType == InputType::imageFile) && dataPath == "")
	{
		std::cout << "error: no input " << std::endl;
		system("pause");
		exit(-1);
	}
	else if (calibFile == "")
	{
		std::cout << "error: need calib file: " << std::endl;
		system("pause");
		exit(-1);
	}
	else if (onlyDisplay == 1 && (keyFramePath == "" || kefFrameCnt == 0))
	{
		std::cout << "error: need key frame path and keyframe count " << std::endl;
		system("pause");
		exit(-1);
	}
	else if (keyFramePath == "")
	{
		keyFramePath = "keyFrame\\";
		std::cout << "warning use default keyFrame folder: " << keyFramePath << std::endl;
	}
	std::cout << std::endl;
}
