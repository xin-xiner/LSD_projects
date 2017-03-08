#pragma once
#include <iostream>
#include <time.h>
#include <fstream>
double quaternionToDegree(Eigen::Vector4f& q);

class PerformAnalyzer
{
public:
	float* depth_currentGT = 0;
	float* depth_current = 0;
	Eigen::Vector3f position_GT_base;
	Eigen::Vector4f pose_GT_base;
	Eigen::Vector3f position_currentGT;
	Eigen::Vector3f position_current;
	Eigen::Vector4f pose_currentGT;
	Eigen::Vector4f pose_current;
	Eigen::Matrix3f Rot_base;
	std::string currentTimeStamp;

	double degree_currentGT;
	double degree_current;
	double degree_GT_base;

	double time_currentSum = 0;
	double time_start = 0;
	long frameNum_currentSum = 0;
	double positionError_currentSum = 0;
	double poseError_currentSum = 0;
	double depthError_currentSum = 0;



public:

	PerformAnalyzer();

	void setTrajectoryFile(std::string trajectoryFileName);

	void resetStartTime();

	void setCameraBase(Eigen::Vector3f& position, Eigen::Vector4f& pose);

	void setCurrentGTCamera(Eigen::Vector3f& position, Eigen::Vector4f& pose);

	void setCurrentCamera(Eigen::Vector3f& position, Eigen::Vector4f& pose);

	PerformAnalyzer& operator++();

	double getAverageFrameProcessTime();

	double getAverageFPS();

	double getAveragePositionError();

	double getAveragePoseError();

	double getTimeSum_sec();

	long getFrameCount();

	void logToFile(std::string fileName, std::string filePath = "");

	std::ofstream trajectoryFile;
	bool makeTrajectoryFile = false;
};


extern PerformAnalyzer analyzer;