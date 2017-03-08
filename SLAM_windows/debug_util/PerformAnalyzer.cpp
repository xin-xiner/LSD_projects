#include "preprocessHeaders.h"
#include "PerformAnalyzer.h"



PerformAnalyzer analyzer;


double quaternionToDegree(Eigen::Vector4f& q)
{
	return ((2 * acos(q(3))) / 3.14) * 180;
}


PerformAnalyzer::PerformAnalyzer()
{
	time_start = clock();
}
void PerformAnalyzer::setTrajectoryFile(std::string trajectoryFileName)
{
	if (trajectoryFileName != "")
	{
		trajectoryFile.open(trajectoryFileName);
		if (!trajectoryFile.is_open())
		{
			std::cout << "trajectoryFile open error" << std::endl;
			system("pause");
		}
		makeTrajectoryFile = true;
	}
}

void PerformAnalyzer::resetStartTime()
{
	time_start = clock();
}


void PerformAnalyzer::setCameraBase(Eigen::Vector3f& position, Eigen::Vector4f& pose)
{
	position_GT_base = position;
	pose_GT_base = pose;
	degree_GT_base = quaternionToDegree(pose);
}


void PerformAnalyzer::setCurrentGTCamera(Eigen::Vector3f& position, Eigen::Vector4f& pose)
{
	position_currentGT = position;
	pose_currentGT = pose;
	degree_currentGT = abs(quaternionToDegree(pose));
	
}


void PerformAnalyzer::setCurrentCamera(Eigen::Vector3f& position, Eigen::Vector4f& pose)
{
	position_current = position;
	pose_currentGT = pose;
	degree_current = abs(quaternionToDegree(pose));
	if (makeTrajectoryFile)
		trajectoryFile << currentTimeStamp << " " << position_current.transpose() << " " << pose_currentGT.transpose() << std::endl;
}


PerformAnalyzer& PerformAnalyzer::operator++()
{
	time_currentSum += (clock() - time_start);
	frameNum_currentSum++;
	positionError_currentSum += abs((position_currentGT - position_current).norm());
	poseError_currentSum += abs(degree_currentGT - degree_current);
	return *this;
}



double PerformAnalyzer::getAverageFrameProcessTime()
{
	return getTimeSum_sec() / frameNum_currentSum;
}

double PerformAnalyzer::getAverageFPS()
{
	return frameNum_currentSum / getTimeSum_sec();
}


double PerformAnalyzer::getAveragePositionError()
{
	return positionError_currentSum / frameNum_currentSum;
}


double PerformAnalyzer::getAveragePoseError()
{
	return poseError_currentSum / frameNum_currentSum;
}

double PerformAnalyzer::getTimeSum_sec()
{
	return time_currentSum / 1000;
}

long PerformAnalyzer::getFrameCount()
{
	return frameNum_currentSum;
}

void PerformAnalyzer::logToFile(std::string fileName, std::string filePath)
{
	std::ofstream log(filePath+fileName);
	log  << fileName << std::endl;
	log << "FrameCount " << getFrameCount() << std::endl;
	log << "getTimeSum(sec) " << getTimeSum_sec() << std::endl;
	log << "-------------------------------------------------------------" << std::endl;
	log << "AverageFPS   " << getAverageFPS() << std::endl;
	log << "AveragePositionError  " << getAveragePositionError() << std::endl;
	log << "AveragePoseError   " << getAveragePoseError() << std::endl;

	log.flush();
	log.close();

	std::cout  << fileName << std::endl;
	std::cout << "FrameCount " << getFrameCount() << std::endl;
	std::cout << "getTimeSum(sec) " << getTimeSum_sec() << std::endl;
	std::cout << "-------------------------------------------------------------" << std::endl;
	std::cout << "AverageFPS   " << getAverageFPS() << std::endl;
	std::cout << "AveragePositionError  " << getAveragePositionError() << std::endl;
	std::cout << "AveragePoseError   " << getAveragePoseError() << std::endl;
}