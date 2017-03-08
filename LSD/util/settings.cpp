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
#include "preprocessHeaders.h"
#include "settings.h"




namespace lsd_slam
{
	RunningStats runningStats;


	bool autoRun = true;
	bool autoRunWithinFrame = true;

	int debugDisplay = 1;

	bool onSceenInfoDisplay = true;
	bool displayDepthMap = true;//edit-by-wx-debug  2015-10-30 关闭对深度图的显示,否则手机上会segfault
	bool dumpMap = false;
	bool doFullReConstraintTrack = false;

	// dyn config
	bool printPropagationStatistics = true;
	bool printFillHolesStatistics = true;
	bool printObserveStatistics = true;
	bool printObservePurgeStatistics = true;
	bool printRegularizeStatistics = true;
	bool printLineStereoStatistics = true;
	bool printLineStereoFails = true;

	bool printTrackingIterationInfo = false;

	bool printFrameBuildDebugInfo = true;
	bool printMemoryDebugInfo = true;

	bool printKeyframeSelectionInfo = true;
	bool printConstraintSearchInfo = true;
	bool printOptimizationInfo = true;
	bool printRelocalizationInfo = true;

	bool printThreadingInfo = true;
	bool printMappingTiming = true;
	bool printOverallTiming = true;

	bool plotTrackingIterationInfo = false;
	bool plotSim3TrackingIterationInfo = false;
	bool plotStereoImages = true;//wx-显示stereo时做对应时的极线图和当前关键帧做stereo匹配时的状态。
	//极线图(Stereo Reference Frame)中的线代表了极线，长度代表了搜索的范围，颜色从绿到蓝代表了找到的匹配的误差从小到大。极线长度由深度可能的范围给定，在createStero时极线长度由一个固定深度最大值计算，在updateStereo时由之前的深度和方差确定。颜色道理上应当是从绿色变到蓝色，但实际情况一般只有蓝色和绿色。
	//关键帧的stereo匹配状态图(Stereo Key Frame)，代表了当前关键帧在做立体匹配时的状态。其中：蓝色代表跟踪的结果不好的点；白色代表还未有深度的点；绿色代表更新深度Hyposis时跳过的点；黄色代表更新了深度的点；红色代表异常值；紫色代表重建结果不合理的点；黑色代表无重建结果；石青(蓝绿色)重建结果不相容的点；
	bool plotTracking = true;//wx-显示对关键帧的tracking状态，利用匹配之后的灰度计算了匹配值，灰色的代表了跟踪较好的点，红色代表跟踪状态不好的点


	float freeDebugParam1 = 1;
	float freeDebugParam2 = 1;
	float freeDebugParam3 = 1;
	float freeDebugParam4 = 1;
	float freeDebugParam5 = 1;

	float KFDistWeight = 4;
	float KFUsageWeight = 3;//wx-deubg 3-30 这里之前是3

	float minUseGrad = 5;
	float cameraPixelNoise2 = 4 * 4;
	float depthSmoothingFactor = 1;

	bool allowNegativeIdepths = true;
	bool useMotionModel = false;
	bool useSubpixelStereo = true;
	bool multiThreading = true;
	bool useAffineLightningEstimation = true;



	bool useFabMap = false;
	bool doSlam = false;
	bool doKFReActivation = false;
	bool doMapping = true;

	int maxLoopClosureCandidates = 10;
	int maxOptimizationIterations = 100;
	int propagateKeyFrameDepthCount = 0;
	float loopclosureStrictness = 1.5;
	float relocalizationTH = 0.7;


	bool saveKeyframes = false;
	bool saveAllTracked = false;
	bool saveLoopClosureImages = false;
	bool saveAllTrackingStages = false;
	bool saveAllTrackingStagesInternal = false;

	bool continuousPCOutput = true;


	bool fullResetRequested = false;
	bool manualTrackingLossIndicated = false;


	std::string packagePath = "";


	void handleKey(char k)
	{
		char kkk = k;
		switch (kkk)
		{
		case 'a': case 'A':
			//		autoRun = !autoRun;		// disabled... only use for debugging & if you really, really know what you are doing
			break;
		case 's': case 'S':
			//		autoRunWithinFrame = !autoRunWithinFrame; 	// disabled... only use for debugging & if you really, really know what you are doing
			break;
		case 'd': case 'D':
			debugDisplay = (debugDisplay + 1) % 6;
			printf("debugDisplay is now: %d\n", debugDisplay);
			break;
		case 'e': case 'E':
			debugDisplay = (debugDisplay - 1 + 6) % 6;
			printf("debugDisplay is now: %d\n", debugDisplay);
			break;
		case 'o': case 'O':
			onSceenInfoDisplay = !onSceenInfoDisplay;
			break;
		case 'r': case 'R':
			printf("requested full reset!\n");
			fullResetRequested = true;
			break;
		case 'm': case 'M':
			printf("Dumping Map!\n");
			dumpMap = true;
			break;
		case 'p': case 'P':
			printf("Tracking all Map-Frames again!\n");
			doFullReConstraintTrack = true;
			break;
		case 'l': case 'L':
			printf("Manual Tracking Loss Indicated!\n");
			manualTrackingLossIndicated = true;
			break;
		}

	}


	DenseDepthTrackerSettings::DenseDepthTrackerSettings()
	{
		// Set default settings
		if (PYRAMID_LEVELS > 6)
			printf("WARNING: Sim3Tracker(): default settings are intended for a maximum of 6 levels!");

		lambdaSuccessFac = 0.5f;
		lambdaFailFac = 2.0f;

		const float stepSizeMinc[6] = { 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8 };
		const int maxIterations[6] = { 5, 100, 50, 100, 100, 100 };


		for (int level = 0; level < PYRAMID_LEVELS; ++level)
		{
			lambdaInitial[level] = 0;
			stepSizeMin[level] = stepSizeMinc[level];
			convergenceEps[level] = 0.999f;
			maxItsPerLvl[level] = maxIterations[level];
		}

		lambdaInitialTestTrack = 0;
		stepSizeMinTestTrack = 1e-3;
		convergenceEpsTestTrack = 0.98;
		maxItsTestTrack = 5;

		var_weight = 1.0;
		huber_d = 3;
	}




}
