#pragma once
#include <preprocessHeaders.h>
#include <../LSD/Tracking/SE3Tracker.h>
#include <../LSD/Tracking/TrackingReference.h>
#include "../LSD/DepthEstimation/DepthMap.h"
#include "../LSD/util/settings.h"
#include "../LSD/DepthEstimation/DepthMapPixelHypothesis.h"
#include "../LSD/DataStructures/Frame.h"
#include "../LSD/util/globalFuncs.h"
#include "../LSD/IOWrapper/ImageDisplay.h"
#include "../LSD/GlobalMapping/KeyFrameGraph.h"
using namespace lsd_slam;

extern float disWeight;
extern int ifUseEdge;
extern int edgeThr;
extern int disType;
extern int startLayer;
#define edgeRatio 10
//(ifUseEdge == 1 ? 5 : 1)

void computeDistanceGradientsFromDisField(const float* disField, int width, int height, Eigen::Vector2f* grad);
void computeDistanceFieldFromGradient(const float* maxGradiens, int width, int height, float* disField);
int computeDisFieldFromDepthFlag(unsigned char* idepth, int w, int h, float* disField);
void makeEdgeFlag(const float* image, int width, int height, unsigned char* edgeFlag);
int computeDisFieldFromImage(const float* image, int width, int height, unsigned char* edgeFlag, float* disField, Eigen::Vector2f* grad, std::string fileName);




void plotEdgeReprojection(lsd_slam::TrackingReference* reference,
	lsd_slam::Frame* frame, const Sophus::SE3f& referenceToFrame, int lvl = 1, int iteration = -1, int plotID = 0);
