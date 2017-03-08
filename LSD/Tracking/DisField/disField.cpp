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
#include "disField.h"
#include <opencv2\imgproc\imgproc.hpp>
using namespace lsd_slam;

float disWeight = 1;
int ifUseEdge = 1;
int edgeThr = 80;
int disType = 0;
int startLayer = 1;
void computeDistanceGradientsFromDisField(const float* disField, int width, int height, Eigen::Vector2f* grad)
{
	const float* img_pt = disField + width;//wx-从第二行开始算起
	const float* img_pt_max = disField + width*(height - 1);
	Eigen::Vector2f* gradxyii_pt = grad + width;
	// in each iteration i need -1,0,p1,mw,pw
	float val_m1 = *(img_pt - 1);
	float val_00 = *img_pt;
	float val_p1;
	for (; img_pt < img_pt_max; img_pt++, gradxyii_pt++)
	{
		val_p1 = *(img_pt + 1);
		*((float*)gradxyii_pt) = 0.5f*(val_p1 - val_m1);//wx- x方向上梯度？
		*(((float*)gradxyii_pt) + 1) = 0.5f*(*(img_pt + width) - *(img_pt - width));//wx- y方向上梯度？
		val_m1 = val_00;//wx-向右移动一个点，如果是在最右边的点，结果好x方向梯度好像就不太对了啊
		val_00 = val_p1;
	}
}



void makeIdepthFlag(const float* maxGradients, unsigned char* idepthFlag, int width, int height)
{
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			int idx = y*width + x;
			if (maxGradients[x + y*width] > MIN_ABS_GRAD_CREATE)
			{
				idepthFlag[idx] = 1;
			}
			else
				idepthFlag[idx] = 0;
		}
	}
}
void computeDistanceFieldFromGradient(const float* maxGradiens, int width, int height, float* disField)
{
	unsigned char* idepthFlag = new unsigned char[width*height];
	makeIdepthFlag(maxGradiens, idepthFlag, width, height);
	computeDisFieldFromDepthFlag(idepthFlag, width, height, disField);
	delete[] idepthFlag;
}

void updateDisRecursion(int h, int w, int dis, float* disField, int height, int width)
{
	if (disField[h*width + w] <= dis)
		return;
	else if (dis < 0)
	{
		dis = 0;
		disField[h*width + w] = dis;
	}
	else
		disField[h*width + w] = dis;
#pragma omp parallel for
	for (int i = h - 1; i <= h + 1; i++)
		for (int j = w - 1; j <= w + 1; j++)
		{
			if (i < 0 || j < 0 || i >= height || j >= width)
				continue;
			updateDisRecursion(i, j, dis + 1, disField, height, width);
		}
}

int computeDisFieldFromDepthFlag(unsigned char* idepth, int w, int h, float* disField)
{
	const unsigned char* id = idepth;
	const unsigned int disInitial = (std::max)(w, h);
#pragma omp parallel for
	for (int i = 0; i < w; i++)
		for (int j = 0; j < h; j++)
		{
			disField[j*w + i] = disInitial;
		}

	int voidEdge = 3;
#pragma omp parallel for
	for (int i = 0 + voidEdge; i < w - voidEdge; i++)
	{
		int currentDis = disInitial;
		for (int j = 0 + voidEdge; j < h - voidEdge; j++)//从上到下遍历
		{
			if (id[j*w + i] != 0)
				currentDis = 0;
			if (disField[j*w + i]>currentDis)
				disField[j*w + i] = currentDis;
			currentDis++;
		}
		for (int j = h - 1 - voidEdge; j >= 0 + voidEdge; j--)//从下到上遍历
		{
			if (id[j*w + i] != 0)
				currentDis = 0;
			if (disField[j*w + i]>currentDis)
				disField[j*w + i] = currentDis;
			currentDis++;
		}
	}
#pragma omp parallel for
	for (int i = 0 + voidEdge; i < h - voidEdge; i++)
	{
		int currentDis = disInitial;
		for (int j = 0 + voidEdge; j < w - voidEdge; j++)//从左到右遍历
		{
			if (id[i*w + j] != 0)
				currentDis = 0;
			if (disField[i*w + j]>currentDis)
				disField[i*w + j] = currentDis;
			currentDis++;
		}
		for (int j = w - 1 - voidEdge; j >= 0 + voidEdge; j--)//从右到左遍历
		{
			if (id[i*w + j] != 0)
				currentDis = 0;
			if (disField[i*w + j]>currentDis)
				disField[i*w + j] = currentDis;
			currentDis++;
		}
	}
	int edgeCount = 0;
	for (int i = 0 + voidEdge; i < h - voidEdge; i++)
		for (int j = 0 + voidEdge; j < w - voidEdge; j++)
		{
			if (idepth[i*w + j] != 0)
			{
				updateDisRecursion(i, j, -1, disField, h, w);
				edgeCount++;
			}
		}
	return edgeCount;
}



void makeEdgeFlag(const float* image, int width, int height, unsigned char* edgeFlag)
{
	cv::Mat imageCV(height, width, CV_8U), edge;
	uchar* imageData = imageCV.data;
	for (int i = 0; i < width*height; i++)
	{
		imageData[i] = (uchar)image[i];
	}
	//imshow("edgeImageCV", imageCV);
	//cv::waitKey(10);
	//std::cout << "edge begin" << std::endl;
	Canny(imageCV, edge, 150, 255);
	//cv::bitwise_not(edge, edge);
	//imshow("edge", edge);
	//std::cout << "edge end" << std::endl;
	//cv::waitKey(10);
	imageData = edge.data;
	for (int i = 0; i < width*height; i++)
	{
		edgeFlag[i] = imageData[i];
	}
}

int computeDisFieldFromImage(const float* image, int width, int height, unsigned char* edgeFlag, float* disField, Eigen::Vector2f* grad, std::string fileName)
{
	cv::Mat imageCV(height, width, CV_8U), edge;
	cv::Mat disImage;// (height, width, CV_8U);
	uchar* imageData = imageCV.data;
#pragma omp parallel for
	for (int i = 0; i < width*height; i++)
	{
		imageData[i] = (uchar)image[i];
	}
	//imshow("edgeImageCV", imageCV);
	//cv::waitKey(10);
	//std::cout << "edge begin" << std::endl;
	int lowThr = 100;//edgeThr;
	int highThr = 30;//lowThr * 2>255 ? 255 : lowThr * 2;
	//cv::blur(imageCV, imageCV, cv::Size(3, 3));
	Canny(imageCV, edge, lowThr, highThr, 3, true);
	//std::string path_edge = "D:\\dataSets\\6\\6\\";
	//edge = cv::imread(path_edge + fileName);
	//cv::resize(edge, edge, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
	//cv::cvtColor(edge, edge, CV_BGR2GRAY);
	cv::threshold(edge, edge, 30, 255, cv::THRESH_BINARY);
	//cv::resize(edge, edge, cv::Size(width, height), 0,0, cv::INTER_NEAREST);
	//cv::imshow("edges ", edge);
	/*cv::imshow("edge_test", edge);
	cv::waitKey(10);*/
	cv::Mat regu_edge = cv::Mat::ones(edge.rows, edge.cols, CV_8U) * 255;
	uchar* edgeData = edge.data;

	/*MatVis::Vis()->VisUcharArray(edgeData, width, height);
	MatVis::Vis()->logToFile("edge.txt");*/
	uchar* regu_edgeData = regu_edge.data;
	const int regu_image_edge_size = 5;
//#pragma omp parallel for
//	for (int h = 0 + regu_image_edge_size; h < height - regu_image_edge_size; h++)
//		for (int w = 0 + regu_image_edge_size; w < width - regu_image_edge_size; w++)
//		{
//			regu_edgeData[h*width + w] = ~edgeData[h*width + w];
//		}
	//cv::imshow("regu_edge", regu_edge);



	cv::bitwise_not(edge, regu_edge);

	//CV_DIST_L1 = 1,CV_DIST_L2=2,CV_DIST_C = 3,CV_DIST_HUBER=7
	cv::distanceTransform(regu_edge, disImage, disType, CV_DIST_MASK_PRECISE);

	disImage *= disWeight;
	//std::cout << type2str(disImage.type()) << std::endl;
	//MatVis::Vis()->VisUcharArray((uchar*)disImage.data, width, height);
	//MatVis::Vis()->logToFile("disField.txt");
	//cv::Mat gradX, gradY;
	//cv::Sobel(disImage, gradX, disImage.type(), 1, 0, 3);
	//cv::Sobel(disImage, gradY, disImage.type(), 0, 1, 3);
	//float* gradXData = (float*)gradX.data;
	//float* gradYData = (float*)gradY.data;

	regu_edgeData = regu_edge.data;
	float* disData = (float*)disImage.data;
	int edgeCount = 0;
#pragma omp parallel for
	for (int i = 0; i < width*height; i++)
	{
		edgeFlag[i] = regu_edgeData[i];
		if (edgeFlag[i] == 0)
			edgeCount++;
		disField[i] = disData[i];
		//grad[i].x() = gradXData[i];
		//grad[i].y() = gradYData[i];
	}
	
	computeDistanceGradientsFromDisField(disField, width, height, grad);

	return edgeCount;
}








void plotEdgeReprojection(lsd_slam::TrackingReference* reference,
	lsd_slam::Frame* frame, const Sophus::SE3f& referenceToFrame, int lvl, int iteration, int plotID)
{
	int w = frame->width(lvl);
	int h = frame->height(lvl);
	cv::Vec3b* depthCImageData;
	cv::Mat idepthImage;
	cv::Mat disFieldImage;
	cv::Vec3b* disCImageData;

	float* imageData = frame->image(lvl);

	const uchar* edgeData = reference->keyframe->edgeFlag(lvl);
	const uchar* currentEdge = frame->edgeFlag(lvl);
	const float* disData = frame->disField(lvl);
	const float* gradData = frame->maxGradients(lvl);
	idepthImage = cv::Mat(h, w, CV_8U);
	disFieldImage = cv::Mat(h, w, CV_8U);
	cv::Mat gradImage = cv::Mat(h, w, CV_8U);

	uchar* gradImageData = gradImage.data;
	uchar* depthImageData = idepthImage.data;
	uchar* disFieldImageData = disFieldImage.data;
	float maxDis = *(std::max_element)(disData, disData + w*h);
	float minDis = *(std::min_element)(disData, disData + w*h);
	float maxGrad = *(std::max_element)(gradData,gradData+w*h);
	float minGrad = *(std::min_element)(gradData, gradData + w*h);
	for (int i = 0; i < w*h; i++)
	{
		depthImageData[i] = (uchar)imageData[i];
		if (currentEdge[i]>0)
			depthImageData[i] = 0;
		if (disData[i] > 0)
		{
			disFieldImageData[i] = ((float)(disData[i] - minDis)) / ((float)(maxDis - minDis)) * 255;
			if (disFieldImageData[i] > 255)
				disFieldImageData[i] = 255;
		}
		else if (disData[i] == 0)
		{
			disFieldImageData[i] = disData[i];
		}

		gradImageData[i] = ((gradData[i]-minGrad) / (maxGrad-minGrad)) * 255;
	}

	cv::cvtColor(idepthImage, idepthImage, cv::COLOR_GRAY2BGR);
	cv::cvtColor(disFieldImage, disFieldImage, cv::COLOR_GRAY2BGR);
	depthCImageData = (cv::Vec3b*)idepthImage.data;
	disCImageData = (cv::Vec3b*)disFieldImage.data;

	Eigen::Matrix3f KLvl = frame->K(lvl);
	float fx_l = KLvl(0, 0);
	float fy_l = KLvl(1, 1);
	float cx_l = KLvl(0, 2);
	float cy_l = KLvl(1, 2);
	Eigen::Matrix3f rotMat = referenceToFrame.rotationMatrix();
	Eigen::Vector3f transVec = referenceToFrame.translation();

	Eigen::Vector3f* refPoint = reference->posData[lvl];
	int refNum = reference->numData[lvl];

	const Eigen::Vector3f* refPoint_max = refPoint + refNum;

	for (; refPoint < refPoint_max; refPoint++)
	{
		Eigen::Vector3f Wxp = rotMat * (*refPoint) + transVec;
		float u_new = (Wxp[0] / Wxp[2])*fx_l + cx_l;
		float v_new = (Wxp[1] / Wxp[2])*fy_l + cy_l;

		Eigen::Vector3f WxpRef = (*refPoint);
		float u = (WxpRef[0] / WxpRef[2])*fx_l + cx_l;
		float v = (WxpRef[1] / WxpRef[2])*fy_l + cy_l;
		if (!(u > 1 && v > 1 && u < w - 2 && v < h - 2))
		{
			continue;
		}
		if (!(u_new > 1 && v_new > 1 && u_new < w - 2 && v_new < h - 2))
		{
			continue;
		}
		int iv_new = v_new;
		int iu_new = u_new;
		if (iv_new*w + iu_new>w*h || iv_new*w + iu_new < 0)
			continue;
		int vref_new = v;
		int uref_new = u;
		if (edgeData[vref_new*w + uref_new] != 0)//|| (disData[iv_new*w + iu_new] >= lastEdgeAverageError*EDGE_THR_SCALE))
		{
			continue;
		}
		depthCImageData[iv_new*w + iu_new] = cv::Vec3b(0, 0, 255);
		disCImageData[iv_new*w + iu_new] = cv::Vec3b(0, 0, 255);
	}
	cv::namedWindow("current frame", 0);
	cv::imshow("current frame", idepthImage);
	std::stringstream ssr;

	ssr << "..\\debug_images\\currentframe\\" << "lvl_" << lvl << "_it_"; if (iteration < 0)ssr << "000"; ssr << iteration << "_" << plotID << ".png";
	cv::imwrite(ssr.str(), idepthImage);

	cv::namedWindow("disField", 0);
	cv::imshow("disField", disFieldImage);

	ssr.clear(); ssr.str("");
	ssr << "..\\debug_images\\disField\\" << "lvl_" << lvl << "_it_"; if (iteration < 0)ssr << "000"; ssr << iteration << "_" << plotID << ".png";
	cv::imwrite(ssr.str(), disFieldImage);
	
	cv::namedWindow("gradImage", 0);
	cv::imshow("gradImage", gradImage);

	ssr.clear(); ssr.str("");
	ssr << "..\\debug_images\\gradImage\\" << "lvl_" << lvl << "_it_"; if (iteration < 0)ssr << "000"; ssr << iteration << "_" << plotID << ".png";
	cv::imwrite(ssr.str(), gradImage);
	cv::waitKey(10);
}





