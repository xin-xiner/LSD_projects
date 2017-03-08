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
#include "DepthMap.h"
#include "../util/settings.h"
#include "DepthMapPixelHypothesis.h"
#include "../DataStructures/Frame.h"
#include "../util/globalFuncs.h"
#include "../IOWrapper/ImageDisplay.h"
#include "../GlobalMapping/KeyFrameGraph.h"
#include "../../SLAM_windows/RGBDSLAM.h"
namespace lsd_slam
{

DepthMap::DepthMap(int w, int h, const Eigen::Matrix3f& K)
{
 width = w;
 height = h;
 activeKeyFrame = 0;
 activeKeyFrameIsReactivated = false;
 otherDepthMap = new DepthMapPixelHypothesis[width*height];
 currentDepthMap = new DepthMapPixelHypothesis[width*height];
 validityIntegralBuffer = (int*)Eigen::internal::aligned_malloc(width*height*sizeof(int));


 debugImageHypothesisHandling = cv::Mat(h,w, CV_8UC3);
 debugImageHypothesisPropagation = cv::Mat(h,w, CV_8UC3);
 debugImageStereoLines = cv::Mat(h,w, CV_8UC3);
 debugImageDepth = cv::Mat(h,w, CV_8UC3);

 this->K = K;
 fx = K(0,0);
 fy = K(1,1);
 cx = K(0,2);
 cy = K(1,2);
 KInv = K.inverse();
 fxi = KInv(0,0);
 fyi = KInv(1,1);
 cxi = KInv(0,2);
 cyi = KInv(1,2);
 reset();
#ifdef TIME_CAL
 msUpdate =  msCreate =  msFinalize = 0;
 msObserve =  msRegularize =  msPropagate =  msFillHoles =  msSetDepth = 0;
 gettimeofday(&lastHzUpdate, NULL);
 nUpdate = nCreate = nFinalize = 0;
 nObserve = nRegularize = nPropagate = nFillHoles = nSetDepth = 0;
 nAvgUpdate = nAvgCreate = nAvgFinalize = 0;
 nAvgObserve = nAvgRegularize = nAvgPropagate = nAvgFillHoles = nAvgSetDepth = 0;
#endif
}
DepthMap::~DepthMap()
{
 if(activeKeyFrame != 0)
  activeKeyFramelock.unlock();
 debugImageHypothesisHandling.release();
 debugImageHypothesisPropagation.release();
 debugImageStereoLines.release();
 debugImageDepth.release();
 delete[] otherDepthMap;
 delete[] currentDepthMap;
 Eigen::internal::aligned_free((void*)validityIntegralBuffer);
}

void DepthMap::reset()
{
 for(DepthMapPixelHypothesis* pt = otherDepthMap+width*height-1; pt >= otherDepthMap; pt--)
  pt->isValid = false;
 for(DepthMapPixelHypothesis* pt = currentDepthMap+width*height-1; pt >= currentDepthMap; pt--)
  pt->isValid = false;
}

void DepthMap::observeDepthRow(int yMin, int yMax, RunningStats* stats)
{
 const float* keyFrameMaxGradBuf = activeKeyFrame->maxGradients(0);
 int successes = 0;
 
 for(int y=yMin;y<yMax; y++)
  for(int x=3;x<width-3;x++)
  {
   int idx = x+y*width;
   DepthMapPixelHypothesis* target = currentDepthMap+idx;
   bool hasHypothesis = target->isValid;
   // ======== 1. check absolute grad =========
   const unsigned char* edgeFlag = activeKeyFrame->edgeFlag(0);
   //std::cout << (int)edgeFlag[idx] << std::endl;
   if (edgeFlag[idx]> 50)
   {
	   //target->isValid = false;
	   continue;
   }


   if(hasHypothesis && keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_DECREASE)//wx-temp 仅保留当前关键帧中梯度较大位置的深度值，舍弃之前关键帧计算出的投影到当前关键帧时梯度不再足够大的深度，感觉这里不太合理
   {
    //target->isValid = false;          //wx-debug 这里可能会删除之前正确的重建结果，但一般不会发生。导致这个事件发生的可能是track跟踪失败或者遮挡等问题
    continue;
   }
   if(keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_CREATE || target->blacklisted < MIN_BLACKLIST)//wx- 梯度如果不足够大则不重建该点，如果改点已经被表示为不好重建的点，则放弃重建
    continue;

   bool success;
   if(!hasHypothesis)
    //guo-不是深度点但是满足一定条件创建一个深度点
    success = observeDepthCreate(x, y, idx, stats);//wx- 如果是之前没有重建过的一点(也可能是被删掉的)，则利用当前序列中最老的那一帧作为参考帧(即离当前关键帧的时间距离最近的一帧)进行重建
   else
    success = observeDepthUpdate(x, y, idx, keyFrameMaxGradBuf, stats);//wx- 如果已经有深度信息，则需要更新
   if(success)
    successes++;
  }

}
void DepthMap::observeDepth()
{
#ifdef RGBDSLAM
 threadReducer.reduce(boost::bind(&DepthMap::observeDepthRowRGBD, this, _1, _2, _3), 3, height - 3, 480);//wx-手动并行化的计算对每行计算深度，同时四个进程在计算，每个进程计算10行的深度，计算完进入下个10行
#else
 threadReducer.reduce(boost::bind(&DepthMap::observeDepthRow, this, _1, _2, _3), 3, height - 3, 480);//wx-手动并行化的计算对每行计算深度，同时四个进程在计算，每个进程计算10行的深度，计算完进入下个10行
#endif
 if(enablePrintDebugInfo && printObserveStatistics)
 {
  printf("OBSERVE (%d): %d / %d created; %d / %d updated; %d skipped; %d init-blacklisted\n",
    activeKeyFrame->id(),
    runningStats.num_observe_created,
    runningStats.num_observe_create_attempted,
    runningStats.num_observe_updated,
    runningStats.num_observe_update_attempted,
    runningStats.num_observe_skip_alreadyGood,
    runningStats.num_observe_blacklisted
  );
 }

 if(enablePrintDebugInfo && printObservePurgeStatistics)
 {
  printf("OBS-PRG (%d): Good: %d; inconsistent: %d; notfound: %d; oob: %d; failed: %d; addSkip: %d;\n",
    activeKeyFrame->id(),
    runningStats.num_observe_good,
    runningStats.num_observe_inconsistent,
    runningStats.num_observe_notfound,
    runningStats.num_observe_skip_oob,
    runningStats.num_observe_skip_fail,
    runningStats.num_observe_addSkip
  );
 }
}


bool DepthMap::makeAndCheckEPL(const int x, const int y, const Frame* const ref, float* pepx, float* pepy, RunningStats* const stats)
{
 int idx = x+y*width;
 // ======= make epl ========
 // calculate the plane spanned by the two camera centers and the point (x,y,1)
 // intersect it with the keyframe's image plane (at depth=1)
 float epx = - fx * ref->thisToOther_t[0] + ref->thisToOther_t[2]*(x - cx);//wx-temp 设参考图摄像机中心在深度图上的投影是点OC = (fx * ref->thisToOther_t[0]+cx，fy * ref->thisToOther_t[1]+cy，ref->thisToOther_t[2])， 感觉上这里的epx，epy应该是在深度图上从OC到(x,y)的向量，但由于OC没有齐次项不为1，所以当前的结果其实是  ref->thisToOther_t[2] * ((x,y) - OC)
 float epy = - fy * ref->thisToOther_t[1] + ref->thisToOther_t[2]*(y - cy);//wx 由于返回的pepx和pepy是单位化后的极线长度，所以齐次项的比例不用在意
 //guo-因为相机移位导致的位移
 if(isnan(epx+epy))
  return false;

 // ======== check epl length =========
 float eplLengthSquared = epx*epx+epy*epy;
 if(eplLengthSquared < MIN_EPL_LENGTH_SQUARED)
 {
  if(enablePrintDebugInfo) stats->num_observe_skipped_small_epl++;
  return false;
 }

 // ===== check epl-grad magnitude ======
 float gx = activeKeyFrameImageData[idx+1] - activeKeyFrameImageData[idx-1];
 float gy = activeKeyFrameImageData[idx+width] - activeKeyFrameImageData[idx-width];
 float eplGradSquared = gx * epx + gy * epy;
 eplGradSquared = eplGradSquared*eplGradSquared / eplLengthSquared; // square and norm with epl-length
 if(eplGradSquared < MIN_EPL_GRAD_SQUARED)
 {
  if(enablePrintDebugInfo) stats->num_observe_skipped_small_epl_grad++;
  return false;
 }

 // ===== check epl-grad angle ======
 if(eplGradSquared / (gx*gx+gy*gy) < MIN_EPL_ANGLE_SQUARED)
 {
  if(enablePrintDebugInfo) stats->num_observe_skipped_small_epl_angle++;
  return false;
 }

 // ===== DONE - return "normalized" epl =====
 float fac = GRADIENT_SAMPLE_DIST / sqrt(eplLengthSquared);
 *pepx = epx * fac;
 *pepy = epy * fac;
 return true;
}

bool DepthMap::observeDepthCreate(const int &x, const int &y, const int &idx, RunningStats* const &stats)
{
 DepthMapPixelHypothesis* target = currentDepthMap+idx;
 //guo-这里有age更新?
 Frame* refFrame = activeKeyFrameIsReactivated ? newest_referenceFrame : oldest_referenceFrame;//wx- activeKeyFrameIsReactivated代表的是是否出现LoopClosure或retrack，如果出现则当前关键帧被替换为环上的之前一帧。所以这时用最新的帧的话更能保证loop帧和当前参考帧的相似程度保证重建质量？而如果是相邻的话一定的视角差会是得重建结果更好？
 if(refFrame->getTrackingParent() == activeKeyFrame)
 {
  bool* wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();
  if(wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])
  {
   if(plotStereoImages)
    debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,0); // BLUE for SKIPPED NOT GOOD TRACKED
   return false;
  }
 }
 float epx, epy;
 //guo-epl是什么?//epipolar line
 bool isGood = makeAndCheckEPL(x, y, refFrame, &epx, &epy, stats);//wx- 计算在深度图上的极线
 if(!isGood) return false;
 if(enablePrintDebugInfo) stats->num_observe_create_attempted++;
 float new_u = x;
 float new_v = y;
 float result_idepth, result_var, result_eplLength;
 float error = doLineStereo(
   new_u,new_v,epx,epy,
   0.0f, 1.0f, 1.0f/MIN_DEPTH,
   refFrame, refFrame->image(0),
   result_idepth, result_var, result_eplLength, stats);
 if(error == -3 || error == -2)
 {
  target->blacklisted--;
  if(enablePrintDebugInfo) stats->num_observe_blacklisted++;
 }
 if(error < 0 || result_var > MAX_VAR)
  return false;
 
 result_idepth = UNZERO(result_idepth);
 // add hypothesis
 *target = DepthMapPixelHypothesis(
   result_idepth,
   result_var,
   VALIDITY_COUNTER_INITIAL_OBSERVE);
 if(plotStereoImages)
  debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,255); // white for GOT CREATED
 if(enablePrintDebugInfo) stats->num_observe_created++;
 
 return true;
}


bool DepthMap::observeDepthUpdate(const int &x, const int &y, const int &idx, const float* keyFrameMaxGradBuf, RunningStats* const &stats)
{
 DepthMapPixelHypothesis* target = currentDepthMap+idx;
 Frame* refFrame;

 if(!activeKeyFrameIsReactivated)//如果不是loopClosure或者reTrack的情况
 {
  if((int)target->nextStereoFrameMinID - referenceFrameByID_offset >= (int)referenceFrameByID.size())//wx- nextStereoFrameMinID是当该点被成功更新一次之后，按照那次计算时的极线长度计算出来的一个不再更新的id范围，即直到nextStereoFrameMinID。这里其实是利用offset量判断这个nextStereoFrameMinID是否已经进入队列，可以开始这一次更新了
  {
   if(plotStereoImages)
    debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,255,0); // GREEN FOR skip//wx- 立体匹配调试图中的绿色都来自于这里，即之前成功更新了一次，等待nextStereoFrameMinID
   if(enablePrintDebugInfo) stats->num_observe_skip_alreadyGood++;
   return false;
  }
                   //wx- nextStereoFrameMinID初始为0，在成功匹配后被设为未来才会遇到的一个帧ID，在与某帧匹配时如果出现不适合立体匹配的情况发生，则会被重新置为0
  if((int)target->nextStereoFrameMinID - referenceFrameByID_offset < 0)//这wx- 种情况只有在nextStereoFrameMinID是0的时候才会发生，即还没有更新过或之前与更新的值匹配时遇到了不适合立体匹配的情况
   refFrame = oldest_referenceFrame;
  else
   refFrame = referenceFrameByID[(int)target->nextStereoFrameMinID - referenceFrameByID_offset];//wx- 参考帧被设为了上次更新成功时预订的nextStereoFrameMinID那一帧，如果这次更新失败，下次会使用当前序列的最老帧进行匹配，这是不是就是论文中所谓的age增加？我竟无言以对
 }
 else
  refFrame = newest_referenceFrame;//如果是loopClosure或者retrack则直接使用最新的帧进行匹配

 if(refFrame->getTrackingParent() == activeKeyFrame)
 {
  bool* wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();//wx- 判断是否是一个跟踪状态合理的点，如果不是合理的点，说明当前的两个摄像机模型对该点是不适合重建的，故放弃重建
  if(wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])//wx- 左移右移什么的只是在除以2吧
  {
   if(plotStereoImages)
    debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,0); // BLUE for SKIPPED NOT GOOD TRACKED
   //target->isValid = false;//wx-debug，这里是我加上去的
   return false;//wx-debug 这里原本是有的
  }
 }
 
 float epx, epy;
 bool isGood = makeAndCheckEPL(x, y, refFrame, &epx, &epy, stats);
 if(!isGood) return false;
 // which exact point to track, and where from.
 float sv = sqrt(target->idepth_var_smoothed);
 float min_idepth = target->idepth_smoothed - sv*STEREO_EPL_VAR_FAC;//wx- 在已经有深度的先验的情况下，将最大可能的深度设为当前深度加上2倍方差
 float max_idepth = target->idepth_smoothed + sv*STEREO_EPL_VAR_FAC;
 if(min_idepth < 0) min_idepth = 0;
 if(max_idepth > 1/MIN_DEPTH) max_idepth = 1/MIN_DEPTH;
 stats->num_observe_update_attempted++;
 float result_idepth, result_var, result_eplLength;
 float error = doLineStereo(
   x,y,epx,epy,
   0.0f, 1.0f, 1.0f / MIN_DEPTH,
   refFrame, refFrame->image(0),
   result_idepth, result_var, result_eplLength, stats);
 
 float diff;
 //if (target->isValid == false)//wx-debug
 // diff == 0;//wx-debug
 //else //wx-debug
  diff = result_idepth - target->idepth_smoothed;//wx- 新计算的深度和之前深度之间的差值

 // if oob: (really out of bounds)//wx- 如果深度图上的极线或者参考帧上的极线超出了图片或离边缘太近就返回-1
 if(error == -1)
 {
  // do nothing, pixel got oob, but is still in bounds in original. I will want to try again.
  if(enablePrintDebugInfo) stats->num_observe_skip_oob++;
  if(plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,255); // RED FOR OOB
  return false;
 }
 // if just not good for stereo (e.g. some inf / nan occured; has inconsistent minimum; ..)
 else if(error == -2)
 {
  if(enablePrintDebugInfo) stats->num_observe_skip_fail++;
  //if(plotStereoImages)//wx-debug
   //debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,255);//wx-debug // PURPLE FOR NON-GOOD //wx- 即利用该像素点匹配误差太大、最好的匹配和次好的差值过小等等匹配问题重建结果可能不好，由于semi-dense 所以舍弃这些点

  target->validity_counter -= VALIDITY_COUNTER_DEC;
  if(target->validity_counter < 0) target->validity_counter = 0;

  target->nextStereoFrameMinID = 0;
  target->idepth_var *= FAIL_VAR_INC_FAC;
  if(target->idepth_var > MAX_VAR)
  {
   //target->isValid = false;              //wx-debug 这里可能会删除之前正确的重建结果，即在当前图片上该深度图投影到当前帧之后对应的像素点，从求得的方差估计来说不适合做重建。由于遮挡，跟踪的精度，边缘(重建结果多在梯度大值)等，很可能之前结果是正确的，投影到当前帧之后对应错误或者当前帧的情况有变化，但不该删掉之前的点。
   //target->blacklisted--;               //wx-这个情况非常多见，很多的点就是因为这个问题被删掉，但是这个地方又涉及到当前深度图的semi-dense的准则问题，按照理论确实这个点不该有结果
  }
  return false;
 }
 // if not found (error too high)
 else if(error == -3)
 {
  if(enablePrintDebugInfo) stats->num_observe_notfound++;
  if(plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0); // BLACK FOR big not-found

  return false;
 }
 else if(error == -4)
 {
  if(plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0); // BLACK FOR big arithmetic error
  return false;
 }
 // if inconsistent
 else if(DIFF_FAC_OBSERVE*diff*diff > result_var + target->idepth_var_smoothed)//wx- 如果之前的深度和本次计算出来的深度相差太大则将该点的深度删掉，这可能是由于跟踪错误了，或者重建匹配错误了等等情况导致。
 {
  if(enablePrintDebugInfo) stats->num_observe_inconsistent++;
  if(plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,0); // Turkoise FOR big inconsistent
  target->idepth_var *= FAIL_VAR_INC_FAC;
  if(target->idepth_var > MAX_VAR) target->isValid = false;      //wx-debug 这里可能删除之前正确的重建结果，很多地方都是由于这个问题被删除和替换。但是很有可能之前的重建结果是正确的，或者本次的重接结果是正确的，但是却被删除了。这里就是我们经常遇到的前几帧明明重建结果很好，后来被删除了的主要来源\
                         另一个来源应该在更新深度时当前帧的方差太大，认为这帧的重建结果很不可靠，所以将当前帧的重建结果连同之前的结果都删掉。这样做的考虑可能是考虑到跟踪总会有问题，要长时间的进行重建的时候，直接拿之后新的数据替换之前的旧数据，总能在未来得到更相容的结果使结果更好看？
  return false;
 }
 else if (target->isValid == false)//wx-debug
 {
  *target = DepthMapPixelHypothesis(
   result_idepth,
   result_var,
   VALIDITY_COUNTER_INITIAL_OBSERVE);
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 255); // Orange for tracking结果不好然后重新建立深度的
 }
 else
 {
  // one more successful observation!
  if(enablePrintDebugInfo) stats->num_observe_good++;
  if(enablePrintDebugInfo) stats->num_observe_updated++;

  // do textbook ekf update:
  // increase var by a little (prediction-uncertainty)
  float id_var = target->idepth_var*SUCC_VAR_INC_FAC;    //wx- 这里就按照EKF的方式更新了深度和方差
  // update var with observation
  float w = result_var / (result_var + id_var);
  float new_idepth = (1-w)*result_idepth + w*target->idepth;
  target->idepth = UNZERO(new_idepth);
  //guo-这里更新深度
  // variance can only decrease from observation; never increase.
  id_var = id_var * w;
  if(id_var < target->idepth_var)
   target->idepth_var = id_var;
  //guo-这里更新方差
  // increase validity!
  target->validity_counter += VALIDITY_COUNTER_INC;
  float absGrad = keyFrameMaxGradBuf[idx];
  if(target->validity_counter > VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f)
   target->validity_counter = VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f;
  // increase Skip!
  if(result_eplLength < MIN_EPL_LENGTH_CROP)           //wx- 这里按照某种神秘的方式根据参考帧上极线的长度设定了一个不再更新深度的时间窗，即直到新进来的帧的ID到nextStereoFrameMinID为止，不再更新这个像素
  {
   float inc = activeKeyFrame->numFramesTrackedOnThis / (float)(activeKeyFrame->numMappedOnThis+5);
   if(inc < 3) inc = 3;
   inc +=  ((int)(result_eplLength*10000)%2);
   if(enablePrintDebugInfo) stats->num_observe_addSkip++;
   if(result_eplLength < 0.5*MIN_EPL_LENGTH_CROP)
    inc *= 3;

   target->nextStereoFrameMinID = refFrame->id() + inc;
  }
  if(plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,255,255); // yellow for GOT UPDATED
  return true;
 }
}
void DepthMap::propagateDepth(Frame* new_keyframe)//wx-为了找到旧关键帧中各像素的深度信息与新关键帧各像素之间的对应关系，先将旧关键帧中的像素点反投影的三维空间，再从三维空间投影到新关键帧，完成深度的传递
{
 runningStats.num_prop_removed_out_of_bounds = 0;
 runningStats.num_prop_removed_colorDiff = 0;
 runningStats.num_prop_removed_validity = 0;
 runningStats.num_prop_grad_decreased = 0;
 runningStats.num_prop_color_decreased = 0;
 runningStats.num_prop_attempts = 0;
 runningStats.num_prop_occluded = 0;
 runningStats.num_prop_created = 0;
 runningStats.num_prop_merged = 0;

 if(new_keyframe->getTrackingParent() != activeKeyFrame)
 {
  printf("WARNING: propagating depth from frame %d to %d, which was tracked on a different frame (%d).\nWhile this should work, it is not recommended.",
    activeKeyFrame->id(), new_keyframe->id(),
    new_keyframe->getTrackingParent()->id());
 }
 // wipe depthmap
 for(DepthMapPixelHypothesis* pt = otherDepthMap+width*height-1; pt >= otherDepthMap; pt--)//wx-清空otherDepthMap，之后将以otherDepthMap为中转，将所有信息整合到otherDepthMap中，然后交换otherDepthMap和currentDepthMap
 {
  pt->isValid = false;
  pt->blacklisted = 0;
 }
 // re-usable values.
 SE3 oldToNew_SE3 = se3FromSim3(new_keyframe->pose->thisToParent_raw).inverse();
 Eigen::Vector3f trafoInv_t = oldToNew_SE3.translation().cast<float>();
 Eigen::Matrix3f trafoInv_R = oldToNew_SE3.rotationMatrix().matrix().cast<float>();

 const bool* trackingWasGood = new_keyframe->getTrackingParent() == activeKeyFrame ? new_keyframe->refPixelWasGoodNoCreate() : 0;

 const float* activeKFImageData = activeKeyFrame->image(0);
 const float* newKFMaxGrad = new_keyframe->maxGradients(0);
 const float* newKFImageData = new_keyframe->image(0);
 //*************************************edit-by-wx 2015 12 18//wx-debug
#ifdef SHOWPROPAGATE
 cv::Vec3b* depthCImageData;
 cv::Mat idepthImage;
 float* imageData = new_keyframe->image(0);
 idepthImage = cv::Mat(height, width, CV_8U);
 uchar* depthImageData = idepthImage.data;
 for (int i = 0; i < width*height; i++)
 {
  depthImageData[i] = (uchar)imageData[i];
 }
 cv::cvtColor(idepthImage, idepthImage, cv::COLOR_GRAY2BGR);
 depthCImageData = (cv::Vec3b*)idepthImage.data;

#endif

 //*************************************

 // go through all pixels of OLD image, propagating forwards.
 for(int y=0;y<height;y++)
  for(int x=0;x<width;x++)
  {
   DepthMapPixelHypothesis* source = currentDepthMap + x + y*width;
   if(!source->isValid)
    continue;
   if(enablePrintDebugInfo) runningStats.num_prop_attempts++;

   Eigen::Vector3f pn = (trafoInv_R * Eigen::Vector3f(x*fxi + cxi,y*fyi + cyi,1.0f)) / source->idepth_smoothed + trafoInv_t;//wx-先将像素点投影到三维空间
   float new_idepth = 1.0f / pn[2];
   float u_new = pn[0]*new_idepth*fx + cx;//wx-从三维空间将点投影到新的关键帧
   float v_new = pn[1]*new_idepth*fy + cy;
   // check if still within image, if not: DROP.
   if(!(u_new > 2.1f && v_new > 2.1f && u_new < width-3.1f && v_new < height-3.1f))
   {
    if(enablePrintDebugInfo) runningStats.num_prop_removed_out_of_bounds++;
    continue;
   }
   int newIDX = (int)(u_new+0.5f) + ((int)(v_new+0.5f))*width;
   float destAbsGrad = newKFMaxGrad[newIDX];
   //*****************************//edit-by-wx 2015-12-18///wx-debug
#ifdef SHOWPROPAGATE
   depthCImageData[newIDX] = cv::Vec3b(0, 0, 255);
#endif
   //*****************************
   if(trackingWasGood != 0)//wx-如果tracking的结果不好，检验两个像素的颜色相容程度，不相容则不传递？
   {
    if(!trackingWasGood[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)]
                        || destAbsGrad < MIN_ABS_GRAD_DECREASE)
    {
     if(enablePrintDebugInfo) runningStats.num_prop_removed_colorDiff++;
     continue;
    }
   }
   else
   {
    float sourceColor = activeKFImageData[x + y*width];
    float destColor = getInterpolatedElement(newKFImageData, u_new, v_new, width);
    float residual = destColor - sourceColor;

    if(residual*residual / (MAX_DIFF_CONSTANT + MAX_DIFF_GRAD_MULT*destAbsGrad*destAbsGrad) > 1.0f || destAbsGrad < MIN_ABS_GRAD_DECREASE)
    {
     if(enablePrintDebugInfo) runningStats.num_prop_removed_colorDiff++;
     continue;
    }
   }
   DepthMapPixelHypothesis* targetBest = otherDepthMap +  newIDX;
   // large idepth = point is near = large increase in variance.
   // small idepth = point is far = small increase in variance.
   float idepth_ratio_4 = new_idepth / source->idepth_smoothed;
   idepth_ratio_4 *= idepth_ratio_4;
   idepth_ratio_4 *= idepth_ratio_4;
   float new_var =idepth_ratio_4*source->idepth_var;
   //*****************************//edit-by-wx 2015-12-18///wx-debug
#ifdef SHOWPROPAGATE
   depthCImageData[newIDX] = cv::Vec3b(255, 0, 0);
#endif
   //*****************************
   // check for occlusion
   if(targetBest->isValid)//wx-由于可能出现三维空间中多个点投影到新的关键帧中对应到同一个像素的情况，所以检查是否这个像素已经被投影过了即是否出现occlusion
                        //检查遮挡，是判断是否超出方差浮动的范围，如果超出则认为这个点不对应三维空间同一个点，则再利用深度判断这两个点谁离摄像机近，如果新点离摄像机更近则认为新点遮挡旧点
         //如果新点遮挡旧点则将旧点的状态置为空，等待之后将新点赋值给该像素的过程
   {
    // if they occlude one another, one gets removed.
    float diff = targetBest->idepth - new_idepth;
    if(DIFF_FAC_PROP_MERGE*diff*diff >
     new_var +
     targetBest->idepth_var)
    {
     if(new_idepth < targetBest->idepth)
     {
      if(enablePrintDebugInfo) runningStats.num_prop_occluded++;
      continue;
     }
     else
     {
      if(enablePrintDebugInfo) runningStats.num_prop_occluded++;
      targetBest->isValid = false;
     }
    }
   }

   if(!targetBest->isValid)
   {
    if(enablePrintDebugInfo) runningStats.num_prop_created++;//wx-新点遮挡旧点，用新点替换旧点的值
    *targetBest = DepthMapPixelHypothesis(
      new_idepth,
      new_var,
      source->validity_counter);
   }
   else
   {
    if(enablePrintDebugInfo) runningStats.num_prop_merged++;//wx-新点和旧点的距离再方差范围内，可能指向三维空间同一个点，进行融合(平滑)
    // merge idepth ekf-style
    float w = new_var / (targetBest->idepth_var + new_var);
    float merged_new_idepth = w*targetBest->idepth + (1.0f-w)*new_idepth;
    // merge validity
    int merged_validity = source->validity_counter + targetBest->validity_counter;
    if(merged_validity > VALIDITY_COUNTER_MAX+(VALIDITY_COUNTER_MAX_VARIABLE))
     merged_validity = VALIDITY_COUNTER_MAX+(VALIDITY_COUNTER_MAX_VARIABLE);
    *targetBest = DepthMapPixelHypothesis(
      merged_new_idepth,
      1.0f/(1.0f/targetBest->idepth_var + 1.0f/new_var),
      merged_validity);
   }
  }
 // swap!
 std::swap(currentDepthMap, otherDepthMap);//wx-将所有新传递了的深度信息放到了otherDepthMap里，然后交换了当前这个地址，相当于赋值给了currentDepthMap
#ifdef SHOWPROPAGATE
 cv::namedWindow("propagation", 0);
 cv::imshow("propagation", idepthImage);
 cv::waitKey(10);
#endif
 if(enablePrintDebugInfo && printPropagationStatistics)
 {
  printf("PROPAGATE: %d: %d drop (%d oob, %d color); %d created; %d merged; %d occluded. %d col-dec, %d grad-dec.\n",
    runningStats.num_prop_attempts,
    runningStats.num_prop_removed_validity + runningStats.num_prop_removed_out_of_bounds + runningStats.num_prop_removed_colorDiff,
    runningStats.num_prop_removed_out_of_bounds,
    runningStats.num_prop_removed_colorDiff,
    runningStats.num_prop_created,
    runningStats.num_prop_merged,
    runningStats.num_prop_occluded,
    runningStats.num_prop_color_decreased,
    runningStats.num_prop_grad_decreased);
 }
}

void DepthMap::regularizeDepthMapFillHolesRow(int yMin, int yMax, RunningStats* stats)
{
 // =========== regularize fill holes
 const float* keyFrameMaxGradBuf = activeKeyFrame->maxGradients(0);
 for(int y=yMin; y<yMax; y++)
 {
  for(int x=3;x<width-2;x++)
  {
   int idx = x+y*width;
   DepthMapPixelHypothesis* dest = otherDepthMap + idx;
   if(dest->isValid) continue;
   if(keyFrameMaxGradBuf[idx]<MIN_ABS_GRAD_DECREASE) continue;
   int* io = validityIntegralBuffer + idx;
   int val = io[2+2*width] - io[2-3*width] - io[-3+2*width] + io[-3-3*width];

   if((dest->blacklisted >= MIN_BLACKLIST && val > VAL_SUM_MIN_FOR_CREATE) || val > VAL_SUM_MIN_FOR_UNBLACKLIST)
   {
    float sumIdepthObs = 0, sumIVarObs = 0;
    int num = 0;
    DepthMapPixelHypothesis* s1max = otherDepthMap + (x-2) + (y+3)*width;
    for (DepthMapPixelHypothesis* s1 = otherDepthMap + (x-2) + (y-2)*width; s1 < s1max; s1+=width)
     for(DepthMapPixelHypothesis* source = s1; source < s1+5; source++)
     {
      if(!source->isValid) continue;
      sumIdepthObs += source->idepth /source->idepth_var;
      sumIVarObs += 1.0f/source->idepth_var;
      num++;
     }
    float idepthObs = sumIdepthObs / sumIVarObs;
    idepthObs = UNZERO(idepthObs);
    //guo-这里用之前计算的深度的一个均值来初始化下一帧的这点的深度
    currentDepthMap[idx] =
     DepthMapPixelHypothesis(
      idepthObs,
      VAR_RANDOM_INIT_INITIAL,
      0);
    if(enablePrintDebugInfo) stats->num_reg_created++;
   }
  }
 }
}

void DepthMap::regularizeDepthMapFillHoles()
{
 buildRegIntegralBuffer();
 runningStats.num_reg_created=0;
 memcpy(otherDepthMap,currentDepthMap,width*height*sizeof(DepthMapPixelHypothesis));
 threadReducer.reduce(boost::bind(&DepthMap::regularizeDepthMapFillHolesRow, this, _1, _2, _3), 3, height-2, 10);
 if(enablePrintDebugInfo && printFillHolesStatistics)
  printf("FillHoles (discreteDepth): %d created\n",
    runningStats.num_reg_created);
}

void DepthMap::buildRegIntegralBufferRow1(int yMin, int yMax, RunningStats* stats)
{
 // ============ build inegral buffers
 int* validityIntegralBufferPT = validityIntegralBuffer+yMin*width;
 DepthMapPixelHypothesis* ptSrc = currentDepthMap+yMin*width;
 for(int y=yMin;y<yMax;y++)
 {
  int validityIntegralBufferSUM = 0;
  for(int x=0;x<width;x++)
  {
   if(ptSrc->isValid)
    validityIntegralBufferSUM += ptSrc->validity_counter;
   *(validityIntegralBufferPT++) = validityIntegralBufferSUM;
   ptSrc++;
  }
 }
}

void DepthMap::buildRegIntegralBuffer()
{
 threadReducer.reduce(boost::bind(&DepthMap::buildRegIntegralBufferRow1, this, _1, _2,_3), 0, height);
 int* validityIntegralBufferPT = validityIntegralBuffer;
 int* validityIntegralBufferPT_T = validityIntegralBuffer+width;
 int wh = height*width;
 for(int idx=width;idx<wh;idx++)
  *(validityIntegralBufferPT_T++) += *(validityIntegralBufferPT++);
}

template<bool removeOcclusions> void DepthMap::regularizeDepthMapRow(int validityTH, int yMin, int yMax, RunningStats* stats)
{
 const int regularize_radius = 2;
 const float regDistVar = REG_DIST_VAR;
 for(int y=yMin;y<yMax;y++)
 {
  for(int x=regularize_radius;x<width-regularize_radius;x++)
  {
   DepthMapPixelHypothesis* dest = currentDepthMap + x + y*width;
   DepthMapPixelHypothesis* destRead = otherDepthMap + x + y*width;
   // if isValid need to do better examination and then update.
   if(enablePrintDebugInfo && destRead->blacklisted < MIN_BLACKLIST)
    stats->num_reg_blacklisted++;
   if(!destRead->isValid)
    continue;
   
   float sum=0, val_sum=0, sumIvar=0;//, min_varObs = 1e20;
   int numOccluding = 0, numNotOccluding = 0;
   for(int dx=-regularize_radius; dx<=regularize_radius;dx++)
    for(int dy=-regularize_radius; dy<=regularize_radius;dy++)
    {
     DepthMapPixelHypothesis* source = destRead + dx + dy*width;
     if(!source->isValid) continue;
//     stats->num_reg_total++;
     float diff =source->idepth - destRead->idepth;
     if(DIFF_FAC_SMOOTHING*diff*diff > source->idepth_var + destRead->idepth_var)
     {
      if(removeOcclusions)
      {
       if(source->idepth > destRead->idepth)
        numOccluding++;
      }
      continue;
     }
     val_sum += source->validity_counter;
     if(removeOcclusions)
      numNotOccluding++;
     float distFac = (float)(dx*dx+dy*dy)*regDistVar;
     float ivar = 1.0f/(source->idepth_var + distFac);
     sum += source->idepth * ivar;
     sumIvar += ivar;

    }
   if(val_sum < validityTH)
   {
    dest->isValid = false;
    if(enablePrintDebugInfo) stats->num_reg_deleted_secondary++;
    dest->blacklisted--;
    if(enablePrintDebugInfo) stats->num_reg_setBlacklisted++;
    continue;
   }

   if(removeOcclusions)
   {
    if(numOccluding > numNotOccluding)
    {
     dest->isValid = false;
     if(enablePrintDebugInfo) stats->num_reg_deleted_occluded++;
     continue;
    }
   }
   sum = sum / sumIvar;
   sum = UNZERO(sum);
  
   // update!
   dest->idepth_smoothed = sum;
   dest->idepth_var_smoothed = 1.0f/sumIvar;
   if(enablePrintDebugInfo) stats->num_reg_smeared++;
  }
 }
}
template void DepthMap::regularizeDepthMapRow<true>(int validityTH, int yMin, int yMax, RunningStats* stats);
template void DepthMap::regularizeDepthMapRow<false>(int validityTH, int yMin, int yMax, RunningStats* stats);

void DepthMap::regularizeDepthMap(bool removeOcclusions, int validityTH)
{
 runningStats.num_reg_smeared=0;
 runningStats.num_reg_total=0;
 runningStats.num_reg_deleted_secondary=0;
 runningStats.num_reg_deleted_occluded=0;
 runningStats.num_reg_blacklisted=0;
 runningStats.num_reg_setBlacklisted=0;
 memcpy(otherDepthMap,currentDepthMap,width*height*sizeof(DepthMapPixelHypothesis));

 if(removeOcclusions)
  threadReducer.reduce(boost::bind(&DepthMap::regularizeDepthMapRow<true>, this, validityTH, _1, _2, _3), 2, height-2, 10);
 else
  threadReducer.reduce(boost::bind(&DepthMap::regularizeDepthMapRow<false>, this, validityTH, _1, _2, _3), 2, height-2, 10);

 if(enablePrintDebugInfo && printRegularizeStatistics)
  printf("REGULARIZE (%d): %d smeared; %d blacklisted /%d new); %d deleted; %d occluded; %d filled\n",
    activeKeyFrame->id(),
    runningStats.num_reg_smeared,
    runningStats.num_reg_blacklisted,
    runningStats.num_reg_setBlacklisted,
    runningStats.num_reg_deleted_secondary,
    runningStats.num_reg_deleted_occluded,
    runningStats.num_reg_created);
}

void DepthMap::initializeRandomly(Frame* new_frame)
{
 activeKeyFramelock = new_frame->getActiveLock();
 activeKeyFrame = new_frame;
 activeKeyFrameImageData = activeKeyFrame->image(0);
 activeKeyFrameIsReactivated = false;
 //guo-这里初始化梯度
 const float* maxGradients = new_frame->maxGradients();
 const unsigned char* edge = new_frame->edgeFlag(0);
 for(int y=1;y<height-1;y++)
 {
  for(int x=1;x<width-1;x++)
  {
   //guo-这里初始化一个随机深度,并初始化feature
  // if(maxGradients[x+y*width] > MIN_ABS_GRAD_CREATE)
  if (edge[x + y*width]<50)
   {
    float idepth = 0.5f + 1.0f * ((rand() % 100001) / 100000.0f);
    currentDepthMap[x+y*width] = DepthMapPixelHypothesis(
      idepth,
      idepth,
      VAR_RANDOM_INIT_INITIAL,
      VAR_RANDOM_INIT_INITIAL,
      20);
   }
   else
   {
    currentDepthMap[x+y*width].isValid = false;
    currentDepthMap[x+y*width].blacklisted = 0;
   }
  }
 }
 MatVis::Vis()->VisIdepth(currentDepthMap, width, height);
 activeKeyFrame->setDepth(currentDepthMap);
}

void DepthMap::setFromExistingKF(Frame* kf)//wx- 仅在找到loopClosure或者进行reTrack时调用
{
 assert(kf->hasIDepthBeenSet());
 activeKeyFramelock = kf->getActiveLock();
 activeKeyFrame = kf;
 const float* idepth = activeKeyFrame->idepth_reAct();//wx- 这些变量都是loopClosure帧中对应的深度和方差
 const float* idepthVar = activeKeyFrame->idepthVar_reAct();
 const unsigned char* validity = activeKeyFrame->validity_reAct();
 DepthMapPixelHypothesis* pt = currentDepthMap;//wx- pt是当前深度图中的深度和方差
 activeKeyFrame->numMappedOnThis = 0;
 activeKeyFrame->numFramesTrackedOnThis = 0;
 activeKeyFrameImageData = activeKeyFrame->image(0);
 activeKeyFrameIsReactivated = true;
 //guo-在这里更新反深度和方差
 for(int y=0;y<height;y++)//wx- 这里在找到loopClosure(或reTrack)之后，用之前关键帧的深度全盘替换了当前帧的深度。感觉这样对LoopClosure不太合理
 {
  for(int x=0;x<width;x++)
  {
   if(*idepthVar > 0)
   {
    *pt = DepthMapPixelHypothesis(
      *idepth,
      *idepthVar,
      *validity);
   }
   else
   {
    currentDepthMap[x+y*width].isValid = false;
    currentDepthMap[x+y*width].blacklisted = (*idepthVar == -2) ? MIN_BLACKLIST-1 : 0;
   }
   idepth++;
   idepthVar++;
   validity++;
   pt++;
  }
 }
 regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
}

void DepthMap::initializeFromGTDepth(Frame* new_frame)
{
 assert(new_frame->hasIDepthBeenSet());
 activeKeyFramelock = new_frame->getActiveLock();
 activeKeyFrame = new_frame;
 activeKeyFrameImageData = activeKeyFrame->image(0);
 activeKeyFrameIsReactivated = false;
 const float* idepth = new_frame->idepth();

 float averageGTIDepthSum = 0;
 int averageGTIDepthNum = 0;
 for(int y=0;y<height;y++)
 {
  for(int x=0;x<width;x++)
  {
   float idepthValue = idepth[x+y*width];
   if(!isnan(idepthValue) && idepthValue > 0)
   {
    averageGTIDepthSum += idepthValue;
    averageGTIDepthNum ++;
   }
  }
 }
 
 for(int y=0;y<height;y++)
 {
  for(int x=0;x<width;x++)
  {
   float idepthValue = idepth[x+y*width];
   
   if(!isnan(idepthValue) && idepthValue > 0)
   {
    currentDepthMap[x+y*width] = DepthMapPixelHypothesis(
      idepthValue,
      idepthValue,
      VAR_GT_INIT_INITIAL,
      VAR_GT_INIT_INITIAL,
      20);
   }
   else
   {
    currentDepthMap[x+y*width].isValid = false;
    currentDepthMap[x+y*width].blacklisted = 0;
   }
  }
 }

 activeKeyFrame->setDepth(currentDepthMap);
}
void DepthMap::resetCounters()
{
 runningStats.num_stereo_comparisons=0;
 runningStats.num_pixelInterpolations=0;
 runningStats.num_stereo_calls = 0;
 runningStats.num_stereo_rescale_oob = 0;
 runningStats.num_stereo_inf_oob = 0;
 runningStats.num_stereo_near_oob = 0;
 runningStats.num_stereo_invalid_unclear_winner = 0;
 runningStats.num_stereo_invalid_atEnd = 0;
 runningStats.num_stereo_invalid_inexistantCrossing = 0;
 runningStats.num_stereo_invalid_twoCrossing = 0;
 runningStats.num_stereo_invalid_noCrossing = 0;
 runningStats.num_stereo_invalid_bigErr = 0;
 runningStats.num_stereo_interpPre = 0;
 runningStats.num_stereo_interpPost = 0;
 runningStats.num_stereo_interpNone = 0;
 runningStats.num_stereo_negative = 0;
 runningStats.num_stereo_successfull = 0;
 runningStats.num_observe_created=0;
 runningStats.num_observe_create_attempted=0;
 runningStats.num_observe_updated=0;
 runningStats.num_observe_update_attempted=0;
 runningStats.num_observe_skipped_small_epl=0;
 runningStats.num_observe_skipped_small_epl_grad=0;
 runningStats.num_observe_skipped_small_epl_angle=0;
 runningStats.num_observe_transit_finalizing=0;
 runningStats.num_observe_transit_idle_oob=0;
 runningStats.num_observe_transit_idle_scale_angle=0;
 runningStats.num_observe_trans_idle_exhausted=0;
 runningStats.num_observe_inconsistent_finalizing=0;
 runningStats.num_observe_inconsistent=0;
 runningStats.num_observe_notfound_finalizing2=0;
 runningStats.num_observe_notfound_finalizing=0;
 runningStats.num_observe_notfound=0;
 runningStats.num_observe_skip_fail=0;
 runningStats.num_observe_skip_oob=0;
 runningStats.num_observe_good=0;
 runningStats.num_observe_good_finalizing=0;
 runningStats.num_observe_state_finalizing=0;
 runningStats.num_observe_state_initializing=0;
 runningStats.num_observe_skip_alreadyGood=0;
 runningStats.num_observe_addSkip=0;

 runningStats.num_observe_blacklisted=0;
}

void DepthMap::updateKeyframe(std::deque< std::shared_ptr<Frame> > referenceFrames)//wx- 在mapping线程中被调用，利用unmappedTrackedFrames作为referenceFrames更新深度图中的深度
{
 assert(isValid());


#ifdef TIME_CAL
 struct timeval tv_start_all, tv_end_all;
 gettimeofday(&tv_start_all, NULL);
#endif
 oldest_referenceFrame = referenceFrames.front().get();
 newest_referenceFrame = referenceFrames.back().get();
 referenceFrameByID.clear();
 referenceFrameByID_offset = oldest_referenceFrame->id();//wx- 由于referenceFrameByID是局部的帧序列，为了能够按照Id访问，将帧号最小的帧作为offset，构造ByID的访问方式
// for(std::shared_ptr<Frame> frame : referenceFrames)
 for (std::deque< std::shared_ptr<Frame> >::iterator c = referenceFrames.begin() ; c != referenceFrames.end() ; c++)//wx- 对参考序列中的每帧都进行立体匹配
 {
  std::shared_ptr<Frame> frame = *c;
  assert(frame->hasTrackingParent());
  if(frame->getTrackingParent() != activeKeyFrame)
  {
   printf("WARNING: updating frame %d with %d, which was tracked on a different frame (%d).\nWhile this should work, it is not recommended.",
     activeKeyFrame->id(), frame->id(),
     frame->getTrackingParent()->id());
  }
  Sim3 refToKf;
  if(frame->pose->trackingParent->frameID == activeKeyFrame->id())
   refToKf = frame->pose->thisToParent_raw;
  else
   refToKf = activeKeyFrame->getScaledCamToWorld().inverse() *  frame->getScaledCamToWorld();
  //guo-计算帧与关键帧的相对运动,只是关键帧
  frame->prepareForStereoWith(activeKeyFrame, refToKf, K, 0);
  while((int)referenceFrameByID.size() + referenceFrameByID_offset <= frame->id())//wx- 利用referenceFrameByID_offset构造按照  [id-referenceFrameByID_offset] 的访问形式，将所有帧按照顺序添加到referenceFrameByID中
   referenceFrameByID.push_back(frame.get());
 }
 resetCounters();
 
 if(plotStereoImages)
 {
  cv::Mat keyFrameImage(activeKeyFrame->height(), activeKeyFrame->width(), CV_32F, const_cast<float*>(activeKeyFrameImageData));
  keyFrameImage.convertTo(debugImageHypothesisHandling, CV_8UC1);
  cv::cvtColor(debugImageHypothesisHandling, debugImageHypothesisHandling, CV_GRAY2RGB);
  cv::Mat oldest_refImage(oldest_referenceFrame->height(), oldest_referenceFrame->width(), CV_32F, const_cast<float*>(oldest_referenceFrame->image(0)));
  cv::Mat newest_refImage(newest_referenceFrame->height(), newest_referenceFrame->width(), CV_32F, const_cast<float*>(newest_referenceFrame->image(0)));
  cv::Mat rfimg = 0.5f*oldest_refImage + 0.5f*newest_refImage;
  rfimg.convertTo(debugImageStereoLines, CV_8UC1);
  cv::cvtColor(debugImageStereoLines, debugImageStereoLines, CV_GRAY2RGB);
 }
#ifdef TIME_CAL
 struct timeval tv_start, tv_end;

 gettimeofday(&tv_start, NULL);
#endif
 //guo-这里也更新深度
 observeDepth();
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msObserve = 0.9*msObserve + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nObserve++;
 //if(rand()%10==0)
 
  gettimeofday(&tv_start, NULL);
#endif
  //guo-这里根据更新后的深度计算一个平均深度
  regularizeDepthMapFillHoles();
#ifdef TIME_CAL
  gettimeofday(&tv_end, NULL);
  msFillHoles = 0.9*msFillHoles + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
  nFillHoles++;
 

 gettimeofday(&tv_start, NULL);
#endif
 //guo-感觉这里是做平滑
 regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msRegularize = 0.9*msRegularize + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nRegularize++;
#endif
 
 // Update depth in keyframe
 if(!activeKeyFrame->depthHasBeenUpdatedFlag)
 {
#ifdef TIME_CAL
  gettimeofday(&tv_start, NULL);
#endif
  activeKeyFrame->setDepth(currentDepthMap);
#ifdef TIME_CAL
  gettimeofday(&tv_end, NULL);
  msSetDepth = 0.9*msSetDepth + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
  nSetDepth++;
#endif
 }
#ifdef TIME_CAL
 gettimeofday(&tv_end_all, NULL);
 msUpdate = 0.9*msUpdate + 0.1*((tv_end_all.tv_sec-tv_start_all.tv_sec)*1000.0f + (tv_end_all.tv_usec-tv_start_all.tv_usec)/1000.0f);
 nUpdate++;
#endif

 activeKeyFrame->numMappedOnThis++;
 activeKeyFrame->numMappedOnThisTotal++;

 if(plotStereoImages)
 {
  //Util::displayImage( "Stereo Key Frame", debugImageHypothesisHandling, false );
  Util::displayImage( "Stereo Reference Frame", debugImageStereoLines, false );
 }

 if(enablePrintDebugInfo && printLineStereoStatistics)
 {
  printf("ST: calls %6d, comp %6d, int %7d; good %6d (%.0f%%), neg %6d (%.0f%%); interp %6d / %6d / %6d\n",
    runningStats.num_stereo_calls,
    runningStats.num_stereo_comparisons,
    runningStats.num_pixelInterpolations,
    runningStats.num_stereo_successfull,
    100*runningStats.num_stereo_successfull / (float) runningStats.num_stereo_calls,
    runningStats.num_stereo_negative,
    100*runningStats.num_stereo_negative / (float) runningStats.num_stereo_successfull,
    runningStats.num_stereo_interpPre,
    runningStats.num_stereo_interpNone,
    runningStats.num_stereo_interpPost);
 }
 if(enablePrintDebugInfo && printLineStereoFails)
 {
  printf("ST-ERR: oob %d (scale %d, inf %d, near %d); err %d (%d uncl; %d end; zro: %d btw, %d no, %d two; %d big)\n",
    runningStats.num_stereo_rescale_oob+
     runningStats.num_stereo_inf_oob+
     runningStats.num_stereo_near_oob,
    runningStats.num_stereo_rescale_oob,
    runningStats.num_stereo_inf_oob,
    runningStats.num_stereo_near_oob,
    runningStats.num_stereo_invalid_unclear_winner+
     runningStats.num_stereo_invalid_atEnd+
     runningStats.num_stereo_invalid_inexistantCrossing+
     runningStats.num_stereo_invalid_noCrossing+
     runningStats.num_stereo_invalid_twoCrossing+
     runningStats.num_stereo_invalid_bigErr,
    runningStats.num_stereo_invalid_unclear_winner,
    runningStats.num_stereo_invalid_atEnd,
    runningStats.num_stereo_invalid_inexistantCrossing,
    runningStats.num_stereo_invalid_noCrossing,
    runningStats.num_stereo_invalid_twoCrossing,
    runningStats.num_stereo_invalid_bigErr);
 }
}
void DepthMap::invalidate()
{
 if(activeKeyFrame==0) return;
 activeKeyFrame=0;
 activeKeyFramelock.unlock();
}
void DepthMap::createKeyFrame(Frame* new_keyframe)
{
 assert(isValid());
 assert(new_keyframe != nullptr);
 assert(new_keyframe->hasTrackingParent());
 //boost::shared_lock<boost::shared_mutex> lock = activeKeyFrame->getActiveLock();
 boost::shared_lock<boost::shared_mutex> lock2 = new_keyframe->getActiveLock();
#ifdef TIME_CAL
 struct timeval tv_start_all, tv_end_all;
 gettimeofday(&tv_start_all, NULL);
#endif

 resetCounters();
 if(plotStereoImages)
 {
  cv::Mat keyFrameImage(new_keyframe->height(), new_keyframe->width(), CV_32F, const_cast<float*>(new_keyframe->image(0)));
  keyFrameImage.convertTo(debugImageHypothesisPropagation, CV_8UC1);
  cv::cvtColor(debugImageHypothesisPropagation, debugImageHypothesisPropagation, CV_GRAY2RGB);
 }

 SE3 oldToNew_SE3 = se3FromSim3(new_keyframe->pose->thisToParent_raw).inverse();
#ifdef TIME_CAL
 struct timeval tv_start, tv_end;
 gettimeofday(&tv_start, NULL);
#endif
 propagateDepth(new_keyframe);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msPropagate = 0.9*msPropagate + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nPropagate++;
#endif
 activeKeyFrame = new_keyframe;
 activeKeyFramelock = activeKeyFrame->getActiveLock();
 activeKeyFrameImageData = new_keyframe->image(0);
 activeKeyFrameIsReactivated = false;

#ifdef TIME_CAL
 gettimeofday(&tv_start, NULL);
#endif
 regularizeDepthMap(true, VAL_SUM_MIN_FOR_KEEP);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msRegularize = 0.9*msRegularize + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nRegularize++;

 gettimeofday(&tv_start, NULL);
#endif
 regularizeDepthMapFillHoles();
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msFillHoles = 0.9*msFillHoles + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nFillHoles++;

 gettimeofday(&tv_start, NULL);
#endif
 regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msRegularize = 0.9*msRegularize + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nRegularize++;
#endif


 // make mean inverse depth be one.//wx-test
#ifdef RGBDSLAM
 float sumIdepth=0, numIdepth=0;
 for(DepthMapPixelHypothesis* source = currentDepthMap; source < currentDepthMap+width*height; source++)
 {
  if(!source->isValid)
   continue;
  sumIdepth += source->idepth_smoothed;
  numIdepth++;
 }
 float rescaleFactor = numIdepth / sumIdepth;
 float rescaleFactor2 = rescaleFactor*rescaleFactor;
 for(DepthMapPixelHypothesis* source = currentDepthMap; source < currentDepthMap+width*height; source++)
 {
  if(!source->isValid)
   continue;
  source->idepth *= rescaleFactor;
  source->idepth_smoothed *= rescaleFactor;
  source->idepth_var *= rescaleFactor2;
  source->idepth_var_smoothed *= rescaleFactor2;
 }

 activeKeyFrame->pose->thisToParent_raw = sim3FromSE3(oldToNew_SE3.inverse(), rescaleFactor);
#else
 activeKeyFrame->pose->thisToParent_raw = sim3FromSE3(oldToNew_SE3.inverse(), 1);//wx-test
#endif
 activeKeyFrame->pose->invalidateCache();
 // Update depth in keyframe
#ifdef TIME_CAL
 gettimeofday(&tv_start, NULL);
#endif
 activeKeyFrame->setDepth(currentDepthMap);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msSetDepth = 0.9*msSetDepth + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nSetDepth++;
 gettimeofday(&tv_end_all, NULL);
 msCreate = 0.9*msCreate + 0.1*((tv_end_all.tv_sec-tv_start_all.tv_sec)*1000.0f + (tv_end_all.tv_usec-tv_start_all.tv_usec)/1000.0f);
 nCreate++;
#endif

 if(plotStereoImages)
 {
  //Util::displayImage( "KeyFramePropagation", debugImageHypothesisPropagation );
 }
}
void DepthMap::addTimingSample()
{
#ifdef TIME_CAL
 struct timeval now;
 gettimeofday(&now, NULL);
 float sPassed = ((now.tv_sec-lastHzUpdate.tv_sec) + (now.tv_usec-lastHzUpdate.tv_usec)/1000000.0f);
 if(sPassed > 1.0f)
 {
  nAvgUpdate = 0.8*nAvgUpdate + 0.2*(nUpdate / sPassed); nUpdate = 0;
  nAvgCreate = 0.8*nAvgCreate + 0.2*(nCreate / sPassed); nCreate = 0;
  nAvgFinalize = 0.8*nAvgFinalize + 0.2*(nFinalize / sPassed); nFinalize = 0;
  nAvgObserve = 0.8*nAvgObserve + 0.2*(nObserve / sPassed); nObserve = 0;
  nAvgRegularize = 0.8*nAvgRegularize + 0.2*(nRegularize / sPassed); nRegularize = 0;
  nAvgPropagate = 0.8*nAvgPropagate + 0.2*(nPropagate / sPassed); nPropagate = 0;
  nAvgFillHoles = 0.8*nAvgFillHoles + 0.2*(nFillHoles / sPassed); nFillHoles = 0;
  nAvgSetDepth = 0.8*nAvgSetDepth + 0.2*(nSetDepth / sPassed); nSetDepth = 0;
  lastHzUpdate = now;
  if(enablePrintDebugInfo && printMappingTiming)
  {
   printf("Upd %3.1fms (%.1fHz); Create %3.1fms (%.1fHz); Final %3.1fms (%.1fHz) // Obs %3.1fms (%.1fHz); Reg %3.1fms (%.1fHz); Prop %3.1fms (%.1fHz); Fill %3.1fms (%.1fHz); Set %3.1fms (%.1fHz)\n",
     msUpdate, nAvgUpdate,
     msCreate, nAvgCreate,
     msFinalize, nAvgFinalize,
     msObserve, nAvgObserve,
     msRegularize, nAvgRegularize,
     msPropagate, nAvgPropagate,
     msFillHoles, nAvgFillHoles,
     msSetDepth, nAvgSetDepth);
  }
 }
#endif
}
void DepthMap::finalizeKeyFrame()
{
 assert(isValid());
#ifdef TIME_CAL
 struct timeval tv_start_all, tv_end_all;
 gettimeofday(&tv_start_all, NULL);
 struct timeval tv_start, tv_end;
 gettimeofday(&tv_start, NULL);
#endif
 regularizeDepthMapFillHoles();
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msFillHoles = 0.9*msFillHoles + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nFillHoles++;
 gettimeofday(&tv_start, NULL);
#endif
 regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msRegularize = 0.9*msRegularize + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nRegularize++;
 gettimeofday(&tv_start, NULL);
#endif
 activeKeyFrame->setDepth(currentDepthMap);
 activeKeyFrame->calculateMeanInformation();
 activeKeyFrame->takeReActivationData(currentDepthMap);
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msSetDepth = 0.9*msSetDepth + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nSetDepth++;
 gettimeofday(&tv_end_all, NULL);
 msFinalize = 0.9*msFinalize + 0.1*((tv_end_all.tv_sec-tv_start_all.tv_sec)*1000.0f + (tv_end_all.tv_usec-tv_start_all.tv_usec)/1000.0f);
 nFinalize++;
#endif
}


int DepthMap::debugPlotDepthMap()//wx-bug 当前程序中这个函数好像有bug
{
 if(activeKeyFrame == 0) return 1;
 cv::Mat keyFrameImage(activeKeyFrame->height(), activeKeyFrame->width(), CV_32F, const_cast<float*>(activeKeyFrameImageData));
 keyFrameImage.convertTo(debugImageDepth, CV_8UC1);
 cv::cvtColor(debugImageDepth, debugImageDepth, CV_GRAY2RGB);
 // debug plot & publish sparse version?
 int refID = referenceFrameByID_offset;

 for(int y=0;y<height;y++)
  for(int x=0;x<width;x++)
  {
   int idx = x + y*width;
   if(currentDepthMap[idx].blacklisted < MIN_BLACKLIST && debugDisplay == 2)
    debugImageDepth.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);
   if(!currentDepthMap[idx].isValid) continue;
   cv::Vec3b color = currentDepthMap[idx].getVisualizationColor(refID);//wx-bug 这里会奔溃，观察后refID一直没被正确赋值，不知道是不是这个原因
   debugImageDepth.at<cv::Vec3b>(y,x) = color;
  }

 return 1;
}


// find pixel in image (do stereo along epipolar line).
// mat: NEW image
// KinvP: point in OLD image (Kinv * (u_old, v_old, 1)), projected
// trafo: x_old = trafo * x_new; (from new to old image)
// realVal: descriptor in OLD image.
// returns: result_idepth : point depth in new camera's coordinate system
// returns: result_u/v : point's coordinates in new camera's coordinate system
// returns: idepth_var: (approximated) measurement variance of inverse depth of result_point_NEW
// returns error if sucessful; -1 if out of bounds, -2 if not found.
inline float DepthMap::doLineStereo(
 const float u, const float v, const float epxn, const float epyn,
 const float min_idepth, const float prior_idepth, float max_idepth,
 const Frame* const referenceFrame, const float* referenceFrameImage,
 float &result_idepth, float &result_var, float &result_eplLength,
 RunningStats* stats)
{
 if(enablePrintDebugInfo) stats->num_stereo_calls++;
 // calculate epipolar line start and end point in old image
 Eigen::Vector3f KinvP = Eigen::Vector3f(fxi*u+cxi,fyi*v+cyi,1.0f);//wx- 这里是u和v是遍历深度图时深度图上的坐标，将深度图中(u,v)点且深度为1时在三维空间中的点//guo-焦距逆
 Eigen::Vector3f pInf = referenceFrame->K_otherToThis_R * KinvP;//wx- KinvP点深度为d时投影到参考图中的完整式子应该是  d  *  referenceFrame->K_otherToThis_R * KinvP  +  referenceFrame->K_otherToThis_t，但是当深度d趋近无穷时，后面的附件项被忽略，转换齐次坐标时无穷远值d也会被消除，但是此时的Pinf的齐次项还不是1
 Eigen::Vector3f pReal = pInf / prior_idepth + referenceFrame->K_otherToThis_t;//guo-真实的位置//wx- 如果知道深度是(x,y,pri)则对应的点投影的位置，齐次项不等于1
             //wx-此时的pReal[2]这个齐次项不为1，其实对应的是投影到参考图上的像素之后，该像素对应的深度值。参考K.inv * pReal，会返回到三维空间中的那个投影点
 float rescaleFactor = pReal[2] * prior_idepth;//wx- pri_depth对应的是同一个三维点在深度图中的深度，pReal[2]对应的是投影点投影到参考图之后的深度
     
            
 float firstX = u - 2*epxn*rescaleFactor;//wx- rescaleFactor按照推论来说应该是两个图片上对应极线的长度比，即从无穷远点到极点的连线的长度比，这里没太理解这个写法
 float firstY = v - 2*epyn*rescaleFactor;//guo-这个尺度矫正可以说是调整分辨率,安装引文iccv13的引文[5],误差匹配跟baseline*分辨率是线性关系的,调整分辨率可以使深度的误差上限保持一个常数
 float lastX = u + 2*epxn*rescaleFactor;
 float lastY = v + 2*epyn*rescaleFactor;
 // width - 2 and height - 2 comes from the one-sided gradient calculation at the bottom
 if (firstX <= 0 || firstX >= width - 2//wx 由于极线可能在图片边缘，搜索第一组五个点时就可能超出图片范围，所以先检查这个位置的找对应的合理性
  || firstY <= 0 || firstY >= height - 2
  || lastX <= 0 || lastX >= width - 2
  || lastY <= 0 || lastY >= height - 2) 
 {
	return -1;
 }
 if(!(rescaleFactor > 0.7f && rescaleFactor < 1.4f))
 {
	  if(enablePrintDebugInfo) stats->num_stereo_rescale_oob++;
	  return -1;
 }
 // calculate values to search for//wx-temp 从当前深度图对应的图片中沿着极线，取当前点前后四个点//guo-这个采样应该是有金字塔尺度缩放的
 float realVal_p1 = getInterpolatedElement(activeKeyFrameImageData,u + epxn*rescaleFactor, v + epyn*rescaleFactor, width);
 float realVal_m1 = getInterpolatedElement(activeKeyFrameImageData,u - epxn*rescaleFactor, v - epyn*rescaleFactor, width);
 float realVal = getInterpolatedElement(activeKeyFrameImageData,u, v, width);
 float realVal_m2 = getInterpolatedElement(activeKeyFrameImageData,u - 2*epxn*rescaleFactor, v - 2*epyn*rescaleFactor, width);
 float realVal_p2 = getInterpolatedElement(activeKeyFrameImageData,u + 2*epxn*rescaleFactor, v + 2*epyn*rescaleFactor, width);

// if(referenceFrame->K_otherToThis_t[2] * max_idepth + pInf[2] < 0.01)

 Eigen::Vector3f pClose = pInf + referenceFrame->K_otherToThis_t*max_idepth;//实际对应到参考图上的取深度最大的值时的点，可以认为是参考图上极线的终点,之后进行齐次化
 // if the assumed close-point lies behind the
 // image, have to change that.
 if(pClose[2] < 0.001f)
 {
	  max_idepth = (0.001f-pInf[2]) / referenceFrame->K_otherToThis_t[2];
	  pClose = pInf + referenceFrame->K_otherToThis_t*max_idepth;
 }
 pClose = pClose / pClose[2]; // pos in new image of point (xy), assuming max_idepth
 Eigen::Vector3f pFar = pInf + referenceFrame->K_otherToThis_t*min_idepth;//实际对应到参考图上的取深度最小值时的点，可以认为是参考图上的极线的开始，之后进行齐次化
 // if the assumed far-point lies behind the image or closter than the near-point,
 // we moved past the Point it and should stop.
 if(pFar[2] < 0.001f || max_idepth < min_idepth)
 {
	  if(enablePrintDebugInfo) stats->num_stereo_inf_oob++;
	  return -1;
 }
 pFar = pFar / pFar[2]; // pos in new image of point (xy), assuming min_idepth

 // check for nan due to eg division by zero.
 if(isnan((float)(pFar[0]+pClose[0])))
  return -4;
 // calculate increments in which we will step through the epipolar line.
 // they are sampleDist (or half sample dist) long
 float incx = pClose[0] - pFar[0];
 float incy = pClose[1] - pFar[1];
 float eplLength = sqrt(incx*incx+incy*incy);
 if(!eplLength > 0 || isinf(eplLength)) return -4;
 if(eplLength > MAX_EPL_LENGTH_CROP)//如果极线太长，顺着极线取到最长
 {
	  pClose[0] = pFar[0] + incx*MAX_EPL_LENGTH_CROP/eplLength;
	  pClose[1] = pFar[1] + incy*MAX_EPL_LENGTH_CROP/eplLength;
 }
 incx *= GRADIENT_SAMPLE_DIST/eplLength;//将incx转化成搜索的步长
 incy *= GRADIENT_SAMPLE_DIST/eplLength;

 // extend one sample_dist to left & right.
 pFar[0] -= incx;//wx-temp 顺着极线往前往后各延长一个步长
 pFar[1] -= incy;
 pClose[0] += incx;
 pClose[1] += incy;

 // make epl long enough (pad a little bit).//wx-temp 刚刚对极线的延长也有可能是缩减，可能取决于符号，再次判断距离是否够长
            //guo-这个就是延长,pFar的大小和incx的符号是同步变换的
 if(eplLength < MIN_EPL_LENGTH_CROP)
 {
	  float pad = (MIN_EPL_LENGTH_CROP - (eplLength)) / 2.0f;
	  pFar[0] -= incx*pad;
	  pFar[1] -= incy*pad;
	  pClose[0] += incx*pad;
	  pClose[1] += incy*pad;
 }
 // if inf point is outside of image: skip pixel.
 if(
   pFar[0] <= SAMPLE_POINT_TO_BORDER ||
   pFar[0] >= width-SAMPLE_POINT_TO_BORDER ||
   pFar[1] <= SAMPLE_POINT_TO_BORDER ||
   pFar[1] >= height-SAMPLE_POINT_TO_BORDER)
 {
	  if(enablePrintDebugInfo) stats->num_stereo_inf_oob++;
	  return -1;
 }

 // if near point is outside: move inside, and test length again.//wx-temp 参考图上极线的终点如果超出范围，修正回图片范围内
 if(
   pClose[0] <= SAMPLE_POINT_TO_BORDER ||
   pClose[0] >= width-SAMPLE_POINT_TO_BORDER ||
   pClose[1] <= SAMPLE_POINT_TO_BORDER ||
   pClose[1] >= height-SAMPLE_POINT_TO_BORDER)
 {
	  if(pClose[0] <= SAMPLE_POINT_TO_BORDER)
	  {
		   float toAdd = (SAMPLE_POINT_TO_BORDER - pClose[0]) / incx;
		   pClose[0] += toAdd * incx;
		   pClose[1] += toAdd * incy;
	  }
	  else if(pClose[0] >= width-SAMPLE_POINT_TO_BORDER)
	  {
		   float toAdd = (width-SAMPLE_POINT_TO_BORDER - pClose[0]) / incx;
		   pClose[0] += toAdd * incx;
		   pClose[1] += toAdd * incy;
	  }
	  if(pClose[1] <= SAMPLE_POINT_TO_BORDER)
	  {
		   float toAdd = (SAMPLE_POINT_TO_BORDER - pClose[1]) / incy;
		   pClose[0] += toAdd * incx;
		   pClose[1] += toAdd * incy;
	  }
	  else if(pClose[1] >= height-SAMPLE_POINT_TO_BORDER)
	  {
		   float toAdd = (height-SAMPLE_POINT_TO_BORDER - pClose[1]) / incy;
		   pClose[0] += toAdd * incx;
		   pClose[1] += toAdd * incy;
	  }
	  // get new epl length
	  float fincx = pClose[0] - pFar[0];
	  float fincy = pClose[1] - pFar[1];
	  float newEplLength = sqrt(fincx*fincx+fincy*fincy);
	  // test again
	  if(
		pClose[0] <= SAMPLE_POINT_TO_BORDER ||
		pClose[0] >= width-SAMPLE_POINT_TO_BORDER ||
		pClose[1] <= SAMPLE_POINT_TO_BORDER ||
		pClose[1] >= height-SAMPLE_POINT_TO_BORDER ||
		newEplLength < 8.0f
		)
	  {
		   if(enablePrintDebugInfo) stats->num_stereo_near_oob++;
		   return -1;
	  }

 }

 // from here on:
 // - pInf: search start-point
 // - p0: search end-point
 // - incx, incy: search steps in pixel
 // - eplLength, min_idepth, max_idepth: determines search-resolution, i.e. the result's variance.

 float cpx = pFar[0];//从极线的开始端开始搜索
 float cpy =  pFar[1];
 float val_cp_m2 = getInterpolatedElement(referenceFrameImage,cpx-2.0f*incx, cpy-2.0f*incy, width);//通过差值获得与在当前深度图上采集的五个点可能对应的点，但是不知道为什么这样计算就能使这些点之间的间隔对应
 float val_cp_m1 = getInterpolatedElement(referenceFrameImage,cpx-incx, cpy-incy, width);
 float val_cp = getInterpolatedElement(referenceFrameImage,cpx, cpy, width);
 float val_cp_p1 = getInterpolatedElement(referenceFrameImage,cpx+incx, cpy+incy, width);
 float val_cp_p2;

 const unsigned char* edgeFlag = const_cast<Frame*>(referenceFrame)->edgeFlag(0);
 /*
  * Subsequent exact minimum is found the following way:
  * - assuming lin. interpolation, the gradient of Error at p1 (towards p2) is given by
  *   dE1 = -2sum(e1*e1 - e1*e2)
  *   where e1 and e2 are summed over, and are the residuals (not squared).
  *
  * - the gradient at p2 (coming from p1) is given by
  *   dE2 = +2sum(e2*e2 - e1*e2)
  *
  * - linear interpolation => gradient changes linearely; zero-crossing is hence given by
  *   p1 + d*(p2-p1) with d = -dE1 / (-dE1 + dE2).
  *
  *
  *
  * => I for later exact min calculation, I need sum(e_i*e_i),sum(e_{i-1}*e_{i-1}),sum(e_{i+1}*e_{i+1})
  *    and sum(e_i * e_{i-1}) and sum(e_i * e_{i+1}),
  *    where i is the respective winning index.
  */

 // walk in equally sized steps, starting at depth=infinity.
 int loopCounter = 0;
 float best_match_x = -1;
 float best_match_y = -1;
 float best_match_err = 1e30;
 float second_best_match_err = 1e30;
 // best pre and post errors.
//  float best_match_errPre=NAN, best_match_errPost=NAN, best_match_DiffErrPre=NAN, best_match_DiffErrPost=NAN;
 float best_match_errPre=0, best_match_errPost=0, best_match_DiffErrPre=0, best_match_DiffErrPost=0;
 bool bestWasLastLoop = false;
 float eeLast = -1; // final error of last comp.
 // alternating intermediate vars
//guo-change
 float e1A=0, e1B=0, e2A=0, e2B=0, e3A=0, e3B=0, e4A=0, e4B=0, e5A=0, e5B=0;
//  float e1A=NAN, e1B=NAN, e2A=NAN, e2B=NAN, e3A=NAN, e3B=NAN, e4A=NAN, e4B=NAN, e5A=NAN, e5B=NAN;
 int loopCBest=-1, loopCSecond =-1;

 bool findEdge = false;

 while(((incx < 0) == (cpx > pClose[0]) && (incy < 0) == (cpy > pClose[1])) || loopCounter == 0)
 {
	  // interpolate one new point
	  val_cp_p2 = getInterpolatedElement(referenceFrameImage,cpx+2*incx, cpy+2*incy, width);
	  uchar edgeFlagP = getInterpolatedElement(edgeFlag, cpx, cpy, width);
	  //std::cout << (int)edgeFlagP << std::endl;

		  // hacky but fast way to get error and differential error: switch buffer variables for last loop.
		  float ee = 0;
	if (edgeFlagP < 50)
	{
			  findEdge = true;
		  if (loopCounter % 2 == 0)
		  {
			  // calc error and accumulate sums.
			  e1A = val_cp_p2 - realVal_p2; ee += e1A*e1A;
			  e2A = val_cp_p1 - realVal_p1; ee += e2A*e2A;
			  e3A = val_cp - realVal;      ee += e3A*e3A;
			  e4A = val_cp_m1 - realVal_m1; ee += e4A*e4A;
			  e5A = val_cp_m2 - realVal_m2; ee += e5A*e5A;
		  }
		  else
		  {
			  // calc error and accumulate sums.
			  e1B = val_cp_p2 - realVal_p2; ee += e1B*e1B;
			  e2B = val_cp_p1 - realVal_p1; ee += e2B*e2B;
			  e3B = val_cp - realVal;      ee += e3B*e3B;
			  e4B = val_cp_m1 - realVal_m1; ee += e4B*e4B;
			  e5B = val_cp_m2 - realVal_m2; ee += e5B*e5B;
		  }

		  // do I have a new winner??
		  // if so: set.
		  if (ee < best_match_err)
		  {
			  // put to second-best
			  second_best_match_err = best_match_err;
			  loopCSecond = loopCBest;
			  // set best.
			  best_match_err = ee;
			  loopCBest = loopCounter;
			  best_match_errPre = eeLast;
			  best_match_DiffErrPre = e1A*e1B + e2A*e2B + e3A*e3B + e4A*e4B + e5A*e5B;
			  best_match_errPost = -1;
			  best_match_DiffErrPost = -1;
			  best_match_x = cpx;
			  best_match_y = cpy;
			  bestWasLastLoop = true;
		  }
		  // otherwise: the last might be the current winner, in which case i have to save these values.
		  else
		  {
			  if (bestWasLastLoop)
			  {
				  best_match_errPost = ee;
				  best_match_DiffErrPost = e1A*e1B + e2A*e2B + e3A*e3B + e4A*e4B + e5A*e5B;
				  bestWasLastLoop = false;
			  }
			  // collect second-best:
			  // just take the best of all that are NOT equal to current best.
			  if (ee < second_best_match_err)
			  {
				  second_best_match_err = ee;
				  loopCSecond = loopCounter;
			  }
		  }
	}
		  // shift everything one further.
		eeLast = ee;
		val_cp_m2 = val_cp_m1; val_cp_m1 = val_cp; val_cp = val_cp_p1; val_cp_p1 = val_cp_p2;
		if (enablePrintDebugInfo) stats->num_stereo_comparisons++;
	  

	  cpx += incx;
	  cpy += incy;
	  loopCounter++;
 }

 if (!findEdge)
 {
	 return -3;
 }

 // if error too big, will return -3, otherwise -2.
 if(best_match_err > 4.0f*(float)MAX_ERROR_STEREO)
 {
	  if(enablePrintDebugInfo) stats->num_stereo_invalid_bigErr++;
	  return -3;
 }

 // check if clear enough winner
 if(abs(loopCBest - loopCSecond) > 1.0f && MIN_DISTANCE_ERROR_STEREO * best_match_err > second_best_match_err)
 {
	  if(enablePrintDebugInfo) stats->num_stereo_invalid_unclear_winner++;
	  return -2;
 }
 bool didSubpixel = false;
 if(useSubpixelStereo)
 {
	  // ================== compute exact match =========================
	  // compute gradients (they are actually only half the real gradient)
	  float gradPre_pre = -(best_match_errPre - best_match_DiffErrPre);
	  float gradPre_this = +(best_match_err - best_match_DiffErrPre);
	  float gradPost_this = -(best_match_err - best_match_DiffErrPost);
	  float gradPost_post = +(best_match_errPost - best_match_DiffErrPost);
	  // final decisions here.
	  bool interpPost = false;
	  bool interpPre = false;
	  // if one is oob: return false.
	  if(enablePrintDebugInfo && (best_match_errPre < 0 || best_match_errPost < 0))
	  {
		stats->num_stereo_invalid_atEnd++;
	  }

	  // - if zero-crossing occurs exactly in between (gradient Inconsistent),
	  else if((gradPost_this < 0) ^ (gradPre_this < 0))
	  {
		   // return exact pos, if both central gradients are small compared to their counterpart.
		   if(enablePrintDebugInfo && (gradPost_this*gradPost_this > 0.1f*0.1f*gradPost_post*gradPost_post ||
			  gradPre_this*gradPre_this > 0.1f*0.1f*gradPre_pre*gradPre_pre))
			stats->num_stereo_invalid_inexistantCrossing++;
	  }
	  // if pre has zero-crossing
	  else if((gradPre_pre < 0) ^ (gradPre_this < 0))
	  {
		   // if post has zero-crossing
		   if((gradPost_post < 0) ^ (gradPost_this < 0))
		   {
			if(enablePrintDebugInfo) stats->num_stereo_invalid_twoCrossing++;
		   }
		   else
			interpPre = true;
	  }
	  // if post has zero-crossing
	  else if((gradPost_post < 0) ^ (gradPost_this < 0))
	  {
			interpPost = true;
	  }
	  // if none has zero-crossing
	  else
	  {
			if(enablePrintDebugInfo) stats->num_stereo_invalid_noCrossing++;
	  }

	  // DO interpolation!
	  // minimum occurs at zero-crossing of gradient, which is a straight line => easy to compute.
	  // the error at that point is also computed by just integrating.
	  if(interpPre)
	  {
		   float d = gradPre_this / (gradPre_this - gradPre_pre);
		   best_match_x -= d*incx;
		   best_match_y -= d*incy;
		   best_match_err = best_match_err - 2*d*gradPre_this - (gradPre_pre - gradPre_this)*d*d;
		   if(enablePrintDebugInfo) stats->num_stereo_interpPre++;
		   didSubpixel = true;
	  }
	  else if(interpPost)
	  {
		   float d = gradPost_this / (gradPost_this - gradPost_post);
		   best_match_x += d*incx;
		   best_match_y += d*incy;
		   best_match_err = best_match_err + 2*d*gradPost_this + (gradPost_post - gradPost_this)*d*d;
		   if(enablePrintDebugInfo) stats->num_stereo_interpPost++;
		   didSubpixel = true;
	  }
	  else
	  {
			if(enablePrintDebugInfo) stats->num_stereo_interpNone++;
	  }
 }

 // sampleDist is the distance in pixel at which the realVal's were sampled
 float sampleDist = GRADIENT_SAMPLE_DIST*rescaleFactor;
 float gradAlongLine = 0;
 float tmp = realVal_p2 - realVal_p1;  gradAlongLine+=tmp*tmp;
 tmp = realVal_p1 - realVal;  gradAlongLine+=tmp*tmp;
 tmp = realVal - realVal_m1;  gradAlongLine+=tmp*tmp;
 tmp = realVal_m1 - realVal_m2;  gradAlongLine+=tmp*tmp;
 gradAlongLine /= sampleDist*sampleDist;
 // check if interpolated error is OK. use evil hack to allow more error if there is a lot of gradient.
 if(best_match_err > (float)MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20)
 {
	  if(enablePrintDebugInfo) stats->num_stereo_invalid_bigErr++;
	  return -3;
 }

 // ================= calc depth (in KF) ====================
 // * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we search for, in the KF.
 // * best_match_x = x-coordinate of found correspondence in the reference frame.
 float idnew_best_match; // depth in the new image
 float alpha; // d(idnew_best_match) / d(disparity in pixel) == conputed inverse depth derived by the pixel-disparity.
 if(incx*incx>incy*incy)
 {
	  float oldX = fxi*best_match_x+cxi;
	  float nominator = (oldX*referenceFrame->otherToThis_t[2] - referenceFrame->otherToThis_t[0]);
	  float dot0 = KinvP.dot(referenceFrame->otherToThis_R_row0);
	  float dot2 = KinvP.dot(referenceFrame->otherToThis_R_row2);
	  idnew_best_match = (dot0 - oldX*dot2) / nominator;
	  alpha = incx*fxi*(dot0*referenceFrame->otherToThis_t[2] - dot2*referenceFrame->otherToThis_t[0]) / (nominator*nominator);
 }
 else
 {
	  float oldY = fyi*best_match_y+cyi;
	  float nominator = (oldY*referenceFrame->otherToThis_t[2] - referenceFrame->otherToThis_t[1]);
	  float dot1 = KinvP.dot(referenceFrame->otherToThis_R_row1);
	  float dot2 = KinvP.dot(referenceFrame->otherToThis_R_row2);
	  idnew_best_match = (dot1 - oldY*dot2) / nominator;
	  alpha = incy*fyi*(dot1*referenceFrame->otherToThis_t[2] - dot2*referenceFrame->otherToThis_t[1]) / (nominator*nominator);
 }


 if(idnew_best_match < 0)
 {
	  if(enablePrintDebugInfo) stats->num_stereo_negative++;
	  if(!allowNegativeIdepths)
			return -2;
 }
 if(enablePrintDebugInfo) stats->num_stereo_successfull++;
 // ================= calc var (in NEW image) ====================
 // calculate error from photometric noise
 float photoDispError = 4.0f * cameraPixelNoise2 / (gradAlongLine + DIVISION_EPS);
 float trackingErrorFac = 0.25f*(1.0f+referenceFrame->initialTrackedResidual);
 // calculate error from geometric noise (wrong camera pose / calibration)
 Eigen::Vector2f gradsInterp = getInterpolatedElement42(activeKeyFrame->gradients(0), u, v, width);
 float geoDispError = (gradsInterp[0]*epxn + gradsInterp[1]*epyn) + DIVISION_EPS;
 geoDispError = trackingErrorFac*trackingErrorFac*(gradsInterp[0]*gradsInterp[0] + gradsInterp[1]*gradsInterp[1]) / (geoDispError*geoDispError);

 //geoDispError *= (0.5 + 0.5 *result_idepth) * (0.5 + 0.5 *result_idepth);
 // final error consists of a small constant part (discretization error),
 // geometric and photometric error.
 result_var = alpha*alpha*((didSubpixel ? 0.05f : 0.5f)*sampleDist*sampleDist +  geoDispError + photoDispError); // square to make variance
 if(plotStereoImages)
 {
  if(rand()%5==0)
  {
   //if(rand()%500 == 0)
   // printf("geo: %f, photo: %f, alpha: %f\n", sqrt(geoDispError), sqrt(photoDispError), alpha, sqrt(result_var));

   //int idDiff = (keyFrame->pyramidID - referenceFrame->id);
   //cv::Scalar color = cv::Scalar(0,0, 2*idDiff);// bw
   //cv::Scalar color = cv::Scalar(sqrt(result_var)*2000, 255-sqrt(result_var)*2000, 0);// bw
//   float eplLengthF = (std::min)((float)MIN_EPL_LENGTH_CROP,(float)eplLength);
//   eplLengthF = (std::max)((float)MAX_EPL_LENGTH_CROP,(float)eplLengthF);
//
//   float pixelDistFound = sqrtf((float)((pReal[0]/pReal[2] - best_match_x)*(pReal[0]/pReal[2] - best_match_x)
//     + (pReal[1]/pReal[2] - best_match_y)*(pReal[1]/pReal[2] - best_match_y)));
//
   float fac = best_match_err / ((float)MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20);//wx-这个值似乎表示了匹配结果的可信度，如果可信度接近于零则画出的极线为蓝色，如果可信度正常则为绿色。
   cv::Scalar color = cv::Scalar(255*fac, 255-255*fac, 0);// bw

   /*
   if(rescaleFactor > 1)
    color = cv::Scalar(500*(rescaleFactor-1),0,0);
   else
    color = cv::Scalar(0,500*(1-rescaleFactor),500*(1-rescaleFactor));
   */
   cv::line(debugImageStereoLines,cv::Point2f(pClose[0], pClose[1]),cv::Point2f(pFar[0], pFar[1]),color,1,8,0);
  }
 }
 result_idepth = idnew_best_match;
 result_eplLength = eplLength;
 return best_match_err;
}

inline float DepthMap::setDepth(
 const float u, const float v,
 Frame* referenceFrame, const float* referenceFrameImage,
 float &result_idepth, float &result_var, float &result_eplLength,
 RunningStats* const stats)
{
 Eigen::Matrix3f rotMat = referenceFrame->thisToOther_R;
 Eigen::Vector3f transVec = referenceFrame->thisToOther_t;
 int idx = u*width + v;
 if (idx<0 || idx>width*height)
  return -2;
 Eigen::Vector3f refPoint = (1.0f / referenceFrame->idepth(0)[idx]) * Eigen::Vector3f((u - referenceFrame->cx(0)) / referenceFrame->fx(0), (v - referenceFrame->cy(0)) / referenceFrame->fy(0), 1);
 /*Eigen::Matrix4f trans;
 trans <<
  rotMat(0, 0), rotMat(0, 1), rotMat(0, 2), transVec(0),
  rotMat(1, 0), rotMat(1, 1), rotMat(1, 2), transVec(1),
  rotMat(2, 0), rotMat(2, 1), rotMat(2, 2), transVec(2),
  0, 0, 0, 1;
 trans = trans.inverse().eval();
 Eigen::Matrix3f rotMatI;
 rotMatI << trans(0, 0), trans(0, 1), trans(0, 2),
  trans(1, 0), trans(1, 1), trans(1, 2),
  trans(2, 0), trans(2, 1), trans(2, 2);
 Eigen::Vector3f transVecI;
 transVecI << trans(0, 3), trans(1, 3), trans(2, 3);*/
 
 
 Eigen::Vector3f Wxp = rotMat*refPoint + transVec;
 float u_new = (Wxp[0] / Wxp[2])*fx + cx;
 float v_new = (Wxp[1] / Wxp[2])*fy + cy;
 // step 1a: coordinates have to be in image:
 // (inverse test to exclude NANs)
 if (!(u_new > 1 && v_new > 1 && u_new < width - 2 && v_new < height - 2))
 {
  return -2;
 }
 
 float idnew_best_match = getInterpolatedElement(referenceFrame->idepth(0), u_new, v_new, width);
 float val_cp_m2 = getInterpolatedElement(referenceFrameImage, u_new, v_new, width);
 float realVal_p1 = getInterpolatedElement(activeKeyFrameImageData, u, v, width);
 result_var = VAR_GT_INIT_INITIAL;
 result_idepth = idnew_best_match;
 result_eplLength = 0;
 return 0;
}

bool DepthMap::observeDepthCreateRGBD(const int idInKef, const float &x, const float &y, const float &z, const int idInRef, Frame* refFrame, RunningStats* const &stats)
{
 int idx = idInKef;
 int error = 0;
 float result_var = VAR_GT_INIT_INITIAL;
 float result_idepth = 1.0 / z;
 if (idx < 0 || idx>width*height)
  error -2;
 DepthMapPixelHypothesis* target = currentDepthMap + idx;

 if (error == -3 || error == -2)
 {
  target->blacklisted--;
  if (enablePrintDebugInfo) stats->num_observe_blacklisted++;
 }
 if (error < 0 || result_var > MAX_VAR)
  return false;
 if (error >= 0)
 {
  *target = DepthMapPixelHypothesis(
   result_idepth,
   result_var,
   VALIDITY_COUNTER_INITIAL_OBSERVE);
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // white for GOT CREATED
  if (enablePrintDebugInfo) stats->num_observe_created++;
  return true;
 }
}
bool DepthMap::observeDepthUpdateRGBD(const int idInKef, const float &x, const float &y, const float &z, const int idInRef, Frame* refFrame, const float* keyFrameMaxGradBuf, RunningStats* const &stats)
{

 int idx = idInKef;
 int error = 0;
 float result_var = VAR_GT_INIT_INITIAL;
 float result_idepth = 1.0/z;
 if (idx < 0 || idx>width*height)
  error = -1;
 DepthMapPixelHypothesis* target = currentDepthMap + idx;
 

 // which exact point to track, and where from.
 float sv = sqrt(target->idepth_var_smoothed);
 float min_idepth = target->idepth_smoothed - sv*STEREO_EPL_VAR_FAC;
 float max_idepth = target->idepth_smoothed + sv*STEREO_EPL_VAR_FAC;
 if (min_idepth < 0) min_idepth = 0;
 if (max_idepth > 1 / MIN_DEPTH) max_idepth = 1 / MIN_DEPTH;
 stats->num_observe_update_attempted++;

 float diff = result_idepth - target->idepth_smoothed;

 // if oob: (really out of bounds)
 if (error == -1)
 {
  // do nothing, pixel got oob, but is still in bounds in original. I will want to try again.
  if (enablePrintDebugInfo) stats->num_observe_skip_oob++;
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // RED FOR OOB
  return false;
 }
 // if just not good for stereo (e.g. some inf / nan occured; has inconsistent minimum; ..)
 else if (error == -2)
 {
  if (enablePrintDebugInfo) stats->num_observe_skip_fail++;
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 255); // PURPLE FOR NON-GOOD

  target->validity_counter -= VALIDITY_COUNTER_DEC;
  if (target->validity_counter < 0) target->validity_counter = 0;

  target->nextStereoFrameMinID = 0;
  target->idepth_var *= FAIL_VAR_INC_FAC;
  if (target->idepth_var > MAX_VAR)
  {
   target->isValid = false;
   target->blacklisted--;
  }
  return false;
 }
 // if not found (error too high)
 else if (error == -3)
 {
  if (enablePrintDebugInfo) stats->num_observe_notfound++;
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // BLACK FOR big not-found

  return false;
 }
 else if (error == -4)
 {
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // BLACK FOR big arithmetic error
  return false;
 }
 // if inconsistent
 else if (DIFF_FAC_OBSERVE*diff*diff > 1e3*(result_var + target->idepth_var_smoothed))
 {
  if (enablePrintDebugInfo) stats->num_observe_inconsistent++;
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 0); // Turkoise FOR big inconsistent
  target->idepth_var *= FAIL_VAR_INC_FAC;
  if (target->idepth_var > MAX_VAR) target->isValid = false;
  return false;
 }

 else
 {
  // one more successful observation!
  if (enablePrintDebugInfo) stats->num_observe_good++;
  if (enablePrintDebugInfo) stats->num_observe_updated++;

  // do textbook ekf update:
  // increase var by a little (prediction-uncertainty)
  float id_var = target->idepth_var*SUCC_VAR_INC_FAC;
  // update var with observation
  float w = result_var / (result_var + id_var);
  float new_idepth = (1 - w)*result_idepth + w*target->idepth;
  target->idepth = UNZERO(new_idepth);
  //guo-这里更新深度
  // variance can only decrease from observation; never increase.
  id_var = id_var * w;
  if (id_var < target->idepth_var)
   target->idepth_var = id_var;
  //guo-这里更新方差
  // increase validity!
  target->validity_counter += VALIDITY_COUNTER_INC;
  float absGrad = keyFrameMaxGradBuf[idx];
  if (target->validity_counter > VALIDITY_COUNTER_MAX + absGrad*(VALIDITY_COUNTER_MAX_VARIABLE) / 255.0f)
   target->validity_counter = VALIDITY_COUNTER_MAX + absGrad*(VALIDITY_COUNTER_MAX_VARIABLE) / 255.0f;
  // increase Skip!
  /*if (result_eplLength < MIN_EPL_LENGTH_CROP)
  {
   float inc = activeKeyFrame->numFramesTrackedOnThis / (float)(activeKeyFrame->numMappedOnThis + 5);
   if (inc < 3) inc = 3;
   inc += ((int)(result_eplLength * 10000) % 2);
   if (enablePrintDebugInfo) stats->num_observe_addSkip++;
   if (result_eplLength < 0.5*MIN_EPL_LENGTH_CROP)
    inc *= 3;

   target->nextStereoFrameMinID = refFrame->id() + inc;
  }*/
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255); // yellow for GOT UPDATED
  return true;
 }
}


void DepthMap::observeDepthRowRGBD(int yMin, int yMax, RunningStats* stats)
{
 const float* keyFrameMaxGradBuf = activeKeyFrame->maxGradients(0);
 int successes = 0;
 Frame* refFrame = activeKeyFrameIsReactivated ? newest_referenceFrame : oldest_referenceFrame;
 bool* wasGoodDuringTracking = 0;
 if (refFrame->getTrackingParent() == activeKeyFrame)
 {
  wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();
 }
 
 for (int y = yMin; y<yMax; y++)
  for (int x = 3; x<width - 3; x++)
  {
   int idx = y*width + x;
   if (idx<0 || idx>width*height)
    continue;
   if (wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])
   {
    continue;
   }
  
   if (refFrame->idepth(0)[idx] < 0)
    continue;

   Eigen::Matrix3f rotMat = refFrame->thisToOther_R;
   Eigen::Vector3f transVec = refFrame->thisToOther_t;
   
   Eigen::Vector3f refPoint = (1.0f / refFrame->idepth(0)[idx]) * Eigen::Vector3f((x - refFrame->cx(0)) / refFrame->fx(0), (y - refFrame->cy(0)) / refFrame->fy(0), 1);
   Eigen::Vector3f Wxp = rotMat*refPoint + transVec;
   int x_new = (Wxp[0] / Wxp[2])*fx + cx;
   int y_new = (Wxp[1] / Wxp[2])*fy + cy;
   float z_new = Wxp[2];
   // step 1a: coordinates have to be in image:
   // (inverse test to exclude NANs)
   if (!(x_new > 1 && y_new > 1 && x_new < width - 2 && y_new < height - 2))
   {
    continue;
   }
   
   int idInKeyframe = y_new*width + x_new;
   if (idInKeyframe<0 || idInKeyframe>width*height)
    continue;
   DepthMapPixelHypothesis* target = currentDepthMap + idInKeyframe;
   bool hasHypothesis = target->isValid;
   if (keyFrameMaxGradBuf[idInKeyframe]<MIN_ABS_GRAD_CREATE)
   {
    continue;
   }
   bool success = true;
   if (!hasHypothesis)
    success = observeDepthCreateRGBD(idInKeyframe,x_new, y_new, z_new, idx, refFrame, stats);
   else
    success = observeDepthUpdateRGBD(idInKeyframe,x_new, y_new, z_new, idx, refFrame, keyFrameMaxGradBuf, stats);

   if (success)
    successes++;
  }

 
}

}