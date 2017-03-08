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


   if(hasHypothesis && keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_DECREASE)//wx-temp ��������ǰ�ؼ�֡���ݶȽϴ�λ�õ����ֵ������֮ǰ�ؼ�֡�������ͶӰ����ǰ�ؼ�֡ʱ�ݶȲ����㹻�����ȣ��о����ﲻ̫����
   {
    //target->isValid = false;          //wx-debug ������ܻ�ɾ��֮ǰ��ȷ���ؽ��������һ�㲻�ᷢ������������¼������Ŀ�����track����ʧ�ܻ����ڵ�������
    continue;
   }
   if(keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_CREATE || target->blacklisted < MIN_BLACKLIST)//wx- �ݶ�������㹻�����ؽ��õ㣬����ĵ��Ѿ�����ʾΪ�����ؽ��ĵ㣬������ؽ�
    continue;

   bool success;
   if(!hasHypothesis)
    //guo-������ȵ㵫������һ����������һ����ȵ�
    success = observeDepthCreate(x, y, idx, stats);//wx- �����֮ǰû���ؽ�����һ��(Ҳ�����Ǳ�ɾ����)�������õ�ǰ���������ϵ���һ֡��Ϊ�ο�֡(���뵱ǰ�ؼ�֡��ʱ����������һ֡)�����ؽ�
   else
    success = observeDepthUpdate(x, y, idx, keyFrameMaxGradBuf, stats);//wx- ����Ѿ��������Ϣ������Ҫ����
   if(success)
    successes++;
  }

}
void DepthMap::observeDepth()
{
#ifdef RGBDSLAM
 threadReducer.reduce(boost::bind(&DepthMap::observeDepthRowRGBD, this, _1, _2, _3), 3, height - 3, 480);//wx-�ֶ����л��ļ����ÿ�м�����ȣ�ͬʱ�ĸ������ڼ��㣬ÿ�����̼���10�е���ȣ�����������¸�10��
#else
 threadReducer.reduce(boost::bind(&DepthMap::observeDepthRow, this, _1, _2, _3), 3, height - 3, 480);//wx-�ֶ����л��ļ����ÿ�м�����ȣ�ͬʱ�ĸ������ڼ��㣬ÿ�����̼���10�е���ȣ�����������¸�10��
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
 float epx = - fx * ref->thisToOther_t[0] + ref->thisToOther_t[2]*(x - cx);//wx-temp ��ο�ͼ��������������ͼ�ϵ�ͶӰ�ǵ�OC = (fx * ref->thisToOther_t[0]+cx��fy * ref->thisToOther_t[1]+cy��ref->thisToOther_t[2])�� �о��������epx��epyӦ���������ͼ�ϴ�OC��(x,y)��������������OCû������Ϊ1�����Ե�ǰ�Ľ����ʵ��  ref->thisToOther_t[2] * ((x,y) - OC)
 float epy = - fy * ref->thisToOther_t[1] + ref->thisToOther_t[2]*(y - cy);//wx ���ڷ��ص�pepx��pepy�ǵ�λ����ļ��߳��ȣ����������ı�����������
 //guo-��Ϊ�����λ���µ�λ��
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
 //guo-������age����?
 Frame* refFrame = activeKeyFrameIsReactivated ? newest_referenceFrame : oldest_referenceFrame;//wx- activeKeyFrameIsReactivated��������Ƿ����LoopClosure��retrack�����������ǰ�ؼ�֡���滻Ϊ���ϵ�֮ǰһ֡��������ʱ�����µ�֡�Ļ����ܱ�֤loop֡�͵�ǰ�ο�֡�����Ƴ̶ȱ�֤�ؽ�����������������ڵĻ�һ�����ӽǲ���ǵ��ؽ�������ã�
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
 //guo-epl��ʲô?//epipolar line
 bool isGood = makeAndCheckEPL(x, y, refFrame, &epx, &epy, stats);//wx- ���������ͼ�ϵļ���
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

 if(!activeKeyFrameIsReactivated)//�������loopClosure����reTrack�����
 {
  if((int)target->nextStereoFrameMinID - referenceFrameByID_offset >= (int)referenceFrameByID.size())//wx- nextStereoFrameMinID�ǵ��õ㱻�ɹ�����һ��֮�󣬰����Ǵμ���ʱ�ļ��߳��ȼ��������һ�����ٸ��µ�id��Χ����ֱ��nextStereoFrameMinID��������ʵ������offset���ж����nextStereoFrameMinID�Ƿ��Ѿ�������У����Կ�ʼ��һ�θ�����
  {
   if(plotStereoImages)
    debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,255,0); // GREEN FOR skip//wx- ����ƥ�����ͼ�е���ɫ�������������֮ǰ�ɹ�������һ�Σ��ȴ�nextStereoFrameMinID
   if(enablePrintDebugInfo) stats->num_observe_skip_alreadyGood++;
   return false;
  }
                   //wx- nextStereoFrameMinID��ʼΪ0���ڳɹ�ƥ�����Ϊδ���Ż�������һ��֡ID������ĳ֡ƥ��ʱ������ֲ��ʺ�����ƥ��������������ᱻ������Ϊ0
  if((int)target->nextStereoFrameMinID - referenceFrameByID_offset < 0)//��wx- �����ֻ����nextStereoFrameMinID��0��ʱ��Żᷢ��������û�и��¹���֮ǰ����µ�ֵƥ��ʱ�����˲��ʺ�����ƥ������
   refFrame = oldest_referenceFrame;
  else
   refFrame = referenceFrameByID[(int)target->nextStereoFrameMinID - referenceFrameByID_offset];//wx- �ο�֡����Ϊ���ϴθ��³ɹ�ʱԤ����nextStereoFrameMinID��һ֡�������θ���ʧ�ܣ��´λ�ʹ�õ�ǰ���е�����֡����ƥ�䣬���ǲ��Ǿ�����������ν��age���ӣ��Ҿ������Զ�
 }
 else
  refFrame = newest_referenceFrame;//�����loopClosure����retrack��ֱ��ʹ�����µ�֡����ƥ��

 if(refFrame->getTrackingParent() == activeKeyFrame)
 {
  bool* wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();//wx- �ж��Ƿ���һ������״̬����ĵ㣬������Ǻ���ĵ㣬˵����ǰ�����������ģ�ͶԸõ��ǲ��ʺ��ؽ��ģ��ʷ����ؽ�
  if(wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])//wx- ��������ʲô��ֻ���ڳ���2��
  {
   if(plotStereoImages)
    debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,0); // BLUE for SKIPPED NOT GOOD TRACKED
   //target->isValid = false;//wx-debug���������Ҽ���ȥ��
   return false;//wx-debug ����ԭ�����е�
  }
 }
 
 float epx, epy;
 bool isGood = makeAndCheckEPL(x, y, refFrame, &epx, &epy, stats);
 if(!isGood) return false;
 // which exact point to track, and where from.
 float sv = sqrt(target->idepth_var_smoothed);
 float min_idepth = target->idepth_smoothed - sv*STEREO_EPL_VAR_FAC;//wx- ���Ѿ�����ȵ����������£��������ܵ������Ϊ��ǰ��ȼ���2������
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
  diff = result_idepth - target->idepth_smoothed;//wx- �¼������Ⱥ�֮ǰ���֮��Ĳ�ֵ

 // if oob: (really out of bounds)//wx- ������ͼ�ϵļ��߻��߲ο�֡�ϵļ��߳�����ͼƬ�����Ե̫���ͷ���-1
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
   //debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,255);//wx-debug // PURPLE FOR NON-GOOD //wx- �����ø����ص�ƥ�����̫����õ�ƥ��ʹκõĲ�ֵ��С�ȵ�ƥ�������ؽ�������ܲ��ã�����semi-dense ����������Щ��

  target->validity_counter -= VALIDITY_COUNTER_DEC;
  if(target->validity_counter < 0) target->validity_counter = 0;

  target->nextStereoFrameMinID = 0;
  target->idepth_var *= FAIL_VAR_INC_FAC;
  if(target->idepth_var > MAX_VAR)
  {
   //target->isValid = false;              //wx-debug ������ܻ�ɾ��֮ǰ��ȷ���ؽ���������ڵ�ǰͼƬ�ϸ����ͼͶӰ����ǰ֮֡���Ӧ�����ص㣬����õķ��������˵���ʺ����ؽ��������ڵ������ٵľ��ȣ���Ե(�ؽ���������ݶȴ�ֵ)�ȣ��ܿ���֮ǰ�������ȷ�ģ�ͶӰ����ǰ֮֡���Ӧ������ߵ�ǰ֡������б仯��������ɾ��֮ǰ�ĵ㡣
   //target->blacklisted--;               //wx-�������ǳ�������ܶ�ĵ������Ϊ������ⱻɾ������������ط����漰����ǰ���ͼ��semi-dense��׼�����⣬��������ȷʵ����㲻���н��
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
 else if(DIFF_FAC_OBSERVE*diff*diff > result_var + target->idepth_var_smoothed)//wx- ���֮ǰ����Ⱥͱ��μ��������������̫���򽫸õ�����ɾ��������������ڸ��ٴ����ˣ������ؽ�ƥ������˵ȵ�������¡�
 {
  if(enablePrintDebugInfo) stats->num_observe_inconsistent++;
  if(plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,0); // Turkoise FOR big inconsistent
  target->idepth_var *= FAIL_VAR_INC_FAC;
  if(target->idepth_var > MAX_VAR) target->isValid = false;      //wx-debug �������ɾ��֮ǰ��ȷ���ؽ�������ܶ�ط���������������ⱻɾ�����滻�����Ǻ��п���֮ǰ���ؽ��������ȷ�ģ����߱��ε��ؽӽ������ȷ�ģ�����ȴ��ɾ���ˡ�����������Ǿ���������ǰ��֡�����ؽ�����ܺã�������ɾ���˵���Ҫ��Դ\
                         ��һ����ԴӦ���ڸ������ʱ��ǰ֡�ķ���̫����Ϊ��֡���ؽ�����ܲ��ɿ������Խ���ǰ֡���ؽ������֮ͬǰ�Ľ����ɾ�����������Ŀ��ǿ����ǿ��ǵ������ܻ������⣬Ҫ��ʱ��Ľ����ؽ���ʱ��ֱ����֮���µ������滻֮ǰ�ľ����ݣ�������δ���õ������ݵĽ��ʹ������ÿ���
  return false;
 }
 else if (target->isValid == false)//wx-debug
 {
  *target = DepthMapPixelHypothesis(
   result_idepth,
   result_var,
   VALIDITY_COUNTER_INITIAL_OBSERVE);
  if (plotStereoImages)
   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 255); // Orange for tracking�������Ȼ�����½�����ȵ�
 }
 else
 {
  // one more successful observation!
  if(enablePrintDebugInfo) stats->num_observe_good++;
  if(enablePrintDebugInfo) stats->num_observe_updated++;

  // do textbook ekf update:
  // increase var by a little (prediction-uncertainty)
  float id_var = target->idepth_var*SUCC_VAR_INC_FAC;    //wx- ����Ͱ���EKF�ķ�ʽ��������Ⱥͷ���
  // update var with observation
  float w = result_var / (result_var + id_var);
  float new_idepth = (1-w)*result_idepth + w*target->idepth;
  target->idepth = UNZERO(new_idepth);
  //guo-����������
  // variance can only decrease from observation; never increase.
  id_var = id_var * w;
  if(id_var < target->idepth_var)
   target->idepth_var = id_var;
  //guo-������·���
  // increase validity!
  target->validity_counter += VALIDITY_COUNTER_INC;
  float absGrad = keyFrameMaxGradBuf[idx];
  if(target->validity_counter > VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f)
   target->validity_counter = VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f;
  // increase Skip!
  if(result_eplLength < MIN_EPL_LENGTH_CROP)           //wx- ���ﰴ��ĳ�����صķ�ʽ���ݲο�֡�ϼ��ߵĳ����趨��һ�����ٸ�����ȵ�ʱ�䴰����ֱ���½�����֡��ID��nextStereoFrameMinIDΪֹ�����ٸ����������
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
void DepthMap::propagateDepth(Frame* new_keyframe)//wx-Ϊ���ҵ��ɹؼ�֡�и����ص������Ϣ���¹ؼ�֡������֮��Ķ�Ӧ��ϵ���Ƚ��ɹؼ�֡�е����ص㷴ͶӰ����ά�ռ䣬�ٴ���ά�ռ�ͶӰ���¹ؼ�֡�������ȵĴ���
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
 for(DepthMapPixelHypothesis* pt = otherDepthMap+width*height-1; pt >= otherDepthMap; pt--)//wx-���otherDepthMap��֮����otherDepthMapΪ��ת����������Ϣ���ϵ�otherDepthMap�У�Ȼ�󽻻�otherDepthMap��currentDepthMap
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

   Eigen::Vector3f pn = (trafoInv_R * Eigen::Vector3f(x*fxi + cxi,y*fyi + cyi,1.0f)) / source->idepth_smoothed + trafoInv_t;//wx-�Ƚ����ص�ͶӰ����ά�ռ�
   float new_idepth = 1.0f / pn[2];
   float u_new = pn[0]*new_idepth*fx + cx;//wx-����ά�ռ佫��ͶӰ���µĹؼ�֡
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
   if(trackingWasGood != 0)//wx-���tracking�Ľ�����ã������������ص���ɫ���ݳ̶ȣ��������򲻴��ݣ�
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
   if(targetBest->isValid)//wx-���ڿ��ܳ�����ά�ռ��ж����ͶӰ���µĹؼ�֡�ж�Ӧ��ͬһ�����ص���������Լ���Ƿ���������Ѿ���ͶӰ���˼��Ƿ����occlusion
                        //����ڵ������ж��Ƿ񳬳�������ķ�Χ�������������Ϊ����㲻��Ӧ��ά�ռ�ͬһ���㣬������������ж���������˭���������������µ����������������Ϊ�µ��ڵ��ɵ�
         //����µ��ڵ��ɵ��򽫾ɵ��״̬��Ϊ�գ��ȴ�֮���µ㸳ֵ�������صĹ���
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
    if(enablePrintDebugInfo) runningStats.num_prop_created++;//wx-�µ��ڵ��ɵ㣬���µ��滻�ɵ��ֵ
    *targetBest = DepthMapPixelHypothesis(
      new_idepth,
      new_var,
      source->validity_counter);
   }
   else
   {
    if(enablePrintDebugInfo) runningStats.num_prop_merged++;//wx-�µ�;ɵ�ľ����ٷ��Χ�ڣ�����ָ����ά�ռ�ͬһ���㣬�����ں�(ƽ��)
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
 std::swap(currentDepthMap, otherDepthMap);//wx-�������´����˵������Ϣ�ŵ���otherDepthMap�Ȼ�󽻻��˵�ǰ�����ַ���൱�ڸ�ֵ����currentDepthMap
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
    //guo-������֮ǰ�������ȵ�һ����ֵ����ʼ����һ֡���������
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
 //guo-�����ʼ���ݶ�
 const float* maxGradients = new_frame->maxGradients();
 const unsigned char* edge = new_frame->edgeFlag(0);
 for(int y=1;y<height-1;y++)
 {
  for(int x=1;x<width-1;x++)
  {
   //guo-�����ʼ��һ��������,����ʼ��feature
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

void DepthMap::setFromExistingKF(Frame* kf)//wx- �����ҵ�loopClosure���߽���reTrackʱ����
{
 assert(kf->hasIDepthBeenSet());
 activeKeyFramelock = kf->getActiveLock();
 activeKeyFrame = kf;
 const float* idepth = activeKeyFrame->idepth_reAct();//wx- ��Щ��������loopClosure֡�ж�Ӧ����Ⱥͷ���
 const float* idepthVar = activeKeyFrame->idepthVar_reAct();
 const unsigned char* validity = activeKeyFrame->validity_reAct();
 DepthMapPixelHypothesis* pt = currentDepthMap;//wx- pt�ǵ�ǰ���ͼ�е���Ⱥͷ���
 activeKeyFrame->numMappedOnThis = 0;
 activeKeyFrame->numFramesTrackedOnThis = 0;
 activeKeyFrameImageData = activeKeyFrame->image(0);
 activeKeyFrameIsReactivated = true;
 //guo-��������·���Ⱥͷ���
 for(int y=0;y<height;y++)//wx- �������ҵ�loopClosure(��reTrack)֮����֮ǰ�ؼ�֡�����ȫ���滻�˵�ǰ֡����ȡ��о�������LoopClosure��̫����
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

void DepthMap::updateKeyframe(std::deque< std::shared_ptr<Frame> > referenceFrames)//wx- ��mapping�߳��б����ã�����unmappedTrackedFrames��ΪreferenceFrames�������ͼ�е����
{
 assert(isValid());


#ifdef TIME_CAL
 struct timeval tv_start_all, tv_end_all;
 gettimeofday(&tv_start_all, NULL);
#endif
 oldest_referenceFrame = referenceFrames.front().get();
 newest_referenceFrame = referenceFrames.back().get();
 referenceFrameByID.clear();
 referenceFrameByID_offset = oldest_referenceFrame->id();//wx- ����referenceFrameByID�Ǿֲ���֡���У�Ϊ���ܹ�����Id���ʣ���֡����С��֡��Ϊoffset������ByID�ķ��ʷ�ʽ
// for(std::shared_ptr<Frame> frame : referenceFrames)
 for (std::deque< std::shared_ptr<Frame> >::iterator c = referenceFrames.begin() ; c != referenceFrames.end() ; c++)//wx- �Բο������е�ÿ֡����������ƥ��
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
  //guo-����֡��ؼ�֡������˶�,ֻ�ǹؼ�֡
  frame->prepareForStereoWith(activeKeyFrame, refToKf, K, 0);
  while((int)referenceFrameByID.size() + referenceFrameByID_offset <= frame->id())//wx- ����referenceFrameByID_offset���찴��  [id-referenceFrameByID_offset] �ķ�����ʽ��������֡����˳����ӵ�referenceFrameByID��
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
 //guo-����Ҳ�������
 observeDepth();
#ifdef TIME_CAL
 gettimeofday(&tv_end, NULL);
 msObserve = 0.9*msObserve + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
 nObserve++;
 //if(rand()%10==0)
 
  gettimeofday(&tv_start, NULL);
#endif
  //guo-������ݸ��º����ȼ���һ��ƽ�����
  regularizeDepthMapFillHoles();
#ifdef TIME_CAL
  gettimeofday(&tv_end, NULL);
  msFillHoles = 0.9*msFillHoles + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
  nFillHoles++;
 

 gettimeofday(&tv_start, NULL);
#endif
 //guo-�о���������ƽ��
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


int DepthMap::debugPlotDepthMap()//wx-bug ��ǰ�������������������bug
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
   cv::Vec3b color = currentDepthMap[idx].getVisualizationColor(refID);//wx-bug ����ᱼ�����۲��refIDһֱû����ȷ��ֵ����֪���ǲ������ԭ��
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
 Eigen::Vector3f KinvP = Eigen::Vector3f(fxi*u+cxi,fyi*v+cyi,1.0f);//wx- ������u��v�Ǳ������ͼʱ���ͼ�ϵ����꣬�����ͼ��(u,v)�������Ϊ1ʱ����ά�ռ��еĵ�//guo-������
 Eigen::Vector3f pInf = referenceFrame->K_otherToThis_R * KinvP;//wx- KinvP�����ΪdʱͶӰ���ο�ͼ�е�����ʽ��Ӧ����  d  *  referenceFrame->K_otherToThis_R * KinvP  +  referenceFrame->K_otherToThis_t�����ǵ����d��������ʱ������ĸ�������ԣ�ת���������ʱ����ԶֵdҲ�ᱻ���������Ǵ�ʱ��Pinf����������1
 Eigen::Vector3f pReal = pInf / prior_idepth + referenceFrame->K_otherToThis_t;//guo-��ʵ��λ��//wx- ���֪�������(x,y,pri)���Ӧ�ĵ�ͶӰ��λ�ã���������1
             //wx-��ʱ��pReal[2]�������Ϊ1����ʵ��Ӧ����ͶӰ���ο�ͼ�ϵ�����֮�󣬸����ض�Ӧ�����ֵ���ο�K.inv * pReal���᷵�ص���ά�ռ��е��Ǹ�ͶӰ��
 float rescaleFactor = pReal[2] * prior_idepth;//wx- pri_depth��Ӧ����ͬһ����ά�������ͼ�е���ȣ�pReal[2]��Ӧ����ͶӰ��ͶӰ���ο�ͼ֮������
     
            
 float firstX = u - 2*epxn*rescaleFactor;//wx- rescaleFactor����������˵Ӧ��������ͼƬ�϶�Ӧ���ߵĳ��ȱȣ���������Զ�㵽��������ߵĳ��ȱȣ�����û̫������д��
 float firstY = v - 2*epyn*rescaleFactor;//guo-����߶Ƚ�������˵�ǵ����ֱ���,��װ����iccv13������[5],���ƥ���baseline*�ֱ��������Թ�ϵ��,�����ֱ��ʿ���ʹ��ȵ�������ޱ���һ������
 float lastX = u + 2*epxn*rescaleFactor;
 float lastY = v + 2*epyn*rescaleFactor;
 // width - 2 and height - 2 comes from the one-sided gradient calculation at the bottom
 if (firstX <= 0 || firstX >= width - 2//wx ���ڼ��߿�����ͼƬ��Ե��������һ�������ʱ�Ϳ��ܳ���ͼƬ��Χ�������ȼ�����λ�õ��Ҷ�Ӧ�ĺ�����
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
 // calculate values to search for//wx-temp �ӵ�ǰ���ͼ��Ӧ��ͼƬ�����ż��ߣ�ȡ��ǰ��ǰ���ĸ���//guo-�������Ӧ�����н������߶����ŵ�
 float realVal_p1 = getInterpolatedElement(activeKeyFrameImageData,u + epxn*rescaleFactor, v + epyn*rescaleFactor, width);
 float realVal_m1 = getInterpolatedElement(activeKeyFrameImageData,u - epxn*rescaleFactor, v - epyn*rescaleFactor, width);
 float realVal = getInterpolatedElement(activeKeyFrameImageData,u, v, width);
 float realVal_m2 = getInterpolatedElement(activeKeyFrameImageData,u - 2*epxn*rescaleFactor, v - 2*epyn*rescaleFactor, width);
 float realVal_p2 = getInterpolatedElement(activeKeyFrameImageData,u + 2*epxn*rescaleFactor, v + 2*epyn*rescaleFactor, width);

// if(referenceFrame->K_otherToThis_t[2] * max_idepth + pInf[2] < 0.01)

 Eigen::Vector3f pClose = pInf + referenceFrame->K_otherToThis_t*max_idepth;//ʵ�ʶ�Ӧ���ο�ͼ�ϵ�ȡ�������ֵʱ�ĵ㣬������Ϊ�ǲο�ͼ�ϼ��ߵ��յ�,֮�������λ�
 // if the assumed close-point lies behind the
 // image, have to change that.
 if(pClose[2] < 0.001f)
 {
	  max_idepth = (0.001f-pInf[2]) / referenceFrame->K_otherToThis_t[2];
	  pClose = pInf + referenceFrame->K_otherToThis_t*max_idepth;
 }
 pClose = pClose / pClose[2]; // pos in new image of point (xy), assuming max_idepth
 Eigen::Vector3f pFar = pInf + referenceFrame->K_otherToThis_t*min_idepth;//ʵ�ʶ�Ӧ���ο�ͼ�ϵ�ȡ�����Сֵʱ�ĵ㣬������Ϊ�ǲο�ͼ�ϵļ��ߵĿ�ʼ��֮�������λ�
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
 if(eplLength > MAX_EPL_LENGTH_CROP)//�������̫����˳�ż���ȡ���
 {
	  pClose[0] = pFar[0] + incx*MAX_EPL_LENGTH_CROP/eplLength;
	  pClose[1] = pFar[1] + incy*MAX_EPL_LENGTH_CROP/eplLength;
 }
 incx *= GRADIENT_SAMPLE_DIST/eplLength;//��incxת���������Ĳ���
 incy *= GRADIENT_SAMPLE_DIST/eplLength;

 // extend one sample_dist to left & right.
 pFar[0] -= incx;//wx-temp ˳�ż�����ǰ������ӳ�һ������
 pFar[1] -= incy;
 pClose[0] += incx;
 pClose[1] += incy;

 // make epl long enough (pad a little bit).//wx-temp �ոնԼ��ߵ��ӳ�Ҳ�п���������������ȡ���ڷ��ţ��ٴ��жϾ����Ƿ񹻳�
            //guo-��������ӳ�,pFar�Ĵ�С��incx�ķ�����ͬ���任��
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

 // if near point is outside: move inside, and test length again.//wx-temp �ο�ͼ�ϼ��ߵ��յ����������Χ��������ͼƬ��Χ��
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

 float cpx = pFar[0];//�Ӽ��ߵĿ�ʼ�˿�ʼ����
 float cpy =  pFar[1];
 float val_cp_m2 = getInterpolatedElement(referenceFrameImage,cpx-2.0f*incx, cpy-2.0f*incy, width);//ͨ����ֵ������ڵ�ǰ���ͼ�ϲɼ����������ܶ�Ӧ�ĵ㣬���ǲ�֪��Ϊʲô�����������ʹ��Щ��֮��ļ����Ӧ
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
   float fac = best_match_err / ((float)MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20);//wx-���ֵ�ƺ���ʾ��ƥ�����Ŀ��Ŷȣ�������ŶȽӽ������򻭳��ļ���Ϊ��ɫ��������Ŷ�������Ϊ��ɫ��
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
  //guo-����������
  // variance can only decrease from observation; never increase.
  id_var = id_var * w;
  if (id_var < target->idepth_var)
   target->idepth_var = id_var;
  //guo-������·���
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