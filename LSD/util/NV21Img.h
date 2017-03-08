#ifndef NV21IMG_H__
#define NV21IMG_H__
#include "preprocessHeaders.h"
#include "string"
//this function for test NV21Img class
void readAndWriteAndDisplay_test(std::string imgName);
/**
*
*NV21 is the image format used in android, and it has YUV channels
*
*/
class NV21Img
{
public:
 /**
 *
 *constructor use input parameters
 *
 *@param [in] h : image height
 *@param [in] w : image width
 *
 */
 NV21Img(int h,int w)
 {
  height = h; 
  width = w;
  Ycn = NULL; 
  UVcn = NULL;
  bCreated=false;
 }
 /**
 *
 *deconstructor release resources
 *
 */
 ~NV21Img()
 {
  freeData();
  height = width = 0;
 }
public:
 /**
 *
 *copy data to a NV21 image
 *
 *@param [out] img : NV21 image to be modified
 *
 */
 void copyTo(NV21Img &img);
 /**
 *
 *release resources
 *
 */
 void freeData();
 /**
 *
 *initialise the NV21 image
 *
 *@param [in] height : image height
 *@param [in] width  : image width
 *
 */
 void nvCreate(int height , int width);
 /**
 *
 *initialise the NV21 image with data loaded from a file
 *
 *@param [in] filename  : file path
 *@param [in] height   : image height
 *@param [in] width   : image width
 *@param [out] return  : 1 for sucess, -1 for fail
 *
 */
 int nvLoadNV21(std::string filename, int height , int width);
 /**
 *
 *initialise the NV21 image with provided data
 *
 *@param [in] buf   : provided data
 *@param [in] height   : image height
 *@param [in] width   : image width
 *@param [out] return  : 1 for success
 *
 */
 int nvLoadNV21(unsigned char *buf, int height, int width);
 /**
 *
 *initialise the NV21 image with a color image
 *
 *@param [in] bgrBuf   : provided color image
 *@param [in] w   : image width
 *@param [in] h   : image height
 *
 */
 void nvLoadImgFromBufferBGR(unsigned char *bgrBuf, int w, int h);
 /**
 *
 *display the NV21 image 
 *
 *@param [in] winName :  window name
 *
 */
 void nvShowNV21(std::string winName = "");
 /**
 *
 *initialise the NV21 image with data loaded from a file
 *
 *@param [in] filename  : file path
 *
 */
 void nvLoadImg(std::string filename);
 /**
 *
 *convert the NV21 image into a color image
 *
 *@param [out] return  : a color image
 *
 */
 cv::Mat nvCvt2BGR();
 /**
 *
 *convert the color image into a NV21 image
 *
 *@param [in] bgr_img  : a color image
 *
 */
 void nvCvt2NV21(cv::Mat bgr_img);
 /**
 *
 *save the NV21 image into a file
 *
 *@param [in] name  : file path
 *
 */
 void nvSaveNV21(std::string name);
 /**
 *
 *save the gray-scale image into a file
 *
 *@param [in] name  : file path
 *
 */
 void nvSaveGrey(std::string name);
 /**
 *
 *save the color image into a file
 *
 *@param [in] name  : file path
 *
 */
 void nvSaveColor(std::string name);
 /**
 *
 *convert a video into a set of NV21 images and save to file
 *
 *@param [in] videoPath : video path
 *@param [in] dstPath : file path to save NV21 images
 *
 */
 static void nvVideo2Frame(std::string videoPath , std::string dstPath);
 /**
 *
 *get n-th frame of a video
 *
 *@param [in] videoPath : video path
 *@param [in] nframe  : frame index
 *@param [in] h   : image height
 *@param [in] w   : image width
 *@param [out] return : -1 for not found videoPath , 0 for not found videoframe at nframe(read finished) , 1 for read success
 *
 */
 int nvGetNV21Frame(std::string videoPath , int nframe , int h = -1 , int w = -1);
 
 /**
 *
 *flip image 
 *
 *@param [in] code  : flip code: 0 Flips around x-axis; >0 Flips around y-axis; <0 Flips around both axes.
 *
 */
 void Flip(int code);
public:
 int width;  ///< image width
 int height;  ///< image height
 int stp;  ///< step
 int stp2;  ///< step2
 uchar *Ycn;  ///< Y channel
 uchar *UVcn; ///< UV channel
 
 cv::Mat Ymat;  ///< Y image
 cv::Mat UVmat;  ///< UV image
 cv::Mat YUVmat;  ///< YUV image
 cv::Mat BGRmat;  ///< BGR image
 IplImage YIpl;  ///< Y image
 IplImage UVIpl;  ///< UV image
 IplImage BGRIpl; ///< BGR image
   
 bool bCreated;  ///< if NV21 image has been created
private:
 float cx; ///< x coordinate of image centre 
 float cy; ///< y coordinate of image centre 
 int   ch; ///< image height
 int   cw; ///< image width
 float Fx; ///< focal length
 float Fy; ///< focal length
};
#endif
