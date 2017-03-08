#include "preprocessHeaders.h"
#include "NV21Img.h"
#include "stdio.h"

#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <sys/stat.h>  // for mkdir()
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>  // for opendir()
#endif // WIN32

#ifdef WIN32
#define TEMPDIR "./"
#else
#define TEMPDIR "/sdcard/"
#endif


void NV21Img::freeData()
{
 if(Ycn != NULL)
 {
  free(Ycn);
  Ycn = NULL;
  UVcn = NULL;
 }
//  if (UVcn != NULL)
//  {
//   free(UVcn);
//   UVcn = NULL;
//  }
 bCreated = false;
}
void NV21Img::nvCreate(int h , int w)
{
 freeData();
 height = h;
 width = w;
 stp = w;
 stp2 = w+w;
 Ycn = (uchar *)malloc(width*height + width*height/2);
 UVcn = Ycn + width*height;
 Ymat = cv::Mat(height,width,CV_8UC1 , Ycn);
 UVmat = cv::Mat(height / 2, width / 2, CV_8UC2, UVcn);
 YUVmat = cv::Mat(height + height / 2, width, CV_8UC1, Ycn);
 YIpl = Ymat;
 UVIpl = UVmat;
 bCreated = true;
}
void NV21Img::copyTo(NV21Img &img)
{
 img.nvCreate(height,width);
 memcpy(img.Ycn,Ycn , width*height);
 memcpy(img.UVcn , UVcn , height*width/2);
 img.cx = cx;
 img.cy = cy;
 img.ch = ch;
 img.cw = cw;
 img.Fx = Fx;
 img.Fy = Fy;
}

int NV21Img::nvLoadNV21(std::string filename, int h , int w)
{
 nvCreate(h,w);
 FILE *file = fopen(filename.c_str(),"rb");
 fread(Ycn , 1 , width*height , file);
 fread(UVcn , 1 , width*height/2 , file);
 fclose(file);
 return 1;
}
int NV21Img::nvLoadNV21(unsigned char *buf, int h , int w)
{
 if (bCreated == false)
 {
  nvCreate(h, w);
 }
 int size = width*height;
 memcpy(Ycn, buf, size);
 memcpy(UVcn, buf+size, size/2);
 return 1;
}
void NV21Img::nvLoadImg(std::string filename)
{
 cv::Mat img = cv::imread(filename);
 nvCvt2NV21(img);
}
void NV21Img::nvLoadImgFromBufferBGR(unsigned char *bgrBuf, int w, int h)
{
 cv::Mat img = cv::Mat(h, w, CV_8UC3, bgrBuf);
 nvCvt2NV21(img);
}

void NV21Img::nvSaveNV21(std::string filename)
{
 FILE *file = fopen(filename.c_str(),"wb");
 fwrite(Ycn , 1 , width*height , file);
 fwrite(UVcn , 1 , width*height/2 , file);
 fclose(file);
}
void NV21Img::nvSaveGrey(std::string name)
{
 cv::imwrite(name,Ymat);
}
void NV21Img::nvSaveColor(std::string name)
{
 cv::Mat color = nvCvt2BGR();
 cv::imwrite(name, color);
}
void NV21Img::nvShowNV21(std::string winName)
{
#ifdef WIN32
 cv::Mat color = nvCvt2BGR();
 cv::namedWindow(winName, 0);
 cv::imshow(winName, color);
#endif
}
int minmax__(int a, int low, int high)
{
 if (a<low) return low;
 if (a>high) return high;
 return a;
}
cv::Mat NV21Img::nvCvt2BGR()
{
 BGRmat.create(height,width,CV_8UC3);
 uchar *BGR;
 unsigned char *Y = (unsigned char*)(Ycn);
 unsigned char *Cr = (unsigned char*)(UVcn);
 unsigned char *Cb = (unsigned char*)(UVcn + 1);

 int r, g, b;
 int y, cb, cr;
 int YLoc = 0;
 for (int i = 0; i<height; i++)
 {
  BGR = BGRmat.ptr<uchar>(i);
  int BGRLoc = 0;
  for (int j = 0; j<width; j++)
  {
   int cbcrx = j >> 1;
   int cbcry = i >> 1;
   int cbcrLoc = cbcry * (width >> 1) + cbcrx;
   //int YLoc = i*m_width+j;
   //int BGRLoc = YLoc * 3;
   y = Y[YLoc];
   cr = Cr[cbcrLoc * 2];
   cb = Cb[cbcrLoc * 2];
   // (3) - (2)ÀÇfast ¹öÀü
   r = (16384 * y + 22970 * (cr - 128)) >> 14;
   g = (16384 * y - 5638 * (cb - 128) - 11700 * (cr - 128)) >> 14;
   b = (16384 * y + 29032 * (cb - 128)) >> 14;

   BGR[BGRLoc++] = (unsigned char)minmax__(b, 0, 255);
   BGR[BGRLoc++] = (unsigned char)minmax__(g, 0, 255);
   BGR[BGRLoc++] = (unsigned char)minmax__(r, 0, 255);
   YLoc++;
  }
 }
 return BGRmat;
}
#define  CLIP(a)  ( (a) <(0) ?  (0) : ( (a) > (255) ? (255) : (a) ) )
void NV21Img::nvCvt2NV21(cv::Mat img)
{
 nvCreate(img.rows , img.cols);
 uchar *ptrs1,*ptrs2;
 uchar *ptrdY = Ycn;
 uchar *ptrdUV = UVcn;
 short R1,G1,B1,R2,G2,B2,R3,G3,B3,R4,G4,B4,R,G,B;
 float Um,Vm;
 for (int i = 0 ; i < img.rows ; i+=2,ptrdY+=stp2 , ptrdUV+=stp)
 {
  ptrs1 = img.ptr<uchar>(i);
  ptrs2 = img.ptr<uchar>(i+1);
  for (int j = 0 ; j < img.cols ; j+=2,ptrs1+=6,ptrs2+=6)
  {
   Um = 0 ; Vm = 0;
   B1 = ptrs1[0] ; G1 = ptrs1[1] ; R1 = ptrs1[2];
   B2 = ptrs1[3] ; G2 = ptrs1[4] ; R2 = ptrs1[5];
   B3 = ptrs2[0] ; G3 = ptrs2[1] ; R3 = ptrs2[2];
   B4 = ptrs2[3] ; G4 = ptrs2[4] ; R4 = ptrs2[5];
   ptrdY[j] = (0.299*R1 + 0.587*G1 + 0.114*B1)+0.5f;
   ptrdY[j+1] = (0.299*R2 + 0.587*G2 + 0.114*B2)+0.5f;
   ptrdY[j+stp] = (0.299*R3 + 0.587*G3 + 0.114*B3)+0.5f;
   ptrdY[j+stp+1] = (0.299*R4 + 0.587*G4 + 0.114*B4)+0.5f;
   R = R1+R2+R3+R4;
   G = G1+G2+G3+G4;
   B = B1+B2+B3+B4;
   ptrdUV[j+1] = CLIP((-0.147*R - 0.289*G + 0.436*B+512)/4) ;
   ptrdUV[j] = CLIP((0.615*R - 0.515*G -0.100*B+512)/4);
  }
 }
}



void NV21Img::Flip(int code)
{
 cv::Mat cvimg = nvCvt2BGR();
 cv::flip(cvimg, cvimg, code);
 nvCvt2NV21(cvimg);
}
