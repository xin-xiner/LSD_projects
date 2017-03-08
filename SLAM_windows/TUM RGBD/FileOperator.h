#pragma once
#include "preprocessHeaders.h"
#include <opencv2\core\core.hpp>
#include <string>
#include <fstream>
#include <iostream>
template<class Vec3,class Vec4>
 class FileParser
 {
 private:
  std::ifstream rgbRecord;
  std::ifstream depthRecord;
  std::ifstream accelerometer;
  std::ifstream groundtruth;
 public:
  std::string timeStamp;
  std::string datasetPath;
  FileParser(const std::string _datasetPath)
  {
   datasetPath = _datasetPath;
   rgbRecord.open(datasetPath + "list\\rgb.txt");
   depthRecord.open(datasetPath + "list\\depth.txt");
   //accelerometer.open(datasetPath + "list\\accelerometer.txt");
   groundtruth.open(datasetPath + "list\\groundtruth.txt");
   if (!(rgbRecord.is_open() && depthRecord.is_open() && groundtruth.is_open()// && accelerometer.is_open()
	   ))
   {
    std::cout << "openFile error" << std::endl;
    system("pause");
    exit(-1);
   }
   timeStamp = "";
  }

  bool getNextRecord(std::string& _rgbImgPath, std::string& _depthImgPath, Vec3& _accelerometer, Vec3& _translate, Vec4& _rotation);
  bool getNextFrameRecord(std::string& _rgbImgPath, std::string& _depthImgPath, Vec3& _accelerometer);
  bool getNextTrajectoryRecord(Vec3& _translate, Vec4& _rotation);
  bool getNextRGBRecord(std::string& _rgbImgPath);
  bool getNextDepthRecord(std::string& _rgbImgPath);
  bool getNextAccelerometerRecord(Vec3& _accelerometer);
 private:
  bool parseLine(const std::string& s, std::string& imgPath);
  bool parseLine(const std::string& s, Vec3& accelerometer);
  bool parseLine(const std::string& s, Vec3& translate, Vec4& rotation);
  bool getLine(std::ifstream& file, std::string& l);
  bool verifyTimeStamp(std::string  stamp);
 };











 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getNextFrameRecord(std::string& _rgbImgPath, std::string& _depthImgPath, Vec3& _accelerometer)
 {
  timeStamp = "";
  std::string line;
  if (!getLine(rgbRecord, line))return false;
  if (!parseLine(line, _rgbImgPath))return false;
  if (!getLine(depthRecord, line))return false;
  if (!parseLine(line, _depthImgPath))return false;
  if (!getLine(accelerometer, line))return false;
  if (!parseLine(line, _accelerometer))return false;
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getNextTrajectoryRecord(Vec3& _translate, Vec4& _rotation)
 {
  timeStamp = "";
  std::string line;
  if (!getLine(groundtruth, line))
   return false;
  if (!parseLine(line, _translate, _rotation))
   return false;
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getNextRecord(std::string& _rgbImgPath, std::string& _depthImgPath, Vec3& _accelerometer, Vec3& _translate, Vec4& _rotation)
 {
  timeStamp = "";
  std::string line;
  std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
  if (!getLine(rgbRecord, line))return false;
#ifdef DEBUG
  std::cout << "rgbTimeStamp  ";
#endif
  if (!parseLine(line, _rgbImgPath))return false;
  if (!getLine(depthRecord, line))return false;
#ifdef DEBUG
  std::cout << "depthTimeStamp  ";
#endif
  if (!parseLine(line, _depthImgPath))return false;
  //if (!getLine(accelerometer, line))return false;
#ifdef DEBUG
  std::cout << "accelerometerTimeStamp  ";
#endif
  //if (!parseLine(line, _accelerometer))return false;
  if (!getLine(groundtruth, line))
   return false;
#ifdef DEBUG
  std::cout << "groundtruthTimeStamp  ";
#endif
  if (!parseLine(line, _translate, _rotation))
   return false;
#ifdef DEBUG
  std::cout << std::endl << std::endl;
#endif
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getNextRGBRecord(std::string& _rgbImgPath)
 {
  timeStamp = "";
  std::string line;
  if (!getLine(rgbRecord, line))return false;
  if (!parseLine(line, _rgbImgPath))return false;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getNextDepthRecord(std::string& _depthImgPath)
 {
  timeStamp = "";
  std::string line;
  if (!getLine(depthRecord, line))return false;
  if (!parseLine(line, _depthImgPath))return false;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getNextAccelerometerRecord(Vec3& _accelerometer)
 {
  timeStamp = "";
  std::string line;
  if (!getLine(accelerometer, line))return false;
  if (!parseLine(line, _accelerometer))return false;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::getLine(std::ifstream& file, std::string& l)
 {
  do
  {
   if (!std::getline(file, l)) return false;
  } while (l[0] == '#');
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::verifyTimeStamp(std::string currentTimeStamp)
 {
  if (timeStamp == "")timeStamp = currentTimeStamp;
  /*else if (abs(timeStamp - currentTimeStamp)>0.02)
  return false;
  else */
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::parseLine(const std::string& s, std::string& imgPath)
 {
  std::string currentTimeStamp;
  std::stringstream stream(s);
  stream >> currentTimeStamp;
#ifdef DEBUG
  disp(currentTimeStamp);
#endif
  if (!(verifyTimeStamp(currentTimeStamp)))return false;
  if (!(stream >> imgPath))return false;
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::parseLine(const std::string& s, Vec3& accelerometer)
 {
  std::string currentTimeStamp;
  std::stringstream stream(s);
  stream >> currentTimeStamp;
#ifdef DEBUG
  disp(currentTimeStamp);
#endif
  if (!(verifyTimeStamp(currentTimeStamp)))return false;
  double d[3];
  for (int i = 0; i < 3; i++)
  {
   if (!(stream >> accelerometer(i)))return false;
  }
  return true;
 }
 template<class Vec3, class Vec4>
 bool FileParser<Vec3, Vec4>::parseLine(const std::string& s, Vec3& translate, Vec4& rotation)
 {
  std::string currentTimeStamp;
  std::stringstream stream(s);
  stream >> currentTimeStamp;
#ifdef DEBUG
  disp(currentTimeStamp);
#endif
  if (!(verifyTimeStamp(currentTimeStamp)))return false;
  for (int i = 0; i < 3; i++)
  {
   if (!(stream >> translate(i)))return false;
  }
  for (int i = 0; i < 4; i++)
  {
   if (!(stream >> rotation(i)))return false;
  }
  return true;
 }