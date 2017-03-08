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
#pragma once
#include "../preprocessHeaders.h"
#include "../util/SophusUtil.h"


namespace lsd_slam
{

class VertexSim3 : public g2o::BaseVertex<7, Sophus::Sim3d>//g2o中可以使用自己定义的顶点类型，并重载一些顶点间的运算函数的方式直接使用g2o//模板里指定了误差的维数7，和顶点本身的类型
{
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 VertexSim3();
 virtual bool read(std::istream& is);
 virtual bool write(std::ostream& os) const;
 virtual void setToOriginImpl() {
  _estimate = Sophus::Sim3d();
 }
 virtual void oplusImpl(const double* update_);//wx- update_即非线性迭代中的deltaX这里重载的是计算好了 deltaX 更新 X的函数，现在的实现是转换到sim3然后直接运算。也许g2o中有专门为SLAM实现的顶点，按照g2o论文来说可能会更快，不过这里的顶点更新时sim3不是so3所以未知
 bool _fix_scale;
};
/**
* \brief 7D edge between two Vertex7
*/                     //wx-同样为了将g2o框架应用到当前问题中，需要重载g2o中的边的类型
class EdgeSim3 : public g2o::BaseBinaryEdge<7, Sophus::Sim3d, VertexSim3, VertexSim3>//wx-这里的模板类里需要指定误差的维数即7，边的类型及Sim3d，边连接的两个顶点的类型即VertexSim3
{
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 EdgeSim3();
 
 virtual bool read(std::istream& is);
 virtual bool write(std::ostream& os) const;
 
 void computeError();              //wx-按照g2o的要求，需要重载computeError函数，用来计算当前值下的误差  
 
 void linearizeOplus();             //这里指定了jacobi矩阵的计算，如果不指定默认使用数值方法计算


 virtual void setMeasurement(const Sophus::Sim3d& m);//指定边的函数
 
 virtual bool setMeasurementData(const double* m);
 
 virtual bool setMeasurementFromState();
 virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
 
 virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/);
 
protected:
 Sophus::Sim3d _inverseMeasurement;
};
}