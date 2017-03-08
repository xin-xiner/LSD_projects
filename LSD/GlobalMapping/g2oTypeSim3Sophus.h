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

class VertexSim3 : public g2o::BaseVertex<7, Sophus::Sim3d>//g2o�п���ʹ���Լ�����Ķ������ͣ�������һЩ���������㺯���ķ�ʽֱ��ʹ��g2o//ģ����ָ��������ά��7���Ͷ��㱾�������
{
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 VertexSim3();
 virtual bool read(std::istream& is);
 virtual bool write(std::ostream& os) const;
 virtual void setToOriginImpl() {
  _estimate = Sophus::Sim3d();
 }
 virtual void oplusImpl(const double* update_);//wx- update_�������Ե����е�deltaX�������ص��Ǽ������ deltaX ���� X�ĺ��������ڵ�ʵ����ת����sim3Ȼ��ֱ�����㡣Ҳ��g2o����ר��ΪSLAMʵ�ֵĶ��㣬����g2o������˵���ܻ���죬��������Ķ������ʱsim3����so3����δ֪
 bool _fix_scale;
};
/**
* \brief 7D edge between two Vertex7
*/                     //wx-ͬ��Ϊ�˽�g2o���Ӧ�õ���ǰ�����У���Ҫ����g2o�еıߵ�����
class EdgeSim3 : public g2o::BaseBinaryEdge<7, Sophus::Sim3d, VertexSim3, VertexSim3>//wx-�����ģ��������Ҫָ������ά����7���ߵ����ͼ�Sim3d�������ӵ�������������ͼ�VertexSim3
{
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 EdgeSim3();
 
 virtual bool read(std::istream& is);
 virtual bool write(std::ostream& os) const;
 
 void computeError();              //wx-����g2o��Ҫ����Ҫ����computeError�������������㵱ǰֵ�µ����  
 
 void linearizeOplus();             //����ָ����jacobi����ļ��㣬�����ָ��Ĭ��ʹ����ֵ��������


 virtual void setMeasurement(const Sophus::Sim3d& m);//ָ���ߵĺ���
 
 virtual bool setMeasurementData(const double* m);
 
 virtual bool setMeasurementFromState();
 virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
 
 virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/);
 
protected:
 Sophus::Sim3d _inverseMeasurement;
};
}