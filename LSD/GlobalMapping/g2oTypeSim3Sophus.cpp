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
#include "g2oTypeSim3Sophus.h"

namespace lsd_slam
{

G2O_USE_TYPE_GROUP(sba);
G2O_REGISTER_TYPE_GROUP(sim3sophus);
G2O_REGISTER_TYPE(VERTEX_SIM3_SOPHUS:EXPMAP, VertexSim3);
G2O_REGISTER_TYPE(EDGE_SIM3_SOPHUS:EXPMAP, EdgeSim3);
VertexSim3::VertexSim3() : g2o::BaseVertex<7, Sophus::Sim3d>()
{
 _marginalized=false;
 _fix_scale = true;
}
bool VertexSim3::write(std::ostream& os) const
{
 // TODO
 assert(false);
 return false;
//     Sim3 cam2world(estimate().inverse());
//     Vector7d lv=cam2world.log();
//     for (int i=0; i<7; i++){
//       os << lv[i] << " ";
//     }
//     for (int i=0; i<2; i++)
//     {
//       os << _focal_length[i] << " ";
//     }
//     for (int i=0; i<2; i++)
//     {
//       os << _principle_point[i] << " ";
//     }
//     return os.good();
}
bool VertexSim3::read(std::istream& is)
{
 // TODO
 assert(false);
 return false;
//     Vector7d cam2world;
//     for (int i=0; i<6; i++){
//       is >> cam2world[i];
//     }
//     is >> cam2world[6];
// //    if (! is) {
// //      // if the scale is not specified we set it to 1;
// //      std::cerr << "!s";
// //      cam2world[6]=0.;
// //    }
// 
//     for (int i=0; i<2; i++)
//     {
//       is >> _focal_length[i];
//     }
//     for (int i=0; i<2; i++)
//     {
//       is >> _principle_point[i];
//     }
// 
//     setEstimate(Sim3(cam2world).inverse());
//     return true;
}

EdgeSim3::EdgeSim3() :
 g2o::BaseBinaryEdge<7, Sophus::Sim3d, VertexSim3, VertexSim3>()
{
}
bool EdgeSim3::write(std::ostream& os) const
{
 // TODO
 assert(false);
 return false;
//     Sim3 cam2world(measurement().inverse());
//     Vector7d v7 = cam2world.log();
//     for (int i=0; i<7; i++)
//     {
//       os  << v7[i] << " ";
//     }
//     for (int i=0; i<7; i++)
//       for (int j=i; j<7; j++){
//         os << " " <<  information()(i,j);
//     }
//     return os.good();
}
bool EdgeSim3::read(std::istream& is)
{
 // TODO
 assert(false);
 return false;
//     Vector7d v7;
//     for (int i=0; i<7; i++){
//       is >> v7[i];
//     }
// 
//     Sim3 cam2world(v7);
//     setMeasurement(cam2world.inverse());
// 
//     for (int i=0; i<7; i++)
//       for (int j=i; j<7; j++)
//       {
//         is >> information()(i,j);
//         if (i!=j)
//           information()(j,i)=information()(i,j);
//       }
//     return true;
}

void VertexSim3::oplusImpl(const double* update_)//wx- update_即非线性迭代中的deltaX这里重载的是计算好了 deltaX 更新 X的函数，现在的实现是转换到sim3然后直接运算。也许g2o中有专门为SLAM实现的顶点，按照g2o论文来说可能会更快，不过这里的顶点更新时sim3不是so3所以未知
{
 Eigen::Map< Eigen::Matrix<double, 7, 1> > update(const_cast<double*>(update_));
 if (_fix_scale) update[6] = 0;
 setEstimate(Sophus::Sim3d::exp(update) * estimate());//wx-estimate()是当前的X值，使用update求了这次迭代之后更新了的值
}

void EdgeSim3::computeError()              //wx-按照g2o的要求，需要重载computeError函数，用来计算当前值下的误差  
{                  //这里计算的是：  起点X1和边的终点X2都是继承自sim3d，可以直接求得相互之间的一个变换Tx。边之间也记录了一个变换T。Tx*T.inverse()为0是理想值，所以直接拿这个值当误差              
 const VertexSim3* _from = static_cast<const VertexSim3*>(_vertices[0]);//wx-边的起始点
 const VertexSim3* _to = static_cast<const VertexSim3*>(_vertices[1]);//wx-边的终止点
 Sophus::Sim3d error_ = _from->estimate().inverse() * _to->estimate() * _inverseMeasurement;//wx- Tx*T.inverse()
 _error = error_.log();//转换成7维的误差向量
}
void  EdgeSim3::linearizeOplus()             //这里指定了jacobi矩阵的计算，如果不指定默认使用数值方法计算
{
 const VertexSim3* _from = static_cast<const VertexSim3*>(_vertices[0]);
 _jacobianOplusXj = _from->estimate().inverse().Adj();
 _jacobianOplusXi = -_jacobianOplusXj;
}
void EdgeSim3::setMeasurement(const Sophus::Sim3d& m)//指定边的函数
{

 _measurement = m;
 _inverseMeasurement = m.inverse();
 //wx-debug 
 //Sophus::Sim3d temp = m;
 //Eigen::Matrix4d transTemp = temp.matrix();
 //debugLogLine("----------------------before scale")
 //for (int ii = 0; ii < 4; ii++)
 //{
 // for (int jj = 0; jj < 4; jj++)
 // {
 //  debugLog(transTemp(ii, jj)); debugLog(" ");//wx-debug
 // }
 // debugLog("\n");
 //}
 //debugLogLine("\n\n");
 //temp.setScale(1);
 //transTemp = temp.matrix();
 //debugLogLine("----------------------after scale")
 // for (int ii = 0; ii < 4; ii++)
 // {
 //  for (int jj = 0; jj < 4; jj++)
 //  {
 //   debugLog(transTemp(ii, jj)); debugLog(" ");//wx-debug
 //  }
 //  debugLog("\n");
 // }
 //debugLogLine("\n\n");
 //_measurement = temp;
 //_inverseMeasurement = temp.inverse();
}
bool EdgeSim3::setMeasurementData(const double* m)
{
 Eigen::Map<const g2o::Vector7d> v(m);
 setMeasurement(Sophus::Sim3d::exp(v));
 return true;
}
bool EdgeSim3::setMeasurementFromState()
{
 const VertexSim3* from = static_cast<const VertexSim3*>(_vertices[0]);
 const VertexSim3* to = static_cast<const VertexSim3*>(_vertices[1]);
 Sophus::Sim3d delta = from->estimate().inverse() * to->estimate();
 setMeasurement(delta);
 return true;
}
void EdgeSim3::initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
{
 VertexSim3 *_from = static_cast<VertexSim3*>(_vertices[0]);
 VertexSim3 *_to = static_cast<VertexSim3*>(_vertices[1]);
 if (from.count(_from) > 0)
  _to->setEstimate(_from->estimate() * _measurement);
 else
  _from->setEstimate(_to->estimate() * _inverseMeasurement);
}

}