#pragma once
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include <boost/bind.hpp>

#include <Eigen/StdVector>
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <sstream>
#include <fstream>
#include <time.h>
#include <deque>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <queue>
#include <algorithm>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/types/sim3/sim3.h>

#include <sophus/sim3.hpp>
#include <sophus/se3.hpp>

#include <sys/types.h>
#include <sys/stat.h>

#if (defined WIN32)||(defined WIN64)
#include <windows.h>
#include <GL\GLew.h>
#include <GL\GL.h>
#include <GL\GLU.h>
#else
#include <unistd.h>
#include <time.h>
#endif // WINDOWS

#include "../SLAM_windows/debug_util/DebugLog.h"
#include "../SLAM_windows/debug_util/MatrixVisualizer.h"
#include "../SLAM_windows/debug_util/PerformAnalyzer.h"