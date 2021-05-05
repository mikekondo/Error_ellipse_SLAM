/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file MapDrawer.h
 * @author Masahiro Tomono
 ****************************************************************************/

#include <stdio.h>
#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"
#include "CovarianceCalculator.h"    //近藤追記:誤差楕円可視化のため
#include "ScanMatcher2D.h"           //近藤追記:誤差楕円可視化のため
#include "PoseGraph.h"               //近藤追記:誤差楕円可視化のため
#include <boost/circular_buffer.hpp> //近藤追記:誤差楕円可視化のため
#include "LoopDetector.h"            //近藤追記:誤差楕円可視化のため
#include "SlamBackEnd.h"             //近藤追記:誤差楕円可視化のため
class MapDrawer
{
private:
  FILE *gp;             // gnuplotへのパイプ
  PointCloudMap *pcmap; // 点群地図
  ScanMatcher2D *smat;  // スキャンマッチング
  double xmin;          // 描画範囲[m]
  double xmax;
  double ymin;
  double ymax;
  double aspectR; // xy比

public:
  MapDrawer() : gp(nullptr), xmin(-20), xmax(20), ymin(-20), ymax(20), aspectR(-1.0), smat(nullptr)
  {
  }

  ~MapDrawer()
  {
    finishGnuplot();
  }

  /////////

  void initGnuplot()
  {
#ifdef _WIN32
    gp = _popen("gnuplot", "w"); // パイプオープン.Windows
#elif __linux__
    gp = popen("gnuplot", "w"); // パイプオープン.Linux
#endif
  }

  void finishGnuplot()
  {
    if (gp != nullptr)
#ifdef _WIN32
      _pclose(gp);
#elif __linux__
      pclose(gp);
#endif
  }

  void setAspectRatio(double a)
  {
    aspectR = a;
    fprintf(gp, "set size ratio %lf\n", aspectR);
  }

  void setRange(double R)
  { // 描画範囲をR四方にする
    xmin = ymin = -R;
    xmax = ymax = R;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void setRange(double xR, double yR)
  { // 描画範囲を±xR、±yRにする
    xmin = -xR;
    xmax = xR;
    ymin = -yR;
    ymax = yR;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void setRange(double xm, double xM, double ym, double yM)
  { // 描画範囲を全部指定
    xmin = xm;
    xmax = xM;
    ymin = ym;
    ymax = yM;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  ////////

  void drawMapGp(const PointCloudMap &pcmap);
  void drawScanGp(const Scan2D &scan);
  void drawTrajectoryGp(const std::vector<Pose2D> &poses);
  void drawGp(const std::vector<LPoint2D> &lps, const std::vector<Pose2D> &poses, bool flush = true);
  void drawEllipse(Pose2D &curPose, Eigen::Matrix3d &cov);
};
