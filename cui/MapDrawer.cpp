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
 * @file MapDrawer.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "MapDrawer.h"
#include <iostream> //近藤追記
using namespace std;

////////// Gnuplotによる地図描画 //////////

// 地図と軌跡を描画
void MapDrawer::drawMapGp(const PointCloudMap &pcmap)
{
  const vector<LPoint2D> &lps = pcmap.globalMap; // 地図の点群
  const vector<Pose2D> &poses = pcmap.poses;     // ロボット軌跡
  drawGp(lps, poses);
}

// スキャン1個を描画
void MapDrawer::drawScanGp(const Scan2D &scan)
{
  vector<Pose2D> poses;
  Pose2D pose;              // 原点
  poses.emplace_back(pose); // drawGpを使うためにvectorに入れる
  drawGp(scan.lps, poses);
}

// ロボット軌跡だけを描画
void MapDrawer::drawTrajectoryGp(const vector<Pose2D> &poses)
{
  vector<LPoint2D> lps; // drawGpを使うためのダミー（空）
  drawGp(lps, poses);
}

void MapDrawer::drawGp(const vector<LPoint2D> &lps, const vector<Pose2D> &poses, bool flush)
{
  printf("drawGp: lps.size=%lu\n", lps.size()); // 点数の確認用

  // gnuplot設定
  setAspectRatio(-1.0);
  fprintf(gp, "set multiplot\n");
  //  fprintf(gp, "plot '-' w p pt 7 ps 0.1, '-' with vector\n");
  //fprintf(gp, "plot '-' w p pt 7 ps 0.1 lc rgb 0xff0000, '-' with vector\n"); //変更
  //fprintf(gp, "plot '-' w p pt 7 ps 0.5 lc rgb 0x0, '-' with vector,'-' w p pt 7 ps 0.5 lc rgb 0xff0000"); //fusion
  fprintf(gp, "plot '-' w p pt 7 ps 0.3 lc rgb 0x0, '-' with vector,'-' w p pt 7 ps 0.5 lc rgb 0x0000ff"); //odom
  // 点群の描画
  int step1 = 1; // 点の間引き間隔。描画が重いとき大きくする
  for (size_t i = 0; i < lps.size(); i += step1)
  {
    const LPoint2D &lp = lps[i];
    fprintf(gp, "%lf %lf\n", lp.x, lp.y); // 点の描画
  }
  fprintf(gp, "e\n");
  // ロボット軌跡の描画
  int step2 = 10; // ロボット位置の間引き間隔
  for (size_t i = 0; i < poses.size(); i += step2)
  {
    const Pose2D &pose = poses[i];
    double cx = pose.tx; // 並進位置
    double cy = pose.ty;
    double cs = pose.Rmat[0][0]; // 回転角によるcos
    double sn = pose.Rmat[1][0]; // 回転角によるsin

    // ロボット座標系の位置と向きを描く
    double dd = 0.9;
    double x1 = cs * dd; // ロボット座標系のx軸
    double y1 = sn * dd;
    double x2 = -sn * dd; // ロボット座標系のy軸
    double y2 = cs * dd;
    fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x1, y1);
    fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x2, y2);
  }
  fprintf(gp, "e\n");
  //融合後誤差楕円の描画
  /* Pose2D fusePose = data_save->get_fusedPose();
  Eigen::Matrix3d fuse_cov = data_save->get_fusedCov();
  double EigenValues[2], EigenVec1[2], EigenVec2[2];
  double fuse_cov2[2][2];
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      fuse_cov2[i][j] = fuse_cov(i, j);
    }
  }
  MyUtil::calEigen2D(fuse_cov2, EigenValues, EigenVec1, EigenVec2);
  int step3 = 10; // 楕円点の間引き数
  for (int i = 0; i < 360; i += step3)
  {
    // 0 deg
    double a = 100; //長軸、短軸の調整パラメータ
    double x = a * sqrt(EigenValues[0]) * cos(DEG2RAD(i));
    double y = a * sqrt(EigenValues[1]) * sin(DEG2RAD(i));
    double rotation = DEG2RAD(i * 0.1);
    cout << "rotation" << rotation << endl;
    // グローバル座標系の楕円座標
    double ellipse_x = cos(fusePose.th) * x - sin(fusePose.th) * y + fusePose.tx;
    double ellipse_y = sin(fusePose.th) * x + cos(fusePose.th) * y + fusePose.ty;
    cout << "ellipse:x" << ellipse_x << endl;
    cout << "ellipse:y" << ellipse_y << endl;
    fprintf(gp, "%lf %lf\n", ellipse_x, ellipse_y);
  }
  fprintf(gp, "e\n");
 */
  //オドメトリ誤差楕円の描画
  Pose2D odomPose = data_save->get_odomPose();
  Eigen::Matrix3d odom_cov = data_save->get_odomCov();
  double EigenValues[2], EigenVec1[2], EigenVec2[2];
  double odom_cov2[2][2];
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      odom_cov2[i][j] = odom_cov(i, j);
    }
  }
  MyUtil::calEigen2D(odom_cov2, EigenValues, EigenVec1, EigenVec2);
  int step3 = 10; // 楕円点の間引き数
  for (int i = 0; i < 360; i += step3)
  {
    // 0 deg
    double a = 100; //長軸、短軸の調整パラメータ
    double x = a * sqrt(EigenValues[0]) * cos(DEG2RAD(i));
    double y = a * sqrt(EigenValues[1]) * sin(DEG2RAD(i));
    double rotation = DEG2RAD(i * 0.1);
    cout << "rotation" << rotation << endl;
    // グローバル座標系の楕円座標
    double ellipse_x = cos(odomPose.th) * x - sin(odomPose.th) * y + odomPose.tx;
    double ellipse_y = sin(odomPose.th) * x + cos(odomPose.th) * y + odomPose.ty;
    cout << "ellipse:x" << ellipse_x << endl;
    cout << "ellipse:y" << ellipse_y << endl;
    fprintf(gp, "%lf %lf\n", ellipse_x, ellipse_y);
  }
  fprintf(gp, "e\n");

  fflush(gp); // バッファのデータを書き出す。これしないと描画がよくない
}
