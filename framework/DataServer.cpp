#include "DataServer.h"
Pose2D DataServer::icpPose;
Pose2D DataServer::odomPose;
Pose2D DataServer::fusedPose;
Pose2D DataServer::optimizedPose;
Pose2D DataServer::currentPose;
Eigen::Matrix3d DataServer::icpCov;
Eigen::Matrix3d DataServer::odomCov;
Eigen::Matrix3d DataServer::fusedCov;
Eigen::Matrix3d DataServer::optimizedCov;
Eigen::Matrix3d DataServer::currentCov;
Eigen::Matrix3d DataServer::totalCov;