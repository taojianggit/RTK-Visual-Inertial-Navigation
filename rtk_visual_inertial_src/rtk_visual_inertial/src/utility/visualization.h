

#pragma once

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include "../swf/swf.h"



void registerPub(ros::NodeHandle& n);

void printStatistics(const SWFOptimization& swf_optimization, double t);

void pubOdometry(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void pubInitialGuess(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void pubKeyPoses(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void pubCameraPose(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void pubPointCloud(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void pubTF(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void pubKeyframe(const SWFOptimization& swf_optimization);

void pubRelocalization(const SWFOptimization& swf_optimization);

void pubCar(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void resetpot(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

void save_result(const SWFOptimization& swf_optimization);

// 新增的残差保存函数
void save_result_total(const SWFOptimization& swf_optimization);
void save_result_gnss_simple(const SWFOptimization& swf_optimization);
void save_result_imu_simple(const SWFOptimization& swf_optimization);
void save_result_imu_detail(const SWFOptimization& swf_optimization);
void save_result_visual_simple(const SWFOptimization& swf_optimization);
void save_result_prior_simple(const SWFOptimization& swf_optimization);
void save_result_comprehensive(const SWFOptimization& swf_optimization);

// RAIM结果输出函数
void save_raim_results_csv(const SWFOptimization& swf_optimization);
void pubRaimResults(const SWFOptimization& swf_optimization, const std_msgs::Header& header);
void pubRaimVisualization(const SWFOptimization& swf_optimization, const std_msgs::Header& header);

// 坐标轴标签函数
void pubCoordinateLabels(const std_msgs::Header& header);