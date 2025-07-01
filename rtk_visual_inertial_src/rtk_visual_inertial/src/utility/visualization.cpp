

#include "visualization.h"
#include "../parameter/parameters.h"
#include <fstream>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <tf/transform_broadcaster.h>
#include "camera_pose_visualization.h"
#include "common_function.h"
using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_right;
ros::Publisher pub_rectify_pose_left;
ros::Publisher pub_rectify_pose_right;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

// RAIM相关发布器
ros::Publisher pub_raim_status;
ros::Publisher pub_raim_diagnostics;
ros::Publisher pub_raim_protection_levels;
ros::Publisher pub_raim_satellites;
ros::Publisher pub_raim_visualization;
ros::Publisher pub_coordinate_labels;

camera_pose_visualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);
#define scalefactor 1
size_t pub_counter = 0;
Vector3d        Ps0;
void registerPub(ros::NodeHandle& n) {
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_right = n.advertise<nav_msgs::Odometry>("camera_pose_right", 1000);
    pub_rectify_pose_left = n.advertise<geometry_msgs::PoseStamped>("rectify_pose_left", 1000);
    pub_rectify_pose_right = n.advertise<geometry_msgs::PoseStamped>("rectify_pose_right", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);

    // RAIM相关话题
    pub_raim_status = n.advertise<std_msgs::Bool>("raim/integrity_available", 100);
    pub_raim_diagnostics = n.advertise<diagnostic_msgs::DiagnosticArray>("raim/diagnostics", 100);
    pub_raim_protection_levels = n.advertise<std_msgs::Float64MultiArray>("raim/protection_levels", 100);
    pub_raim_satellites = n.advertise<std_msgs::Float64MultiArray>("raim/satellite_status", 100);
    pub_raim_visualization = n.advertise<visualization_msgs::MarkerArray>("raim/visualization", 100);
    pub_coordinate_labels = n.advertise<visualization_msgs::MarkerArray>("coordinate_labels", 100);

    // cameraposevisual.setScale(10);
    // cameraposevisual.setLineWidth(10);
}

Eigen::Vector3d ecef2geo_google_map(const Eigen::Vector3d& xyz) {

#define EARTH_ECCE_2            6.69437999014e-3    // WGS 84 (Earth eccentricity)^2 (m^2)
#define EARTH_MEAN_RADIUS       6371009             // Mean R of ellipsoid(m) IU Gedosey& Geophysics
#define EARTH_SEMI_MAJOR        6378137             // WGS 84 Earth semi-major axis (m)
#define R2D                     (180.0/M_PI)        // radius to degree


    Eigen::Vector3d lla = Eigen::Vector3d::Zero();
    if (xyz.x() == 0 && xyz.y() == 0) {
        LOG(ERROR) << "LLA coordinate is not defined if x = 0 and y = 0";
        return lla;
    }

    double e2 = EARTH_ECCE_2;
    double a = EARTH_SEMI_MAJOR;
    double a2 = a * a;
    double b2 = a2 * (1 - e2);
    double b = sqrt(b2);
    double ep2 = (a2 - b2) / b2;
    double p = xyz.head<2>().norm();

    // two sides and hypotenuse of right angle triangle with one angle = theta:
    double s1 = xyz.z() * a;
    double s2 = p * b;
    double h = sqrt(s1 * s1 + s2 * s2);
    double sin_theta = s1 / h;
    double cos_theta = s2 / h;

    // two sides and hypotenuse of right angle triangle with one angle = lat:
    s1 = xyz.z() + ep2 * b * pow(sin_theta, 3);
    s2 = p - a * e2 * pow(cos_theta, 3);
    h = sqrt(s1 * s1 + s2 * s2);
    double tan_lat = s1 / s2;
    double sin_lat = s1 / h;
    double cos_lat = s2 / h;
    double lat = atan(tan_lat);
    double lat_deg = lat * R2D;

    double N = a2 * pow((a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat), -0.5);
    double altM = p / cos_lat - N;

    double lon = atan2(xyz.y(), xyz.x());
    double lon_deg = lon * R2D;
    lla << lat_deg, lon_deg, altM;
    return lla;
}


void printStatistics(const SWFOptimization& swf_optimization, double t) {

    if (swf_optimization.solver_flag != SWFOptimization::SolverFlag::NonLinear)
        return;

    int index = swf_optimization.rover_count + swf_optimization.image_count - 1;
    
    printf("=====================================\n");
    printf("Timestamp: %.3f\n", swf_optimization.headers[index]);
    printf("=====================================\n");
    
    // 状态信息（简洁版本）
    auto pos = swf_optimization.InitRwgw.transpose() * swf_optimization.Ps[index];
    auto vel = swf_optimization.InitRwgw.transpose() * swf_optimization.Vs[index];
    auto ypr = Utility::R2ypr(swf_optimization.InitRwgw.transpose() * swf_optimization.Rs[index]);
    
    printf("STATE INFO:\n");
    printf("  Position (ENU): [%.2f, %.2f, %.2f]\n", pos.x(), pos.y(), pos.z());
    printf("  Velocity (ENU): [%.2f, %.2f, %.2f]\n", vel.x(), vel.y(), vel.z());
    printf("  Orientation (YPR): [%.1f, %.1f, %.1f]\n", 
           ypr.x()*180/M_PI, ypr.y()*180/M_PI, ypr.z()*180/M_PI);
    printf("  Gyro Bias: [%.6f, %.6f, %.6f]\n", 
           swf_optimization.Bgs[index].x(), swf_optimization.Bgs[index].y(), swf_optimization.Bgs[index].z());
    printf("  Acc Bias: [%.6f, %.6f, %.6f]\n", 
           swf_optimization.Bas[index].x(), swf_optimization.Bas[index].y(), swf_optimization.Bas[index].z());
    
    // 传感器状态
    printf("SENSOR COUNT:\n");
    printf("  GNSS Frames: %d, Image Frames: %d\n", 
           swf_optimization.rover_count, swf_optimization.image_count);
    
    // RTK Fix状态
    bool rtk_fix = fabs(swf_optimization.last_fix_time - swf_optimization.headers[index]) < 0.2;
    printf("  RTK Fix: %s\n", rtk_fix ? "YES" : "NO");
    
    // 残差信息（分行显示，更清晰）
    const auto& res = swf_optimization.current_residuals;
    printf("RESIDUAL INFO:\n");
    printf("  Total: %.6f\n", res.total_residual_norm);
    printf("  GNSS:  %.6f (n=%d)\n", res.gnss_stats.residual_norm, (int)res.gnss_residuals.size());
    printf("  IMU:   %.6f (type=%s)\n", res.imu_stats.residual_norm, 
           (res.imu_factor_type == 0) ? "STD" : "JOINT");
    printf("  Visual:%.6f (n=%d)\n", res.visual_stats.residual_norm, (int)res.visual_residuals.size());
    printf("  Prior: %.6f (dim=%d)\n", res.prior_stats.residual_norm, (int)res.prior_residual.size());
    
    // 性能信息
    printf("PERFORMANCE:\n");
    printf("  Process Time: %.1f ms\n", t);
    
    // IMU-GNSS因子效率
    printf("IMU-GNSS FACTORS:\n");
    printf("  Efficiency: ");
    for (int i = 0; i < swf_optimization.image_count + 1; i++) {
        if (swf_optimization.imu_gnss_factor[i])
            printf("%.2f,", swf_optimization.imu_gnss_factor[i]->update_round * 1.0 / swf_optimization.imu_gnss_factor[i]->evaluate_round);
    }
    printf("\n");
    
    printf("=====================================\n\n");



    if (ESTIMATE_EXTRINSIC) {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++) {
            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = swf_optimization.ric[i];
            eigen_T.block<3, 1>(0, 3) = swf_optimization.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if (i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }



    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;

    sum_of_path += (swf_optimization.Ps[swf_optimization.rover_count + swf_optimization.image_count - 1] - last_path).norm();
    last_path = swf_optimization.Ps[swf_optimization.rover_count + swf_optimization.image_count - 1];

}
void resetpot(const SWFOptimization& swf_optimization, const std_msgs::Header& header) {
    Ps0 = swf_optimization.Ps[0];
}
void pubOdometry(const SWFOptimization& swf_optimization, const std_msgs::Header& header) {
    Vector3d        Ps[(FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)];


    for (int i = 0; i < swf_optimization.rover_count + swf_optimization.image_count; i++) {
        Ps[i] = swf_optimization.Ps[i] - Ps0;
    }
    if (swf_optimization.solver_flag == SWFOptimization::SolverFlag::NonLinear) {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(swf_optimization.InitRwgw.transpose() * swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1]);
        Vector3d tmpP = swf_optimization.InitRwgw.transpose() * Ps[swf_optimization.rover_count + swf_optimization.image_count - 1] * scalefactor;

        Vector3d tmpv = swf_optimization.InitRwgw.transpose() * swf_optimization.Vs[swf_optimization.rover_count + swf_optimization.image_count - 1] * scalefactor;

        odometry.pose.pose.position.x = tmpP.x();
        odometry.pose.pose.position.y = tmpP.y();
        odometry.pose.pose.position.z = tmpP.z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = tmpv.x();
        odometry.twist.twist.linear.y = tmpv.y();
        odometry.twist.twist.linear.z = tmpv.z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
        if (swf_optimization.rover_count + swf_optimization.image_count - 1 < 0)return;
        // write result to file

        {
            nav_msgs::Odometry odometry;
            odometry.header = header;
            odometry.header.frame_id = "world";
            odometry.pose.pose.position.x = swf_optimization.tic[0].x();
            odometry.pose.pose.position.y = swf_optimization.tic[0].y();
            odometry.pose.pose.position.z = swf_optimization.tic[0].z();
            Quaterniond tmp_q{swf_optimization.ric[0]};
            odometry.pose.pose.orientation.x = tmp_q.x();
            odometry.pose.pose.orientation.y = tmp_q.y();
            odometry.pose.pose.orientation.z = tmp_q.z();
            odometry.pose.pose.orientation.w = tmp_q.w();
            pub_extrinsic.publish(odometry);
        }

    }
}

void save_result(const SWFOptimization& swf_optimization) {


    static bool f;
    if (!f) {
        f = true;
        std::ofstream fout(RESULT_PATH, std::ios::out);
        fout << "time,px,py,pz,vx,vy,vz,yaw,pitch,roll,ax,ay,az,bax,bay,baz,bgx,bgy,bgz,rtk_fix,mag_yaw,havegps,trajectory_sum,lat,lon,pbgx,pbgy,pbgz";
        fout << std::endl;
        fout.close();
    }


    ofstream foutC(RESULT_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    foutC.precision(10);
    Eigen::Vector3d xyz = swf_optimization.InitRwgw.transpose() * (
                              swf_optimization.base_pos + swf_optimization.Ps[swf_optimization.rover_count + swf_optimization.image_count - 1]
                          );
    Eigen::Vector3d vxyz = swf_optimization.InitRwgw.transpose() * swf_optimization.Vs[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d ypr = Utility::R2ypr(swf_optimization.InitRwgw.transpose() * swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1]);
    Eigen::Vector3d acc_w = swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1] * swf_optimization.acc_0) - G;

    Eigen::Vector3d ba = swf_optimization.Bas[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d bg = swf_optimization.Bgs[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d Pbgw = swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1] * Pbg);
    int havegps = (swf_optimization.frame_types[swf_optimization.rover_count + swf_optimization.image_count - 1] == swf_optimization.GnssFrame ||
                   swf_optimization.frame_types[swf_optimization.rover_count + swf_optimization.image_count - 2] == swf_optimization.GnssFrame ||
                   swf_optimization.frame_types[swf_optimization.rover_count + swf_optimization.image_count - 3] == swf_optimization.GnssFrame) ? 1 : 0;

    int rtk_fix = fabs(swf_optimization.last_fix_time - swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1]) < 0.2 ? 1 : 0;
    double mag_yaw = swf_optimization.mag_yaw;
    double distance = 0;

    Eigen::Vector3d xyz2 = swf_optimization.base_pos + swf_optimization.Ps[swf_optimization.rover_count + swf_optimization.image_count - 1];

    Eigen::Vector3d lla = ecef2geo_google_map( xyz2);

    foutC << xyz.x() << "," << xyz.y() << "," << xyz.z() << ","
          << vxyz.x() << "," << vxyz.y() << "," << vxyz.z() << ","
          << ypr.x() << "," << ypr.y() << "," << ypr.z() << ","
          << acc_w.x() << "," << acc_w.y() << "," << acc_w.z() << ","
          << ba.x() << "," << ba.y() << "," << ba.z() << ","
          << bg.x() << "," << bg.y() << "," << bg.z() << ","
          << rtk_fix << ","
          << mag_yaw << ","
          << havegps << ","
          << distance << ","
          << lla.x() << "," << lla.y() << ","
          << Pbgw.x() << "," << Pbgw.y() << "," << Pbgw.z();

    foutC << std::endl;
    foutC.close();
    
    // 同时保存新的详细残差CSV文件
    save_result_comprehensive(swf_optimization);
}


void pubCameraPose(const SWFOptimization& swf_optimization, const std_msgs::Header& header) {

    Vector3d        Ps[(FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)];


    for (int i = 0; i < swf_optimization.rover_count + swf_optimization.image_count; i++) {
        Ps[i] = swf_optimization.Ps[i] - Ps0;
    }

    int idx2 = swf_optimization.rover_count + swf_optimization.image_count - 1;
    if (swf_optimization.solver_flag == SWFOptimization::SolverFlag::NonLinear) {
        int i = idx2;
        Vector3d P = swf_optimization.InitRwgw.transpose() * (Ps[i] + swf_optimization.Rs[i] * swf_optimization.tic[0]) * scalefactor;
        Quaterniond R = Quaterniond(swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[i] * swf_optimization.ric[0]));

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        if (USE_STEREO) {
            Vector3d P_r = swf_optimization.InitRwgw.transpose() * (Ps[i] + swf_optimization.Rs[i] * swf_optimization.tic[1]) * scalefactor;
            Quaterniond R_r = Quaterniond(swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[i] * swf_optimization.ric[1]));

            nav_msgs::Odometry odometry_r;
            odometry_r.header = header;
            odometry_r.header.frame_id = "world";
            odometry_r.pose.pose.position.x = P_r.x();
            odometry_r.pose.pose.position.y = P_r.y();
            odometry_r.pose.pose.position.z = P_r.z();
            odometry_r.pose.pose.orientation.x = R_r.x();
            odometry_r.pose.pose.orientation.y = R_r.y();
            odometry_r.pose.pose.orientation.z = R_r.z();
            odometry_r.pose.pose.orientation.w = R_r.w();
            pub_camera_pose_right.publish(odometry_r);
        }

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        if (USE_STEREO) {
            Vector3d P = swf_optimization.InitRwgw.transpose() * (Ps[i] + swf_optimization.Rs[i] * swf_optimization.tic[1]) * scalefactor;
            Quaterniond R = Quaterniond(swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[i] * swf_optimization.ric[1]));
            cameraposevisual.add_pose(P, R);
        }
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(const SWFOptimization& swf_optimization, const std_msgs::Header& header) {
    Vector3d        Ps[(FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)];


    for (int i = 0; i < swf_optimization.rover_count + swf_optimization.image_count; i++) {
        Ps[i] = swf_optimization.Ps[i] - Ps0;
    }

    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    for (auto& it_per_id : swf_optimization.f_manager.feature) {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < swf_optimization.rover_count + swf_optimization.image_count - 1 - 2))
            continue;
        if (it_per_id.start_frame > (swf_optimization.rover_count + swf_optimization.image_count - 1) * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;

        Vector3d w_pts_i = swf_optimization.InitRwgw.transpose() * (it_per_id.ptsInWorld - Ps0);

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto& it_per_id : swf_optimization.f_manager.feature) {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < swf_optimization.rover_count + swf_optimization.image_count - 1 - 2))
            continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
                && it_per_id.solve_flag == 1 ) {
            Vector3d w_pts_i = swf_optimization.InitRwgw.transpose() * (it_per_id.ptsInWorld - Ps0);

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}



void pubKeyframe(const SWFOptimization& swf_optimization) {
    // pub camera pose, 2D-3D points of keyframe
    if (swf_optimization.solver_flag == SWFOptimization::SolverFlag::NonLinear && swf_optimization.marg_flag == 0) {
        int i = swf_optimization.i2f[FEATURE_WINDOW_SIZE - 2];
        Vector3d P = swf_optimization.Ps[i] - swf_optimization.Rs[i] * Pbg;
        Quaterniond R = Quaterniond(swf_optimization.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(swf_optimization.headers[i]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(swf_optimization.headers[i]);
        point_cloud.header.frame_id = "world";
        for (auto& it_per_id : swf_optimization.f_manager.feature) {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < FEATURE_WINDOW_SIZE - 2
                    && it_per_id.start_frame + frame_size - 1 >= FEATURE_WINDOW_SIZE - 2
                    && it_per_id.solve_flag == 1) {

                Vector3d w_pts_i = it_per_id.ptsInWorld;
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = FEATURE_WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}

// =============== 新增的残差保存函数 ===============

void save_result_total(const SWFOptimization& swf_optimization) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        std::ofstream fout(RESULT_PATH_TOTAL, std::ios::out);
        fout << "time,px,py,pz,vx,vy,vz,yaw,pitch,roll,ax,ay,az,bax,bay,baz,bgx,bgy,bgz,rtk_fix,mag_yaw,havegps,trajectory_sum,lat,lon,pbgx,pbgy,pbgz,total_residual_norm,optimization_cost\n";
        fout.close();
    }

    std::ofstream fout(RESULT_PATH_TOTAL, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    fout.precision(10);
    
    // 计算状态信息（与原save_result相同）
    Eigen::Vector3d xyz = swf_optimization.InitRwgw.transpose() * (
                              swf_optimization.base_pos + swf_optimization.Ps[swf_optimization.rover_count + swf_optimization.image_count - 1]
                          );
    Eigen::Vector3d vxyz = swf_optimization.InitRwgw.transpose() * swf_optimization.Vs[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d ypr = Utility::R2ypr(swf_optimization.InitRwgw.transpose() * swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1]);
    Eigen::Vector3d acc_w = swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1] * swf_optimization.acc_0) - G;
    Eigen::Vector3d ba = swf_optimization.Bas[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d bg = swf_optimization.Bgs[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d Pbgw = swf_optimization.InitRwgw.transpose() * (swf_optimization.Rs[swf_optimization.rover_count + swf_optimization.image_count - 1] * Pbg);
    
    int havegps = (swf_optimization.frame_types[swf_optimization.rover_count + swf_optimization.image_count - 1] == swf_optimization.GnssFrame ||
                   swf_optimization.frame_types[swf_optimization.rover_count + swf_optimization.image_count - 2] == swf_optimization.GnssFrame ||
                   swf_optimization.frame_types[swf_optimization.rover_count + swf_optimization.image_count - 3] == swf_optimization.GnssFrame) ? 1 : 0;
    int rtk_fix = fabs(swf_optimization.last_fix_time - swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1]) < 0.2 ? 1 : 0;
    double mag_yaw = swf_optimization.mag_yaw;
    double distance = 0;
    Eigen::Vector3d xyz2 = swf_optimization.base_pos + swf_optimization.Ps[swf_optimization.rover_count + swf_optimization.image_count - 1];
    Eigen::Vector3d lla = ecef2geo_google_map(xyz2);

    // 输出原有数据
    fout << xyz.x() << "," << xyz.y() << "," << xyz.z() << ","
         << vxyz.x() << "," << vxyz.y() << "," << vxyz.z() << ","
         << ypr.x() << "," << ypr.y() << "," << ypr.z() << ","
         << acc_w.x() << "," << acc_w.y() << "," << acc_w.z() << ","
         << ba.x() << "," << ba.y() << "," << ba.z() << ","
         << bg.x() << "," << bg.y() << "," << bg.z() << ","
         << rtk_fix << "," << mag_yaw << "," << havegps << "," << distance << ","
         << lla.x() << "," << lla.y() << ","
         << Pbgw.x() << "," << Pbgw.y() << "," << Pbgw.z() << ",";
    
    // 新增总残差信息
    fout << swf_optimization.current_residuals.total_residual_norm << ","
         << swf_optimization.current_residuals.optimization_cost;
    
    fout << std::endl;
    fout.close();
}

void save_result_gnss_simple(const SWFOptimization& swf_optimization) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        std::ofstream fout(RESULT_PATH_GNSS, std::ios::out);
        fout << "time,gnss_residual_count,gnss_residual_norm,gnss_weighted_norm,gnss_rms_error,rtk_carrier_phase_count,rtk_carrier_phase_norm,rtk_pseudorange_count,rtk_pseudorange_norm,doppler_count,doppler_norm,gnss_outlier_count,gnss_outlier_ratio\n";
        fout.close();
    }
    
    std::ofstream fout(RESULT_PATH_GNSS, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    fout.precision(10);
    
    const auto& gnss_residuals = swf_optimization.current_residuals.gnss_residuals;
    const auto& gnss_stats = swf_optimization.current_residuals.gnss_stats;
    
    // 简化统计（后续可以细化分类）
    fout << gnss_residuals.size() << ","
         << gnss_stats.residual_norm << ","
         << gnss_stats.weighted_norm << ","
         << gnss_stats.rms_error << ","
         << gnss_residuals.size() << "," // 暂时用总数作为载波相位数
         << gnss_stats.residual_norm << ","
         << 0 << "," << 0.0 << "," // RTK伪距数和范数
         << 0 << "," << 0.0 << "," // 多普勒数和范数
         << 0 << "," << 0.0; // 外点数和比例
    
    fout << std::endl;
    fout.close();
}

void save_result_imu_simple(const SWFOptimization& swf_optimization) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        std::ofstream fout(RESULT_PATH_IMU, std::ios::out);
        fout << "time,imu_factor_type,imu_residual_norm,imu_weighted_norm,imu_rms_error,imu_position_residual_norm,imu_velocity_residual_norm,imu_rotation_residual_norm,imu_ba_residual_norm,imu_bg_residual_norm,imu_gnss_joint_factor_active\n";
        fout.close();
    }
    
    std::ofstream fout(RESULT_PATH_IMU, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    fout.precision(10);
    
    const auto& imu_residual = swf_optimization.current_residuals.imu_residual;
    const auto& imu_stats = swf_optimization.current_residuals.imu_stats;
    
    // 计算分解的残差范数
    double pos_norm = sqrt(imu_residual.segment<3>(0).squaredNorm());
    double vel_norm = sqrt(imu_residual.segment<3>(3).squaredNorm());
    double rot_norm = sqrt(imu_residual.segment<3>(6).squaredNorm());
    double ba_norm = sqrt(imu_residual.segment<3>(9).squaredNorm());
    double bg_norm = sqrt(imu_residual.segment<3>(12).squaredNorm());
    
    fout << swf_optimization.current_residuals.imu_factor_type << ","
         << imu_stats.residual_norm << ","
         << imu_stats.weighted_norm << ","
         << imu_stats.rms_error << ","
         << pos_norm << ","
         << vel_norm << ","
         << rot_norm << ","
         << ba_norm << ","
         << bg_norm << ","
         << (swf_optimization.current_residuals.imu_factor_type == 1 ? 1 : 0);
    
    fout << std::endl;
    fout.close();
}

void save_result_imu_detail(const SWFOptimization& swf_optimization) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        std::ofstream fout(RESULT_PATH_IMU_DETAIL, std::ios::out);
        fout << "time,imu_factor_type,imu_res_0,imu_res_1,imu_res_2,imu_res_3,imu_res_4,imu_res_5,imu_res_6,imu_res_7,imu_res_8,imu_res_9,imu_res_10,imu_res_11,imu_res_12,imu_res_13,imu_res_14\n";
        fout.close();
    }
    
    std::ofstream fout(RESULT_PATH_IMU_DETAIL, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    fout.precision(10);
    
    fout << swf_optimization.current_residuals.imu_factor_type << ",";
    
    const auto& imu_residual = swf_optimization.current_residuals.imu_residual;
    for (int i = 0; i < 15; i++) {
        fout << imu_residual(i);
        if (i < 14) fout << ",";
    }
    fout << std::endl;
    fout.close();
}

void save_result_visual_simple(const SWFOptimization& swf_optimization) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        std::ofstream fout(RESULT_PATH_VISUAL, std::ios::out);
        fout << "time,visual_feature_count,visual_reprojection_x_mean,visual_reprojection_y_mean,visual_residual_norm,visual_weighted_norm,visual_rms_error,visual_max_residual,visual_min_residual,visual_mean_residual,visual_outlier_count,visual_outlier_ratio,visual_depth_mean,visual_depth_std,visual_parallax_mean\n";
        fout.close();
    }
    
    std::ofstream fout(RESULT_PATH_VISUAL, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    fout.precision(10);
    
    const auto& visual_residuals = swf_optimization.current_residuals.visual_residuals;
    const auto& visual_stats = swf_optimization.current_residuals.visual_stats;
    
    // 特征点数量
    fout << visual_residuals.size() << ",";
    
    // 计算X/Y方向重投影误差均值
    double x_mean = 0.0, y_mean = 0.0;
    double max_residual = 0.0, min_residual = 1e10;
    double mean_residual = 0.0;
    if (!visual_residuals.empty()) {
        for (const auto& res : visual_residuals) {
            x_mean += res.second(0);
            y_mean += res.second(1);
            double norm = res.second.norm();
            max_residual = std::max(max_residual, norm);
            min_residual = std::min(min_residual, norm);
            mean_residual += norm;
        }
        x_mean /= visual_residuals.size();
        y_mean /= visual_residuals.size();
        mean_residual /= visual_residuals.size();
    } else {
        min_residual = 0.0;
    }
    
    fout << x_mean << "," << y_mean << ","
         << visual_stats.residual_norm << ","
         << visual_stats.weighted_norm << ","
         << visual_stats.rms_error << ","
         << max_residual << ","
         << min_residual << ","
         << mean_residual << ","
         << 0 << "," << 0.0 << "," // 外点数和比例
         << 5.0 << "," << 2.0 << "," // 平均深度和标准差（暂时使用默认值）
         << 10.0; // 平均视差（暂时使用默认值）
    
    fout << std::endl;
    fout.close();
}

void save_result_prior_simple(const SWFOptimization& swf_optimization) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        std::ofstream fout(RESULT_PATH_PRIOR, std::ios::out);
        fout << "time,prior_residual_dim,prior_residual_norm,prior_weighted_norm,prior_rms_error,marginalization_active,marg_feature_count,marg_state_count,marg_constraint_count\n";
        fout.close();
    }
    
    std::ofstream fout(RESULT_PATH_PRIOR, std::ios::app);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << swf_optimization.headers[swf_optimization.rover_count + swf_optimization.image_count - 1] * 1e9 << ",";
    fout.precision(10);
    
    const auto& prior_residual = swf_optimization.current_residuals.prior_residual;
    const auto& prior_stats = swf_optimization.current_residuals.prior_stats;
    
    fout << prior_residual.size() << ","
         << prior_stats.residual_norm << ","
         << prior_stats.weighted_norm << ","
         << prior_stats.rms_error << ","
         << (swf_optimization.last_marg_info ? 1 : 0) << ","
         << 0 << "," << 0 << "," << 0; // 边缘化特征数、状态数、约束数（暂时使用默认值）
    
    fout << std::endl;
    fout.close();
}

void save_result_comprehensive(const SWFOptimization& swf_optimization) {
    // 确保所有6个文件使用相同的时间戳
    save_result_total(swf_optimization);
    save_result_gnss_simple(swf_optimization);
    save_result_imu_simple(swf_optimization);
    save_result_imu_detail(swf_optimization);
    save_result_visual_simple(swf_optimization);
    save_result_prior_simple(swf_optimization);
    save_raim_results_csv(swf_optimization);
}

// RAIM结果CSV文件输出
void save_raim_results_csv(const SWFOptimization& swf_optimization) {
    // 检查数据有效性
    if (swf_optimization.rover_count + swf_optimization.image_count <= 1) {
        return; // 没有足够的数据
    }
    
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        try {
            std::ofstream fout("raim_results.csv", std::ios::out);
            if (!fout.is_open()) {
                ROS_WARN("Cannot open raim_results.csv for writing");
                return;
            }
            fout << "timestamp,integrity_available,fault_detected,test_statistic,chi2_threshold,"
                 << "protection_level_horizontal,protection_level_vertical,faulty_satellite_prn,"
                 << "gdop,available_satellites,detection_probability,false_alarm_rate,"
                 << "position_x,position_y,position_z,clock_bias\n";
            fout.close();
        } catch (const std::exception& e) {
            ROS_WARN("Error initializing RAIM CSV file: %s", e.what());
            return;
        }
    }
    
    try {
        std::ofstream fout("raim_results.csv", std::ios::app);
        if (!fout.is_open()) {
            return;
        }
        
        fout.setf(std::ios::fixed, std::ios::floatfield);
        fout.precision(0);
        
        // 获取时间戳
        int index = swf_optimization.rover_count + swf_optimization.image_count - 1;
        if (index < 0 || index >= (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)) {
            fout.close();
            return;
        }
        
        double timestamp = swf_optimization.headers[index] * 1e9;
        fout << timestamp << ",";
    
        fout.precision(6);
        const auto& raim_result = swf_optimization.last_raim_result_;
        
        // RAIM基本状态
        fout << (raim_result.integrity_available ? 1 : 0) << ","
             << (raim_result.fault_detected ? 1 : 0) << ","
             << raim_result.test_statistic << ","
             << raim_result.chi2_threshold << ","
             << raim_result.protection_level_horizontal << ","
             << raim_result.protection_level_vertical << ","
             << raim_result.faulty_satellite_prn << ","
             << raim_result.gdop << ","
             << raim_result.available_satellites << ","
             << raim_result.detection_probability << ","
             << raim_result.false_alarm_rate << ",";
        
        // 位置信息
        if (index >= 0 && index < (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)) {
            Vector3d current_pos = swf_optimization.Ps[index];
            fout << current_pos.x() << ","
                 << current_pos.y() << ","
                 << current_pos.z() << ",";
        } else {
            fout << "0,0,0,";
        }
        
        // 时钟偏差
        double clock_bias = 0.0;
        if (index >= 0 && index < (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1) && 
            swf_optimization.para_gnss_dt[index] != nullptr) {
            clock_bias = swf_optimization.para_gnss_dt[index][0];
        }
        fout << clock_bias;
        
        fout << std::endl;
        fout.close();
        
    } catch (const std::exception& e) {
        ROS_WARN("Error writing RAIM CSV data: %s", e.what());
    }
}

// RAIM结果ROS话题发布
void pubRaimResults(const SWFOptimization& swf_optimization, const std_msgs::Header& header) {
    // 移除订阅者检查，确保RAIM数据始终被处理
    ROS_INFO("Publishing RAIM results...");
    
    const auto& raim_result = swf_optimization.last_raim_result_;
    
    // 发布完好性状态
    std_msgs::Bool integrity_msg;
    integrity_msg.data = raim_result.integrity_available;
    pub_raim_status.publish(integrity_msg);
    
    // 发布诊断信息
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header = header;
    
    diagnostic_msgs::DiagnosticStatus raim_status;
    raim_status.name = "RAIM_Integrity_Monitor";
    raim_status.hardware_id = "GNSS_RAIM";
    
    if (raim_result.integrity_available) {
        if (raim_result.fault_detected) {
            raim_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            raim_status.message = "Fault detected in satellite PRN " + std::to_string(raim_result.faulty_satellite_prn);
        } else {
            raim_status.level = diagnostic_msgs::DiagnosticStatus::OK;
            raim_status.message = "RAIM integrity available, no faults detected";
        }
    } else {
        raim_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        raim_status.message = "RAIM integrity not available";
    }
    
    // 添加详细参数
    diagnostic_msgs::KeyValue kv;
    
    try {
        kv.key = "test_statistic";
        kv.value = std::to_string(raim_result.test_statistic);
        raim_status.values.push_back(kv);
        
        kv.key = "chi2_threshold";
        kv.value = std::to_string(raim_result.chi2_threshold);
        raim_status.values.push_back(kv);
        
        kv.key = "gdop";
        kv.value = std::to_string(raim_result.gdop);
        raim_status.values.push_back(kv);
        
        kv.key = "available_satellites";
        kv.value = std::to_string(raim_result.available_satellites);
        raim_status.values.push_back(kv);
    } catch (const std::exception& e) {
        ROS_WARN("Error creating RAIM diagnostic values: %s", e.what());
    }
    
    diag_array.status.push_back(raim_status);
    pub_raim_diagnostics.publish(diag_array);
    
    // 发布保护级
    std_msgs::Float64MultiArray protection_msg;
    protection_msg.data.push_back(raim_result.protection_level_horizontal);
    protection_msg.data.push_back(raim_result.protection_level_vertical);
    pub_raim_protection_levels.publish(protection_msg);
    
    // 发布卫星状态
    std_msgs::Float64MultiArray sat_msg;
    sat_msg.data.push_back(raim_result.available_satellites);
    sat_msg.data.push_back(raim_result.faulty_satellite_prn);
    sat_msg.data.push_back(raim_result.gdop);
    pub_raim_satellites.publish(sat_msg);
}

// RAIM可视化发布
void pubRaimVisualization(const SWFOptimization& swf_optimization, const std_msgs::Header& header) {
    ROS_INFO("RAIM Visualization function called! rover_count=%d, image_count=%d", 
             swf_optimization.rover_count, swf_optimization.image_count);
    
    const auto& raim_result = swf_optimization.last_raim_result_;
    
    ROS_INFO("RAIM Data: integrity_available=%d, fault_detected=%d, hpl=%.2f, sats=%d",
             raim_result.integrity_available, raim_result.fault_detected, 
             raim_result.protection_level_horizontal, raim_result.available_satellites);
    
    // 检查位置数据有效性
    if (swf_optimization.rover_count + swf_optimization.image_count <= 0) {
        ROS_WARN("RAIM Visualization: No position data available");
        return;
    }
    
    visualization_msgs::MarkerArray marker_array;
    
    // 获取当前位置
    int index = swf_optimization.rover_count + swf_optimization.image_count - 1;
    ROS_INFO("Position index: %d", index);
    
    if (index < 0 || index >= (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)) {
        ROS_WARN("RAIM Visualization: Invalid index %d", index);
        return;
    }
    
    // 使用与轨迹相同的坐标变换（减去初始位置偏移）
    Vector3d pos_with_offset = swf_optimization.Ps[index] - Ps0;
    Vector3d current_pos = swf_optimization.InitRwgw.transpose() * pos_with_offset * scalefactor;
    ROS_INFO("Current position: (%.2f, %.2f, %.2f)", current_pos.x(), current_pos.y(), current_pos.z());
    
    // 创建保护级可视化圆圈 (固定在相机视野中)
    visualization_msgs::Marker protection_circle;
    protection_circle.header = header;
    protection_circle.header.frame_id = "world";  // 使用world坐标系但位置固定
    protection_circle.ns = "raim_protection";
    protection_circle.id = 0;
    protection_circle.type = visualization_msgs::Marker::CYLINDER;
    protection_circle.action = visualization_msgs::Marker::ADD;
    
    // 设置位置跟随车辆路径，显示在路径上方
    protection_circle.pose.position.x = current_pos.x();
    protection_circle.pose.position.y = current_pos.y();
    protection_circle.pose.position.z = current_pos.z() + 10.0;  // 车辆上方10米
    protection_circle.pose.orientation.w = 1.0;
    
    // 设置较小的尺寸
    protection_circle.scale.x = 8.0;  // 直径8米
    protection_circle.scale.y = 8.0;
    protection_circle.scale.z = 0.5;  // 更薄的圆圈
    
    // 根据状态设置颜色
    if (raim_result.integrity_available) {
        if (raim_result.fault_detected) {
            protection_circle.color.r = 1.0; // 红色 - 检测到故障
            protection_circle.color.g = 0.0;
            protection_circle.color.b = 0.0;
        } else {
            protection_circle.color.r = 0.0; // 绿色 - 正常
            protection_circle.color.g = 1.0;
            protection_circle.color.b = 0.0;
        }
    } else {
        protection_circle.color.r = 0.5; // 灰色 - 不可用
        protection_circle.color.g = 0.5;
        protection_circle.color.b = 0.5;
    }
    protection_circle.color.a = 0.9; // 增加不透明度，更明显
    
    marker_array.markers.push_back(protection_circle);
    
    // 创建状态文本 (固定在相机视野中)
    visualization_msgs::Marker status_text;
    status_text.header = header;
    status_text.header.frame_id = "world";  // 使用world坐标系
    status_text.ns = "raim_status_text";
    status_text.id = 1;
    status_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    status_text.action = visualization_msgs::Marker::ADD;
    
    status_text.pose.position.x = current_pos.x();  // 跟随车辆
    status_text.pose.position.y = current_pos.y();  // 跟随车辆 
    status_text.pose.position.z = current_pos.z() + 40.0;  // 轨迹上方40米
    status_text.pose.orientation.w = 1.0;
    
    status_text.scale.z = 10.0; // 更大的文字高度
    
    std::string status_str;
    try {
        status_str = "RAIM: ";
        if (raim_result.integrity_available) {
            status_str += raim_result.fault_detected ? "FAULT" : "OK";
            status_str += "\n" + raim_result.failure_reason;
            status_str += "\nHPL:" + std::to_string((int)raim_result.protection_level_horizontal) + "m";
            status_str += " GDOP:" + std::to_string(raim_result.gdop).substr(0, 4);
            status_str += " Sats:" + std::to_string(raim_result.available_satellites);
        } else {
            status_str += "N/A";
            if (!raim_result.failure_reason.empty()) {
                status_str += "\n" + raim_result.failure_reason;
            }
        }
    } catch (const std::exception& e) {
        status_str = "RAIM: ERROR";
        ROS_WARN("Error creating RAIM status string: %s", e.what());
    }
    
    status_text.text = status_str;
    status_text.color = protection_circle.color;
    status_text.color.a = 1.0; // 文本不透明
    
    marker_array.markers.push_back(status_text);
    
    ROS_INFO("RAIM: Publishing markers at (%.1f,%.1f,%.1f) - %s", 
             current_pos.x(), current_pos.y(), current_pos.z(), status_str.c_str());
    
    pub_raim_visualization.publish(marker_array);
    
    // 发布坐标轴标签
    pubCoordinateLabels(header);
    
    // 发布TF变换
    static tf::TransformBroadcaster tf_broadcaster;
    
    // 车辆中心frame
    tf::Transform vehicle_transform;
    vehicle_transform.setOrigin(tf::Vector3(current_pos.x(), current_pos.y(), current_pos.z()));
    vehicle_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster.sendTransform(tf::StampedTransform(vehicle_transform, header.stamp, "world", "vehicle_center"));
    
    // 创建固定的相机frame (相对于world，不跟随车辆移动)
    tf::Transform camera_transform;
    camera_transform.setOrigin(tf::Vector3(0, 0, 0));  // 相机在固定位置
    camera_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster.sendTransform(tf::StampedTransform(camera_transform, header.stamp, "world", "rviz_camera"));
    
}

void pubCoordinateLabels(const std_msgs::Header& header) {
    visualization_msgs::MarkerArray marker_array;
    
    // X轴标签
    visualization_msgs::Marker x_label;
    x_label.header = header;
    x_label.header.frame_id = "world";
    x_label.ns = "coordinate_labels";
    x_label.id = 0;
    x_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    x_label.action = visualization_msgs::Marker::ADD;
    x_label.pose.position.x = 15.0;  // X轴上15米处
    x_label.pose.position.y = 0.0;
    x_label.pose.position.z = 0.0;
    x_label.pose.orientation.w = 1.0;
    x_label.scale.z = 3.0;  // 文字高度
    x_label.color.r = 1.0;  // 红色
    x_label.color.g = 0.0;
    x_label.color.b = 0.0;
    x_label.color.a = 1.0;
    x_label.text = "X";
    marker_array.markers.push_back(x_label);
    
    // Y轴标签
    visualization_msgs::Marker y_label;
    y_label.header = header;
    y_label.header.frame_id = "world";
    y_label.ns = "coordinate_labels";
    y_label.id = 1;
    y_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    y_label.action = visualization_msgs::Marker::ADD;
    y_label.pose.position.x = 0.0;
    y_label.pose.position.y = 15.0;  // Y轴上15米处
    y_label.pose.position.z = 0.0;
    y_label.pose.orientation.w = 1.0;
    y_label.scale.z = 3.0;  // 文字高度
    y_label.color.r = 0.0;
    y_label.color.g = 1.0;  // 绿色
    y_label.color.b = 0.0;
    y_label.color.a = 1.0;
    y_label.text = "Y";
    marker_array.markers.push_back(y_label);
    
    // Z轴标签
    visualization_msgs::Marker z_label;
    z_label.header = header;
    z_label.header.frame_id = "world";
    z_label.ns = "coordinate_labels";
    z_label.id = 2;
    z_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    z_label.action = visualization_msgs::Marker::ADD;
    z_label.pose.position.x = 0.0;
    z_label.pose.position.y = 0.0;
    z_label.pose.position.z = 15.0;  // Z轴上15米处
    z_label.pose.orientation.w = 1.0;
    z_label.scale.z = 3.0;  // 文字高度
    z_label.color.r = 0.0;
    z_label.color.g = 0.0;
    z_label.color.b = 1.0;  // 蓝色
    z_label.color.a = 1.0;
    z_label.text = "Z";
    marker_array.markers.push_back(z_label);
    
    pub_coordinate_labels.publish(marker_array);
}
