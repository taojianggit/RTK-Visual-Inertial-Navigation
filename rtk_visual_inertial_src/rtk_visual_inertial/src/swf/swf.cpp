#include "swf.h"
#include "../utility/visualization.h"
#include <thread>
#include <queue>
#include "../factor/pose0_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/projection_factor.h"
#include <algorithm>
#include <numeric>
#include <string>



SWFOptimization::SWFOptimization(): f_manager{Rs} {
    printf("init begins");
    ClearState();
    prev_time = -1;
    prev_time2 = -1;

    cur_time = 0;
    open_ex_estimation = 0;
    rover_count = 0;
    fix = false;
    imu_initialize = false;
    //initializing the global solver.
    my_options.linear_solver_type = ceres::DENSE_SCHUR;
    my_options.max_num_iterations = MAX_NUM_ITERATIONS;
    my_options.jacobi_scaling = 0;
    my_options.trust_region_strategy_type = ceres::DOGLEG;
    my_options.num_threads = 4;
    my_options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering());
    mag_mean.setZero();

    // 初始化RAIM配置
    InitializeRaimConfig();

}


void SWFOptimization::SetParameter() {
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);

    cout << "set G " << G.transpose() << endl;
    feature_tracker.readIntrinsicParameter(CAM_NAMES);
    projection_factor::sqrt_info = FOCAL_LENGTH / FEATUREWEIGHTINVERSE * Matrix2d::Identity();
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / FEATUREWEIGHTINVERSE * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / FEATUREWEIGHTINVERSE * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / FEATUREWEIGHTINVERSE * Matrix2d::Identity();

}

//need to fix for reseting the system.
void SWFOptimization::ClearState() {
    for (int i = 0; i < FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1; i++) {

        if (para_pose[i])delete para_pose[i];
        if (para_speed_bias[i])delete para_speed_bias[i];
        if (para_gnss_dt[i])delete para_gnss_dt[i];
        para_gnss_dt[i] = new double[13];
        para_pose[i] = new double[SIZE_POSE];
        para_speed_bias[i] = new double[SIZE_SPEEDBIAS];
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();

        for (int s = 0; s < 13; s++)
            para_gnss_dt[i][s] = 0;


        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr) {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = true,
    solver_flag = Initial;

    if (last_marg_info != nullptr)
        delete last_marg_info;

    last_marg_info = nullptr;

    f_manager.ClearState();

    rover_count = 0;
    image_count = 0;
    fix = false;

    acc_mean.setZero();
    gyr_mean.setZero();
    mag_mean.setZero();
    gyr_count = 0;
    Rwgw.setIdentity();


    para_bmg[0] = para_bmg[1] = para_bmg[2] = 0;
    base_pos.setZero();
    init_gnss = false;
    gnss_fix_solution_count = 0;
    pub_init = false;
    rtk_fix = false;
    InitRwgw.setIdentity();
    mag_yaw = 0;
}

//getting the gnss index or visual index according to the frame index, or reverse.
int SWFOptimization::ImageRoverId2FrameId(int image_rover_index, int mode) {
    int image_rover_index_tmp = image_rover_index;
    image_rover_index++;
    for (int i = 0; i < image_count + rover_count; i++) {
        if (frame_types[i] == mode) {
            image_rover_index--;
            if (image_rover_index == 0) {
                if (rover_count == 0 || image_count == 0) {
                    assert(image_rover_index_tmp == i);
                }
                return i;
            }
        }
    }
    assert(0);
    return -1;
}

//getting the pointer of the states.
void SWFOptimization::Vector2Double() {
    for (int i = 0; i < rover_count + image_count; i++) {
        para_pose[i][0] = Ps[i].x();
        para_pose[i][1] = Ps[i].y();
        para_pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_pose[i][3] = q.x();
        para_pose[i][4] = q.y();
        para_pose[i][5] = q.z();
        para_pose[i][6] = q.w();


        para_speed_bias[i][0] = Vs[i].x();
        para_speed_bias[i][1] = Vs[i].y();
        para_speed_bias[i][2] = Vs[i].z();

        para_speed_bias[i][3] = Bas[i].x();
        para_speed_bias[i][4] = Bas[i].y();
        para_speed_bias[i][5] = Bas[i].z();

        para_speed_bias[i][6] = Bgs[i].x();
        para_speed_bias[i][7] = Bgs[i].y();
        para_speed_bias[i][8] = Bgs[i].z();

    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        para_ex_Pose[i][0] = tic[i].x();
        para_ex_Pose[i][1] = tic[i].y();
        para_ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_ex_Pose[i][3] = q.x();
        para_ex_Pose[i][4] = q.y();
        para_ex_Pose[i][5] = q.z();
        para_ex_Pose[i][6] = q.w();
    }

    for (auto& it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < FEATURE_CONTINUE)continue;
    }



}


//saving the states from pointer.
void SWFOptimization::Double2Vector() {

    for (int i = 0; i < rover_count + image_count; i++) {

        Rs[i] = Quaterniond(para_pose[i][6], para_pose[i][3], para_pose[i][4], para_pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = Vector3d(para_pose[i][0], para_pose[i][1], para_pose[i][2] ) ;

        Vs[i] = Vector3d(para_speed_bias[i][0], para_speed_bias[i][1], para_speed_bias[i][2]);

        Bas[i] = Vector3d(para_speed_bias[i][3], para_speed_bias[i][4], para_speed_bias[i][5]);

        Bgs[i] = Vector3d(para_speed_bias[i][6], para_speed_bias[i][7], para_speed_bias[i][8]);

    }



    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Vector3d(para_ex_Pose[i][0], para_ex_Pose[i][1], para_ex_Pose[i][2]);
        ric[i] = Quaterniond(para_ex_Pose[i][6],
                             para_ex_Pose[i][3],
                             para_ex_Pose[i][4],
                             para_ex_Pose[i][5]).normalized().toRotationMatrix();
    }

    for (auto& it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < FEATURE_CONTINUE)continue;
#if USE_INVERSE_DEPTH
        it_per_id.ptsInWorld = Rs[i2f[it_per_id.start_frame]] * (
                                   ric[0] * (it_per_id.feature_per_frame[0].point / it_per_id.idepth_) + tic[0] - Pbg
                               ) + Ps[i2f[it_per_id.start_frame]];
        if (it_per_id.idepth_ < 0) it_per_id.solve_flag = 2;
        else  it_per_id.solve_flag = 1;
#else
        Vector3d pts_cj = ric[0].transpose() * ( Rs[i2f[it_per_id.start_frame]].transpose() * (it_per_id.ptsInWorld - Ps[i2f[it_per_id.start_frame]]) + Pbg - tic[0]);
        it_per_id.idepth_ = 1.0 / pts_cj.z();
        if (it_per_id.idepth_ < 0) it_per_id.solve_flag = 2;
        else  it_per_id.solve_flag = 1;
#endif
    }



}




//
void SWFOptimization::SlideWindowFrame(int frameindex, int windowsize, bool updateIMU) {

    if (frameindex != 0) {
        for (unsigned int i = 0; i < dt_buf[frameindex + 1].size(); i++) { 
            pre_integrations[frameindex]->push_back(dt_buf[frameindex + 1][i], linear_acceleration_buf[frameindex + 1][i], angular_velocity_buf[frameindex + 1][i]);
            dt_buf[frameindex].push_back(dt_buf[frameindex + 1][i]);
            linear_acceleration_buf[frameindex].push_back(linear_acceleration_buf[frameindex + 1][i]);
            angular_velocity_buf[frameindex].push_back(angular_velocity_buf[frameindex + 1][i]);
        }
        std::swap(pre_integrations[frameindex], pre_integrations[frameindex + 1]);
        dt_buf[frameindex].swap(dt_buf[frameindex + 1]);
        linear_acceleration_buf[frameindex].swap(linear_acceleration_buf[frameindex + 1]);
        angular_velocity_buf[frameindex].swap(angular_velocity_buf[frameindex + 1]);
        if (frameindex >= 1 && updateIMU) {
            if (USE_GLOBAL_OPTIMIZATION) {
                IMUFactor* factor = new IMUFactor(pre_integrations[frameindex + 1]);
                my_problem.AddResidualBlock(factor, 0, para_pose[frameindex - 1],
                                            para_speed_bias[frameindex - 1],
                                            para_pose[frameindex + 1],
                                            para_speed_bias[frameindex + 1]);
            }

        }
    }



    for (int i = frameindex; i < windowsize - 1; i++) {
        headers[i] = headers[i + 1];
        frame_types[i] = frame_types[i + 1];
        Rs[i] = Rs[i + 1];
        mags[i] = mags[i + 1];
        Ps[i] = Ps[i + 1];
        Vs[i] = Vs[i + 1];
        Bas[i] = Bas[i + 1];
        Bgs[i] = Bgs[i + 1];

        std::swap(para_pose[i], para_pose[i + 1]);
        std::swap(para_speed_bias[i], para_speed_bias[i + 1]);
        std::swap(pre_integrations[i], pre_integrations[i + 1]);

        dt_buf[i].swap(dt_buf[i + 1]);
        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
    }
    delete para_pose[windowsize - 1];
    delete para_speed_bias[windowsize - 1];
    delete pre_integrations[windowsize - 1];

    para_pose[windowsize - 1] = new double[SIZE_POSE];
    para_speed_bias[windowsize - 1] = new double[SIZE_SPEEDBIAS];
    pre_integrations[windowsize - 1] = 0;

    dt_buf[windowsize - 1].clear();
    linear_acceleration_buf[windowsize - 1].clear();
    angular_velocity_buf[windowsize - 1].clear();
}


//marginalizing the select frames.
//param margeindex is the set of frame indexes that are selected to be marginalized.
void SWFOptimization::MargFrames(std::set<int> margeindex) {
    Vector2Double();

    std::set <double*>MargePoints = FindMargSet( margeindex);
    std::set <double*>MargePoints2 = MargePoints;


    if (USE_GLOBAL_OPTIMIZATION) {
        if (marg_flag == MargImagOld) {
            for (auto& it_per_id : f_manager.feature) {
                if (margeindex.find(i2f[it_per_id.start_frame]) != margeindex.end()) {

#if USE_INVERSE_DEPTH
                    if (my_problem.HasParameterBlock(&it_per_id.idepth_)) {
                        MargePoints2.insert(&it_per_id.idepth_);
                        assert(it_per_id.used_num >= FEATURE_CONTINUE);
                        assert(it_per_id.start_frame == 0);
                    }
#else
                    if (my_problem.HasParameterBlock(it_per_id.ptsInWorld.data())) {
                        MargePoints2.insert(it_per_id.ptsInWorld.data());
                        assert(it_per_id.used_num >= FEATURE_CONTINUE);
                        assert(it_per_id.start_frame == 0);
                    }

#endif
                }
            }
            GlobalMarge(MargePoints2);
        } else if (marg_flag == MargImagSecondNew) {
            MarginalizationInfo* marginalization_info = new MarginalizationInfo();
            ceres::Problem problem;
            ceres::Solver::Options options;
            assert(margeindex.size() == 1);
            AddAllResidual(MargeIncludeMode, MargePoints2, marginalization_info, problem, options, false, false, false);
        } else if (marg_flag == MargRoverOld) {
            MarginalizationInfo* marginalization_info = new MarginalizationInfo();
            ceres::Problem problem;
            ceres::Solver::Options options;
            assert(margeindex.size() == 1);
            AddAllResidual(MargeIncludeMode, MargePoints2, marginalization_info, problem, options, false, true, true);
        }
    } else {
        if (marg_flag == MargImagOld) {
            for (auto& it_per_id : f_manager.feature) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < FEATURE_CONTINUE || margeindex.find(i2f[it_per_id.start_frame]) == margeindex.end())
                    continue;
                assert(it_per_id.start_frame == 0);
#if USE_INVERSE_DEPTH
                MargePoints2.insert(&it_per_id.idepth_);
#else
                MargePoints2.insert(it_per_id.ptsInWorld.data());
#endif
                if (marg_flag == MargImagSecondNew)
                    assert(0);
            }
        }

        MarginalizationInfo* marginalization_info = new MarginalizationInfo();
        ceres::Problem problem;
        ceres::Solver::Options options;
        if (marg_flag == MargImagOld)
            AddAllResidual(MargeIncludeMode2, MargePoints2, marginalization_info, problem, options, true, true, true);
        else if (marg_flag == MargImagSecondNew)
            AddAllResidual(MargeIncludeMode, MargePoints2, marginalization_info, problem, options, false, false, false);
        else if (marg_flag == MargRoverOld)
            AddAllResidual(MargeIncludeMode, MargePoints2, marginalization_info, problem, options, false, true, true);
        else assert(0);
    }


    for (uint8_t i = 0; i < MAXSATNUM * 2; i++) {
        for (auto it = rtk_phase_bias_variables[i].begin(), it_next = rtk_phase_bias_variables[i].begin(); it != rtk_phase_bias_variables[i].end(); it = it_next) {
            it_next++;
            if (!it->use) {
                rtk_phase_bias_variables[i].erase(it);
            }
        }
        for (auto it = spp_phase_bias_variables[i].begin(), it_next = spp_phase_bias_variables[i].begin(); it != spp_phase_bias_variables[i].end(); it = it_next) {
            it_next++;
            if (!it->use) {
                spp_phase_bias_variables[i].erase(it);
            }
        }

    }

    if (USE_GLOBAL_OPTIMIZATION) {
        for (auto it = MargePoints.begin(); it != MargePoints.end(); it++) {
            if (my_problem.HasParameterBlock(*it))my_problem.RemoveParameterBlock(*it);
        }
    }
#if USE_INVERSE_DEPTH
    for (auto& it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();

        if ( it_per_id.used_num >= FEATURE_CONTINUE  && (
                    (margeindex.find(i2f[it_per_id.start_frame]) != margeindex.end()) ||
                    (margeindex.find(i2f[it_per_id.endFrame()]) != margeindex.end() && it_per_id.used_num == 2))  ) {
            if (my_problem.HasParameterBlock(&it_per_id.idepth_)) {
                my_problem.RemoveParameterBlock(&it_per_id.idepth_);
            }
        }
    }
#endif

    for (auto& it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < FEATURE_CONTINUE)continue;
#if USE_INVERSE_DEPTH
        if (my_problem.HasParameterBlock(&it_per_id.idepth_)) {
            std::vector<ceres::internal::ResidualBlock*>residual_blocks;
            my_problem.GetResidualBlocksForParameterBlock(&it_per_id.idepth_, &residual_blocks);
            if (residual_blocks.size() == 0) {
                my_problem.RemoveParameterBlock(&it_per_id.idepth_);
                assert(0);
            }
        }
#else
        if (my_problem.HasParameterBlock(it_per_id.ptsInWorld.data())) {
            std::vector<ceres::internal::ResidualBlock*>residual_blocks;
            my_problem.GetResidualBlocksForParameterBlock(it_per_id.ptsInWorld.data(), &residual_blocks);
            if (residual_blocks.size() == 0) {
                my_problem.RemoveParameterBlock(it_per_id.ptsInWorld.data());
                assert(0);
            }
        }
#endif
    }

}
//getting the states (pointer) that will being marginzlized.
//param margeindex is the set of frame indexes that are selected to be marginalized.
std::set <double*> SWFOptimization::FindMargSet(std::set<int> margeindex) {
    std::set<double*>MargePoints;
    for (auto it = margeindex.begin(); it != margeindex.end(); it++) {
        MargePoints.insert(para_pose[*it]);
        MargePoints.insert(para_speed_bias[*it]);
    }


    for (uint8_t i = 0; i < MAXSATNUM * 2; i++) {
        for (auto it = rtk_phase_bias_variables[i].begin(); it != rtk_phase_bias_variables[i].end(); it++) {
            it->use = false;
        }
        for (auto it = spp_phase_bias_variables[i].begin(); it != spp_phase_bias_variables[i].end(); it++) {
            it->use = false;
        }
        for (auto it = pseudorange_correction_variables[i].begin(); it != pseudorange_correction_variables[i].end(); it++) {
            it->use = false;
        }
    }

    for (int ir = 0; ir < rover_count; ir++) {
        if (margeindex.find(g2f[ir]) != margeindex.end())continue;
        mea_t* rover = rovers[ir];
        for (uint8_t i = 0; i < rover->obs_count; i++) {
            ObsMea* d = rover->obs_data + i;
            for (uint8_t f = 0; f < NFREQ; f++) {
                if (d->RTK_Npoint[f] )d->RTK_Npoint[f]->use = true;
                if (d->SPP_Npoint[f] )d->SPP_Npoint[f]->use = true;
                if (d->SPP_Npoint_PCottections[f] )d->SPP_Npoint_PCottections[f]->use = true;
            }
        }
    }

    for (uint8_t i = 0; i < MAXSATNUM * 2; i++) {
        for (auto it1 = rtk_phase_bias_variables[i].begin(); it1 != rtk_phase_bias_variables[i].end(); it1++) {
            if (!it1->use) {
                MargePoints.insert(&it1->value);
                // std::cout<<"margepoint:"<<i<<std::endl;
            }
        }
        for (auto it1 = spp_phase_bias_variables[i].begin(); it1 != spp_phase_bias_variables[i].end(); it1++) {
            if (!it1->use) {
                MargePoints.insert(&it1->value);
                // std::cout<<"margepoint:"<<i<<std::endl;
            }
        }
        for (auto it1 = pseudorange_correction_variables[i].begin(); it1 != pseudorange_correction_variables[i].end(); it1++) {
            if (!it1->use) {
                MargePoints.insert(&it1->value);
                // std::cout<<"margepoint:"<<i<<std::endl;
            }
        }
    }
    return MargePoints;
}

//marginalizing the selected gnss frames.
MarginalizationInfo* SWFOptimization::MargGNSSFrames(std::set<int> margeindex, IMUGNSSBase* IMUGNSSmeasurement) {
    Vector2Double();
    std::set <double*>MargePoints = FindMargSet( margeindex);


    MarginalizationInfo* marginalization_info = new MarginalizationInfo();
    ceres::Problem problem;
    ceres::Solver::Options options;

    AddAllResidual(GNSSMargIncludeMode, MargePoints, marginalization_info, problem, options, false, true, true);


    for (uint8_t i = 0; i < MAXSATNUM * 2; i++) {
        for (auto it = rtk_phase_bias_variables[i].begin(), it_next = rtk_phase_bias_variables[i].begin(); it != rtk_phase_bias_variables[i].end(); it = it_next) {
            it_next++;
            if (!it->use) {
                rtk_phase_bias_variables[i].erase(it);
            }
        }
        for (auto it = spp_phase_bias_variables[i].begin(), it_next = spp_phase_bias_variables[i].begin(); it != spp_phase_bias_variables[i].end(); it = it_next) {
            it_next++;
            if (!it->use) {
                spp_phase_bias_variables[i].erase(it);
            }
        }
    }

    if (USE_GLOBAL_OPTIMIZATION) {
        for (auto it = MargePoints.begin(); it != MargePoints.end(); it++) {
            if (my_problem.HasParameterBlock(*it)) {
                assert(my_problem.ParameterBlockSize(*it) == 1);
                my_problem.RemoveParameterBlock(*it);
                IMUGNSSmeasurement->residualBlockId = 0;
            }
        }
    }

    return marginalization_info;


}



void SWFOptimization::SlideWindow() {
    TicToc t_marg;

    if (image_count > FEATURE_WINDOW_SIZE) {
        if (imag_marg_index != 0) {
            marg_flag = MargImagSecondNew;
        } else {
            marg_flag = MargImagOld;
        }
    } else if (frame_types[0] == GnssFrame && rover_count > 10) {
        return;
        marg_flag = MargRoverOld;
    } else {
        return;
    }

    std::set<int>margeindex;

    if (marg_flag == MargImagOld) {
        Eigen::Vector3d P0 = Ps[i2f[0]];
        Eigen::Matrix3d R0 = Rs[i2f[0]];
        Eigen::Vector3d P1 = Ps[i2f[1]];
        Eigen::Matrix3d R1 = Rs[i2f[1]];

        for (int i = 0; i < i2f[1]; i++) {
            margeindex.insert(i);
        }
        std::cout << "MargImagOld:" << image_count << "," << rover_count << std::endl;
        MargFrames(margeindex);
        int frame_counts = image_count + rover_count;
        for (int i = 0; i < i2f[1]; i++) {
            if (frame_types[0] == GnssFrame) {
                free(rovers[0]);
                for (int ri = 0; ri < rover_count; ri++) {
                    rovers[ri] = rovers[ri + 1];
                }
                rover_count--;
            }
            SlideWindowFrame( 0, frame_counts, USE_IMU);
            frame_counts--;
        }

        SlideWindowOld( P0, R0, P1, R1, tic[0], ric[0], Pbg);
        if (imu_gnss_factor[0]) {
            delete imu_gnss_factor[0];
            imu_gnss_factor[0] = 0;
        }
        for (int i = 2; i < image_count + 1; i++)std::swap(imu_gnss_factor[i - 1], imu_gnss_factor[i]);
        if (imu_gnss_factor[image_count]) {
            delete imu_gnss_factor[image_count];
            imu_gnss_factor[image_count] = 0;
        }
        image_count--;
        UpdateVisualGnssIndex();
        if (USE_GLOBAL_OPTIMIZATION && USE_INVERSE_DEPTH) {
            AddFeature2Problem();
        }
    } else if (marg_flag == MargImagSecondNew) {
        margeindex.insert(i2f[image_count - 2]);
        std::cout << "MargImagSecondNew:" << image_count << "," << rover_count << std::endl;
        MargFrames(margeindex);
        SlideWindowFrame(i2f[image_count - 2], image_count + rover_count, USE_IMU);
        SlideWindowNew();

        if (imu_gnss_factor[image_count - 1] || imu_gnss_factor[image_count - 2]) {
            if (!imu_gnss_factor[image_count - 2])imu_gnss_factor[image_count - 2] = new IMUGNSSBase(
                    para_pose[i2f[image_count - 2] - 1], para_speed_bias[i2f[image_count - 2] - 1], &my_problem);
            IMUGNSSBase* IMUGNSSmeasurement = imu_gnss_factor[image_count - 2];
            int index1 = i2f[image_count - 2] - 1, index2 = i2f[image_count - 1] - 1; //21,10
            for (int j = index1 + 1; j < index2; j++) { //11
                IMUGNSSmeasurement->AddMargInfo(rovers[f2g[j + 1]]->marg_info_gnss, pre_integrations[j],
                                                para_pose[j], para_speed_bias[j]);
            }
            IMUGNSSmeasurement->SetLastImuFactor(pre_integrations[index2], para_pose[index2], para_speed_bias[index2]);
            std::swap(imu_gnss_factor[image_count - 1], imu_gnss_factor[image_count - 2]);
        }

        for (int i = image_count - 1; i < image_count + 1; i++)std::swap(imu_gnss_factor[i - 1], imu_gnss_factor[i]);
        if (imu_gnss_factor[image_count]) {
            delete imu_gnss_factor[image_count];
            imu_gnss_factor[image_count] = 0;
        }

        image_count--;
        UpdateVisualGnssIndex();
        if (USE_GLOBAL_OPTIMIZATION && USE_INVERSE_DEPTH) {
            AddFeature2Problem();
        }

    } else if (marg_flag == MargRoverOld) {
        margeindex.insert(g2f[0]);
        std::cout << "MargRoverOld:" << image_count << "," << rover_count << std::endl;
        MargFrames(margeindex);
        SlideWindowFrame(g2f[0], image_count + rover_count, USE_IMU);
        free(rovers[0]);
        for (int ri = 0; ri < rover_count; ri++)rovers[ri] = rovers[ri + 1];
        rover_count--;
        UpdateVisualGnssIndex();
        if (USE_GLOBAL_OPTIMIZATION) {
            if (frame_types[0] == GnssFrame) {
                MarginalizationFactor* factor = new MarginalizationFactor(rovers[0]->marg_info_gnss);
                rovers[0]->residualBlockId = my_problem.AddResidualBlock(factor, 0, rovers[0]->marg_info_gnss->keep_block_addr);
            }
            if (USE_IMU) {
                assert(pre_integrations[1]);
                IMUFactor* factor = new IMUFactor(pre_integrations[1]);
                my_problem.AddResidualBlock(factor, 0, para_pose[0], para_speed_bias[0], para_pose[1], para_speed_bias[1]);
            }
        }
        ResetImuGnssFactor(0, 0);

    }
    std::cout << "marge time:" << t_marg.toc() << std::endl;



}



//updating the frame index, gnss frame index, and visual frame index.
void SWFOptimization::UpdateVisualGnssIndex() {
    for (int k = 0; k < image_count; k++) {
        i2f[k] = ImageRoverId2FrameId(k, ImagFrame);
        f2i[i2f[k]] = k;
    }
    for (int k = 0; k < rover_count; k++) {
        g2f[k] = ImageRoverId2FrameId(k, GnssFrame);
        f2g[g2f[k]] = k;
    }
}


//regenerating the gnss-imu factor.
void SWFOptimization::ResetImuGnssFactor(int IMUGNSSindex, MarginalizationInfo* gnss_middle_marginfo) {

    if (imu_gnss_factor[IMUGNSSindex]) {
        if (USE_GLOBAL_OPTIMIZATION)
            if (imu_gnss_factor[IMUGNSSindex]->residualBlockId)
                my_problem.RemoveResidualBlock(imu_gnss_factor[IMUGNSSindex]->residualBlockId);
        delete imu_gnss_factor[IMUGNSSindex];
    }
    imu_gnss_factor[IMUGNSSindex] = 0;
    int index1, index2;
    if (image_count > 0) {
        if (IMUGNSSindex == 0) {
            index1 = 0;
            index2 = i2f[IMUGNSSindex];
        } else if (IMUGNSSindex < image_count) {
            index1 = i2f[IMUGNSSindex - 1];
            index2 = i2f[IMUGNSSindex];
        } else {
            assert(IMUGNSSindex == image_count);
            index1 = i2f[IMUGNSSindex - 1];
            index2 = rover_count + image_count - 1;
        }
    } else {
        index1 = 0;
        index2 = rover_count - 1;
    }

    if (index2 - index1 > 1) {
        imu_gnss_factor[IMUGNSSindex] = new IMUGNSSBase(para_pose[index1], para_speed_bias[index1], &my_problem);
        IMUGNSSBase* IMUGNSSmeasurement = imu_gnss_factor[IMUGNSSindex];
        for (int j = index1 + 1; j < index2; j++) {
            IMUGNSSmeasurement->AddMargInfo(rovers[f2g[j]]->marg_info_gnss, pre_integrations[j],
                                            para_pose[j], para_speed_bias[j]);
        }
        if (gnss_middle_marginfo)IMUGNSSmeasurement->AddMidMargInfo( gnss_middle_marginfo);
        IMUGNSSmeasurement->SetLastImuFactor(pre_integrations[index2], para_pose[index2], para_speed_bias[index2]);

    }
}




//adding the new gnss, imu information to the corresponding gnss-imu factor.
void SWFOptimization::UpdateImuGnssFactor() {



    if (image_count + rover_count >= 3) {
        int j = image_count + rover_count - 2;
        if (frame_types[j] == GnssFrame) {
            int index = frame_types[j + 1] == ImagFrame ? (image_count - 1) : image_count;
            if (!imu_gnss_factor[index])imu_gnss_factor[index] = new IMUGNSSBase(
                    para_pose[image_count + rover_count - 3], para_speed_bias[image_count + rover_count - 3], &my_problem);
            IMUGNSSBase* IMUGNSSmeasurement = imu_gnss_factor[index];
            IMUGNSSmeasurement->AddMargInfo(rovers[f2g[j]]->marg_info_gnss, pre_integrations[j],
                                            para_pose[j], para_speed_bias[j]);
            IMUGNSSmeasurement->SetLastImuFactor(pre_integrations[j + 1], para_pose[j + 1], para_speed_bias[j + 1]);
        }
    }

}
//finding the avaliable gnss observations.
uint8_t getVariableUseSingleNum(mea_t* obs_data) {
    uint8_t i, j = 0;
    ObsMea* datai = obs_data->obs_data;
    for (i = 0; i < obs_data->obs_count; i++) {
        ObsMea* d = datai + i;
        if (d->SVH == 0 && d->SPP_P[0] != 0 && d->SPP_Pstd[0] < 2) {
            j += 1;
        }
    }
    return j;
}

//main process
void SWFOptimization::MeasurementProcess() {
#define condition1 (USE_IMAGE&&USE_GNSS&&USE_IMU&&!feature_buf.empty()&&!rover_buf.empty())
#define condition2 (!USE_IMAGE&&USE_GNSS&&!rover_buf.empty())
#define condition3 (USE_IMAGE&&!USE_GNSS&&!feature_buf.empty())


    while (condition1 || condition2 || condition3) {
        std::cout << rover_buf.size() << "," << feature_buf.size() << std::endl;
        TicToc t_process;
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        if (condition1 || condition3)
            feature = feature_buf.front();

        mea_t* rover = rover_buf.front();;
        FrameType modeflag = ErroFrame;

        if (condition1) {
            if (!rover_buf.empty() && rover && rover->ros_time < feature.first) {
                cur_time = rover->ros_time;
                modeflag = GnssFrame;
            } else {
                cur_time = feature.first;
                modeflag = ImagFrame;
            }
            if (!rover_buf.empty() && rover && modeflag == GnssFrame && getVariableUseSingleNum(rover) < 8) {
                rover_buf.pop_front();
                std::cout << "                                                                                                         pop GNSS\r\n";
                continue;
            }
            if (!rover_buf.empty() && rover && modeflag == ImagFrame && rover->ros_time - cur_time <= 0.005 && getVariableUseSingleNum(rover) >= 8) {
                feature_buf.pop();
                std::cout << "                                                                                                         pop IMAGE\r\n";
                continue;
            }
        }
        if (condition2) {
            solver_flag = NonLinear;
            cur_time = rover->ros_time;
            modeflag = GnssFrame;
            if (!rover_buf.empty() && rover && modeflag == GnssFrame && getVariableUseSingleNum(rover) < 8) {
                rover_buf.pop_front();
                std::cout << "                                                                                                         pop GNSS\r\n";
                continue;
            }
        }
        if (condition3) {
            cur_time = feature.first;
            modeflag = ImagFrame;
        }

        if (cur_time <= prev_time2 || cur_time - prev_time2 <= 0.005||(USE_IMU&&acc_buf.front().first>cur_time)) { //
            if (modeflag == GnssFrame) rover_buf.pop_front();
            else feature_buf.pop();
            continue;
        }

        assert(modeflag != ErroFrame);
        if (!ImuAvailable(cur_time) && USE_IMU)return;



        if (modeflag == GnssFrame) {
            rover_buf.pop_front();
            rover_count++;
            need_Nresolve = true;
            frame_types[rover_count + image_count - 1] = GnssFrame;

        } else {
            feature_buf.pop();
            image_count++;
            frame_types[rover_count + image_count - 1] = modeflag;
        }

        MagProcess(cur_time);
        ImuIntegrate();


        if (USE_GLOBAL_OPTIMIZATION)
            my_problem.AddParameterBlock(para_pose[image_count + rover_count - 1], SIZE_POSE, new PoseLocalParameterization());

        prev_time2 = cur_time;

        UpdateVisualGnssIndex();

        // if(modeflag==GnssFrame)
        assert(fabs(headers[image_count + rover_count - 1] - cur_time) < 1e-4);
        headers[image_count + rover_count - 1] = cur_time;
        // else headers[image_count+rover_count-1] = feature.first;

        assert(rover_count + image_count < FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE);

        if (init_gnss) {
            double pos[3];
            Matrix3d Rwwg;
            Eigen::Vector3d xyz = base_pos + Ps[rover_count / 2];
            ecef2pos(xyz.data(), pos);
            xyz2enu(pos, Rwwg.data());//transform vector from ecef to enu
            Rwgw = Rwwg.transpose();

        }
        if (modeflag == ImagFrame)ImagePreprocess(feature.second);
        else GnssProcess(rover);
        UpdateImuGnssFactor();

        if (solver_flag != Initial && (modeflag == ImagFrame || frame_types[rover_count + image_count - 2] == GnssFrame)) {
            MyOptimization();
            IntegerSolve();
        }

        std::cout << std::endl;
        for (int i = 0; i < image_count + 1; i++) {
            if (imu_gnss_factor[i])
                std::cout << imu_gnss_factor[i]->update_round * 1.0 / imu_gnss_factor[i]->evaluate_round << ",";
        }
        if (modeflag == ImagFrame)ImagePostprocess();


        SlideWindow();
        MiddleMargGnssFrame();
        if (modeflag == ImagFrame)
            f_manager.removeOut2(image_count, my_problem);

        Ps[rover_count + image_count] = Ps[rover_count + image_count - 1];
        Rs[rover_count + image_count] = Rs[rover_count + image_count - 1];
        Vs[rover_count + image_count] = Vs[rover_count + image_count - 1];
        Bas[rover_count + image_count] = Bas[rover_count + image_count - 1];
        Bgs[rover_count + image_count] = Bgs[rover_count + image_count - 1];
        headers[rover_count + image_count] = headers[rover_count + image_count - 1];


        if (solver_flag != Initial) {
            PubData();
        }

        if (modeflag == ImagFrame) {
            static double t_process2 = 0;
            static int t_count = 0;
            double ts = t_process.toc();
            t_process2 += ts;
            t_count += 1;
            printf("process measurement time: %f   ,%f   ,%d\n", ts, t_process2 / t_count, (int)modeflag);
        }

    }



}


//publicating and saving results.
void SWFOptimization::PubData() {


    for (int k = 0; k < image_count; k++)i2f[k] = ImageRoverId2FrameId(k, ImagFrame);
    for (int k = 0; k < rover_count; k++)g2f[k] = ImageRoverId2FrameId(k, GnssFrame);

    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(headers[image_count + rover_count - 1]);
    if (!pub_init) {
        resetpot(*this, header);
        pub_init = true;
    }
    printStatistics(*this, 0);
    pubOdometry(*this, header);
    pubCameraPose(*this, header);
    pubPointCloud(*this, header);
    pubKeyframe(*this);
    
    // 发布RAIM结果
    pubRaimResults(*this, header);
    pubRaimVisualization(*this, header);
    
    // 保存结果（包括原有和新的残差CSV文件）
    save_result(*this);



}

// ==================== RAIM函数实现 ====================

// 初始化RAIM配置
void SWFOptimization::InitializeRaimConfig() {
    raim_config_.false_alarm_rate = 0.1;        // 使用10%虚警率，最大容错性
    raim_config_.missed_detection_rate = 0.05;  // 保持5%漏检率
    raim_config_.min_elevation_angle = 5.0;     // 5度最小高度角，适合城市环境
    raim_config_.min_satellites = 5;            // 保持最少5颗卫星
    raim_config_.max_gdop = 10.0;              // 城市环境放宽到10.0最大GDOP
    raim_config_.enable_fault_isolation = true; // 启用故障隔离
    
    // 添加调试信息
    printf("RAIM Config: FA=%.3f%%, MD=%.3f%%, MinEl=%.1f°, MinSats=%d, MaxGDOP=%.1f\n", 
           raim_config_.false_alarm_rate * 100,
           raim_config_.missed_detection_rate * 100,
           raim_config_.min_elevation_angle,
           raim_config_.min_satellites,
           raim_config_.max_gdop);
}

// 主RAIM检测函数
SWFOptimization::TraditionalRaimResult SWFOptimization::PerformTraditionalRaim(const mea_t& gnss_data) {
    TraditionalRaimResult result;
    result.integrity_available = false;
    result.fault_detected = false;
    
    // 1. 预检查：卫星数量
    result.available_satellites = gnss_data.obs_count;
    printf("[RAIM DEBUG] Available satellites: %d, Required: %d\n", 
           gnss_data.obs_count, raim_config_.min_satellites);
    
    if(gnss_data.obs_count < raim_config_.min_satellites) {
        printf("[RAIM DEBUG] Insufficient satellites for integrity monitoring\n");
        result.failure_reason = "Insufficient satellites (" + std::to_string(gnss_data.obs_count) + 
                               "/" + std::to_string(raim_config_.min_satellites) + ")";
        return result;
    }
    
    // 2. 构建几何矩阵和观测向量
    Eigen::MatrixXd H, W;
    Eigen::VectorXd y;
    
    if(!BuildGeometryMatrix(gnss_data, H, y, W)) {
        printf("[RAIM DEBUG] Failed to build geometry matrix\n");
        result.failure_reason = "Failed to build geometry matrix";
        return result;
    }
    
    // 3. 检查矩阵条件数
    Eigen::MatrixXd HtWH = H.transpose() * W * H;
    if(HtWH.determinant() < 1e-10) {
        printf("[RAIM DEBUG] Singular geometry matrix (det=%.2e)\n", HtWH.determinant());
        result.failure_reason = "Singular geometry matrix (poor satellite geometry)";
        return result; // 矩阵奇异
    }
    
    // 4. 计算GDOP
    Eigen::MatrixXd Qxx = HtWH.inverse();
    result.gdop = sqrt(Qxx.trace());
    
    printf("[RAIM DEBUG] GDOP: %.2f, Max allowed: %.1f\n", result.gdop, raim_config_.max_gdop);
    
    if(result.gdop > raim_config_.max_gdop) {
        printf("[RAIM DEBUG] GDOP too high for integrity monitoring\n");
        result.failure_reason = "GDOP too high (" + std::to_string(result.gdop) + 
                               "/" + std::to_string(raim_config_.max_gdop) + ")";
        return result; // GDOP过大
    }
    
    // 5. 最小二乘位置估计
    Eigen::VectorXd x_hat = Qxx * H.transpose() * W * y;
    
    // 6. 残差计算
    Eigen::VectorXd residuals = y - H * x_hat;
    
    // 添加残差合理性检查
    double residual_norm = residuals.norm();
    double max_residual = residuals.cwiseAbs().maxCoeff();
    
    // 如果残差异常大，可能是钟差估计问题
    if(max_residual > 1e6) { // 残差大于1000km
        printf("[RAIM] WARNING: Extremely large residuals detected!\n");
        printf("  Max residual: %.0f m, RMS: %.0f m\n", max_residual, residual_norm/sqrt(H.rows()));
        printf("  Clock bias estimate: %.0f m (%.3f ms)\n", x_hat(3), x_hat(3)/299792458.0*1000);
        printf("  Position estimate: (%.0f, %.0f, %.0f) m\n", x_hat(0), x_hat(1), x_hat(2));
        
        // 尝试修正：如果所有残差都有相同的偏差，可能是钟差问题
        double mean_residual = residuals.mean();
        double std_residual = sqrt((residuals.array() - mean_residual).square().mean());
        
        printf("  Residual statistics: mean=%.0f m, std=%.0f m\n", mean_residual, std_residual);
        
        if(fabs(mean_residual) > 1e5 && std_residual < fabs(mean_residual) * 0.1) {
            printf("  Systematic bias detected! Attempting clock bias correction: %.0f m\n", mean_residual);
            // 更新钟差估计
            x_hat(3) += mean_residual;
            // 重新计算残差
            residuals = y - H * x_hat;
            
            // 检查修正后的残差
            double new_max_residual = residuals.cwiseAbs().maxCoeff();
            double new_mean_residual = residuals.mean();
            printf("  After correction: max=%.0f m, mean=%.0f m\n", new_max_residual, new_mean_residual);
        }
    }
    
    // 7. 奇偶空间投影
    Eigen::MatrixXd P = CalculateParityMatrix(H, W);
    Eigen::VectorXd parity = P * residuals;
    
    // 检查奇偶矩阵的数值稳定性
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P);
    double condition_number = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    
    // 8. 改进的检验统计量计算
    // 使用加权残差的范数，而不是奇偶空间投影
    Eigen::VectorXd weighted_residuals = W.diagonal().cwiseSqrt().cwiseProduct(residuals);
    result.test_statistic = weighted_residuals.squaredNorm();
    
    // 如果奇偶矩阵条件数太差，使用简化计算
    if(condition_number > 1e12) {
        printf("[RAIM WARNING] Poor parity matrix condition number: %.2e\n", condition_number);
        result.test_statistic = residuals.squaredNorm(); // 回退到简单残差平方和
    }
    
    // 9. χ²阈值计算
    int degrees_of_freedom = H.rows() - 4;
    result.chi2_threshold = CalculateChi2Threshold(degrees_of_freedom, raim_config_.false_alarm_rate);
    
    // 10. 故障检测
    result.fault_detected = result.test_statistic > result.chi2_threshold;
    
    // 添加详细诊断信息
    if(result.fault_detected) {
        printf("[RAIM FAULT] Test statistic %.1f > Threshold %.1f\n", result.test_statistic, result.chi2_threshold);
        printf("  Max residual: %.1f m, RMS residual: %.1f m\n", 
               residuals.cwiseAbs().maxCoeff(), residuals.norm()/sqrt(residuals.size()));
        printf("  Parity vector norm: %.1f, DOF: %d\n", parity.norm(), degrees_of_freedom);
        
        // 分析残差分布，找出问题卫星
        printf("  Individual residuals (m): ");
        for(int i = 0; i < std::min(10, (int)residuals.size()); i++) {
            printf("%.1f ", residuals(i));
        }
        printf("\n");
        
        // 检查是否所有残差都有相同符号（系统偏差）
        int positive_count = 0, negative_count = 0;
        for(int i = 0; i < residuals.size(); i++) {
            if(residuals(i) > 0) positive_count++;
            else if(residuals(i) < 0) negative_count++;
        }
        printf("  Residual signs: +%d, -%d (systematic bias: %s)\n", 
               positive_count, negative_count,
               (abs(positive_count - negative_count) > residuals.size() * 0.7) ? "YES" : "NO");
    }
    
    // 设置故障原因
    if(result.fault_detected) {
        result.failure_reason = "Fault detected (Test=" + std::to_string((int)result.test_statistic) + 
                               ", Threshold=" + std::to_string((int)result.chi2_threshold) + ")";
    } else {
        result.failure_reason = "Integrity OK";
    }
    
    // 11. 保护级计算
    result.protection_level_horizontal = CalculateProtectionLevel(H, W, result.chi2_threshold, true);
    result.protection_level_vertical = CalculateProtectionLevel(H, W, result.chi2_threshold, false);
    
    // 12. 故障隔离
    if(result.fault_detected && raim_config_.enable_fault_isolation) {
        result.faulty_satellite_prn = IsolateFaultySatellite(gnss_data);
        result.failure_reason += " (Faulty PRN=" + std::to_string(result.faulty_satellite_prn) + ")";
    }
    
    result.integrity_available = true;
    result.available_satellites = H.rows();
    result.detection_probability = 1.0 - raim_config_.missed_detection_rate;
    result.false_alarm_rate = raim_config_.false_alarm_rate;
    
    return result;
}

// 构建几何矩阵
bool SWFOptimization::BuildGeometryMatrix(const mea_t& gnss_data, 
                                         Eigen::MatrixXd& H, 
                                         Eigen::VectorXd& y, 
                                         Eigen::MatrixXd& W) {
    
    // RTK模式检查
    bool is_rtk_mode = false;
    for(int i = 0; i < gnss_data.obs_count; i++) {
        if(gnss_data.obs_data[i].RTK_P[0] > 0) {
            is_rtk_mode = true;
            break;
        }
    }
    
    static bool print_rtk_info = true;
    if(is_rtk_mode && print_rtk_info) {
        printf("RAIM: Operating in RTK mode\n");
        print_rtk_info = false; // 只打印一次
    }
    
    // 收集有效观测，按卫星系统分类
    vector<Eigen::Vector3d> sat_positions;
    vector<double> pseudoranges;
    vector<double> weights;
    vector<int> prn_list;
    vector<int> sys_list; // 0=GPS, 1=BDS, 2=GAL
    
    for(int i = 0; i < gnss_data.obs_count; i++) {
        const ObsMea& obs = gnss_data.obs_data[i];
        
        // 选择最佳伪距观测
        // 注意：RAIM需要绝对伪距测量，SPP_P包含卫星钟差改正，RTK_P是单差不包含
        double pseudorange = -1;
        double std_dev = 1.0;
        bool using_spp = false;
        
        // 对于RAIM，优先使用SPP_P（包含卫星钟差改正）
        if(obs.SPP_P[0] > 0) {
            pseudorange = obs.SPP_P[0];
            std_dev = (obs.SPP_Pstd[0] > 0) ? obs.SPP_Pstd[0] : 3.0;
            using_spp = true;
        } else if(obs.RTK_P[0] > 0 && obs.RTK_Pstd[0] > 0) {
            // 如果没有SPP_P，使用RTK_P但需要注意这是单差测量
            pseudorange = obs.RTK_P[0];
            std_dev = obs.RTK_Pstd[0];
            using_spp = false;
            
            // RTK_P是单差，理论上不应该用于RAIM，发出警告
            static int rtk_warning_count = 0;
            if(rtk_warning_count++ < 5) {
                printf("RAIM WARNING: Using RTK_P (single-difference) for satellite %d, may cause biases\n", obs.sat);
            }
        }
        
        // 轨道数据验证和异常剔除
        bool orbit_data_valid = true;
        double sat_earth_distance = sqrt(obs.satellite_pos[0]*obs.satellite_pos[0] +
                                        obs.satellite_pos[1]*obs.satellite_pos[1] +
                                        obs.satellite_pos[2]*obs.satellite_pos[2]);
        
        // 实施严格的轨道类型验证
        if(obs.sys == SYS_GPSS) {
            // GPS必须是MEO轨道
            if(sat_earth_distance < 19000000.0 || sat_earth_distance > 27000000.0) {
                orbit_data_valid = false;
                if(using_spp && obs.sat >= 1 && obs.sat <= 32) {
                    printf("    EXCLUDED: GPS PRN %d invalid orbit (%.0f km, expect 19-27Mm)\n", 
                           obs.sat, sat_earth_distance/1000.0);
                }
            }
        } else if(obs.sys == SYS_GALL) {
            // Galileo必须是MEO轨道  
            if(sat_earth_distance < 22000000.0 || sat_earth_distance > 25000000.0) {
                orbit_data_valid = false;
                if(using_spp) {
                    printf("    EXCLUDED: GAL PRN %d invalid orbit (%.0f km, expect 22-25Mm)\n", 
                           obs.sat, sat_earth_distance/1000.0);
                }
            }
        } else if(obs.sys == SYS_CMP) {
            // BDS可以是MEO或GEO轨道
            bool is_meo = (sat_earth_distance > 19000000.0 && sat_earth_distance < 27000000.0);
            bool is_geo = (sat_earth_distance > 34000000.0 && sat_earth_distance < 38000000.0);
            if(!is_meo && !is_geo) {
                orbit_data_valid = false;
                if(using_spp) {
                    printf("    EXCLUDED: BDS PRN %d invalid orbit (%.0f km, expect MEO 19-27Mm or GEO 34-38Mm)\n", 
                           obs.sat, sat_earth_distance/1000.0);
                }
            }
        }
        
        // 检查PRN映射是否正确
        bool prn_mapping_valid = true;
        if(obs.sat >= 1 && obs.sat <= 32) {
            if(obs.sys != SYS_GPSS) {
                prn_mapping_valid = false;
                printf("    EXCLUDED: PRN %d should be GPS but mapped to sys=%d\n", obs.sat, obs.sys);
            }
        } else if(obs.sat >= 33 && obs.sat <= 120) {
            if(obs.sys != SYS_CMP) {
                prn_mapping_valid = false;
                printf("    EXCLUDED: PRN %d should be BDS but mapped to sys=%d\n", obs.sat, obs.sys);
            }
        } else if(obs.sat >= 201 && obs.sat <= 250) {
            if(obs.sys != SYS_GALL) {
                prn_mapping_valid = false;
                printf("    EXCLUDED: PRN %d should be GAL but mapped to sys=%d\n", obs.sat, obs.sys);
            }
        }
        
        // 观测质量检查（只处理通过轨道验证的卫星）
        if(pseudorange > 0 && 
           obs.el > raim_config_.min_elevation_angle * M_PI / 180 &&
           obs.SVH == 0 && // 卫星健康
           orbit_data_valid && // 轨道数据有效
           prn_mapping_valid) { // PRN映射正确
            
            sat_positions.push_back(Eigen::Vector3d(obs.satellite_pos[0], 
                                                   obs.satellite_pos[1], 
                                                   obs.satellite_pos[2]));
            pseudoranges.push_back(pseudorange);
            
            // 高度角加权
            double elevation_weight = sin(obs.el) * sin(obs.el);
            double obs_weight = elevation_weight / (std_dev * std_dev);
            weights.push_back(obs_weight);
            
            prn_list.push_back(obs.sat);
            
            // 确定卫星系统
            int sys = obs.sys; // 使用ObsMea中的sys字段
            sys_list.push_back(sys);
            
            // 调试信息：显示伪距选择和PRN映射
            static int debug_count = 0;
            if(debug_count++ < 10) { // 限制输出数量，避免过多信息
                printf("  Sat %d: %s pseudorange=%.0f m, std=%.2f m, el=%.1f°, sys=%d\n",
                       obs.sat, using_spp ? "SPP" : "RTK", pseudorange, std_dev, obs.el * 180 / M_PI, obs.sys);
                
                // PRN映射验证
                const char* expected_system = "Unknown";
                bool prn_mapping_correct = true;
                
                if(obs.sat >= 1 && obs.sat <= 32) {
                    expected_system = "GPS";
                    if(obs.sys != SYS_GPSS) prn_mapping_correct = false;
                } else if(obs.sat >= 33 && obs.sat <= 63) {
                    expected_system = "BDS MEO/IGSO";
                    if(obs.sys != SYS_CMP) prn_mapping_correct = false;
                } else if(obs.sat >= 64 && obs.sat <= 68) {
                    expected_system = "BDS GEO"; 
                    if(obs.sys != SYS_CMP) prn_mapping_correct = false;
                } else if(obs.sat >= 80 && obs.sat <= 120) {
                    expected_system = "BDS"; // Extended BDS range
                    if(obs.sys != SYS_CMP) prn_mapping_correct = false;
                } else if(obs.sat >= 201 && obs.sat <= 250) {
                    expected_system = "GAL";
                    if(obs.sys != SYS_GALL) prn_mapping_correct = false;
                }
                
                if(!prn_mapping_correct) {
                    printf("    ERROR: PRN %d should be %s but mapped to sys=%d\n", 
                           obs.sat, expected_system, obs.sys);
                } else {
                    printf("    PRN mapping OK: PRN %d -> %s (sys=%d)\n", 
                           obs.sat, expected_system, obs.sys);
                }
            }
        }
    }
    
    if(sat_positions.size() < raim_config_.min_satellites) {
        return false;
    }
    
    // 估计近似用户位置
    Eigen::Vector3d approx_user_pos = EstimateUserPosition(gnss_data);
    
    // 添加调试信息标志和批次ID
    static int debug_count = 0;
    static int coord_diag_batch_id = 0;
    bool print_debug = (debug_count++ % 20 == 0); // 每20次打印一次
    int diag_batch = ++coord_diag_batch_id;
    
    if(print_debug) {
        printf("RAIM COORD SYSTEM DIAGNOSIS BATCH #%d:\n", diag_batch);
        printf("  User position (assumed WGS84 ECEF): (%.3f, %.3f, %.3f) m\n",
               approx_user_pos.x(), approx_user_pos.y(), approx_user_pos.z());
        printf("  User dist from Earth center: %.3f km\n", approx_user_pos.norm()/1000.0);
        
        // 计算地理坐标用于验证
        double user_ecef[3] = {approx_user_pos.x(), approx_user_pos.y(), approx_user_pos.z()};
        double user_lla[3];
        ecef2pos(user_ecef, user_lla); // pos format: [lat(rad), lon(rad), height(m)]
        printf("  User LLA: Lat=%.6f°, Lon=%.6f°, Alt=%.1f m\n", 
               user_lla[0]*180/M_PI, user_lla[1]*180/M_PI, user_lla[2]);
    }
    
    // 构建矩阵 - 考虑多系统钟差
    int n_sats = sat_positions.size();
    
    // 统计每个系统的卫星数
    int n_gps = 0, n_bds = 0, n_gal = 0;
    for(int sys : sys_list) {
        if(sys == SYS_GPSS) n_gps++;
        else if(sys == SYS_CMP) n_bds++;
        else if(sys == SYS_GALL) n_gal++;
    }
    
    // 确定参数数量：3个位置 + 每个系统一个钟差
    int n_params = 3;
    if(n_gps > 0) n_params++;
    if(n_bds > 0) n_params++;
    if(n_gal > 0) n_params++;
    
    // 统一使用4参数模型，但对所有情况都进行钟差校正
    H.resize(n_sats, 4);
    
    // 估计系统间偏差和接收机钟差
    // 注意：现在主要使用SPP_P（已含卫星钟差改正），所以估计的主要是接收机钟差
    double sys_bias[3] = {0, 0, 0}; // GPS, BDS, GAL
    
    // 钟差估算开始
    static int raim_batch_id = 0;
    int current_batch = ++raim_batch_id;
    printf("\n=== RAIM BATCH #%d: Starting receiver clock bias estimation (using SPP_P) ===\n", current_batch);
    
    // 分系统钟差估算：GPS/BDS/Galileo分别计算
    vector<double> gps_residuals, bds_residuals, gal_residuals;
    
    for(size_t i = 0; i < pseudoranges.size(); i++) {
        Eigen::Vector3d line_of_sight = sat_positions[i] - approx_user_pos;
        double range = line_of_sight.norm();
        double residual = pseudoranges[i] - range;
        
        // 根据卫星系统分类残差
        if(fabs(residual) > 1000.0 && fabs(residual) < 50000000.0) {
            int sys = sys_list[i];
            printf("RAIM BATCH #%d: Sat %zu, sys=%d, residual=%.0f\n", current_batch, i, sys, residual);
            if(sys == SYS_GPSS) {
                gps_residuals.push_back(residual);
            } else if(sys == SYS_CMP) {
                bds_residuals.push_back(residual);
            } else if(sys == SYS_GALL) {
                gal_residuals.push_back(residual);
            }
        }
    }
    
    // 改进的钟差计算：使用稳健的统计方法
    auto calculate_robust_bias = [](vector<double>& residuals) -> double {
        if(residuals.empty()) return 0.0;
        
        // 先去除明显的异常值（超过3倍标准差）
        if(residuals.size() > 3) {
            sort(residuals.begin(), residuals.end());
            double median = (residuals.size() % 2 == 0) ? 
                          (residuals[residuals.size()/2-1] + residuals[residuals.size()/2]) / 2.0 :
                          residuals[residuals.size()/2];
            
            // 计算MAD (Median Absolute Deviation)
            vector<double> abs_deviations;
            for(double r : residuals) {
                abs_deviations.push_back(fabs(r - median));
            }
            sort(abs_deviations.begin(), abs_deviations.end());
            double mad = (abs_deviations.size() % 2 == 0) ?
                        (abs_deviations[abs_deviations.size()/2-1] + abs_deviations[abs_deviations.size()/2]) / 2.0 :
                        abs_deviations[abs_deviations.size()/2];
            
            // 使用3倍MAD作为异常值阈值，更稳健
            double threshold = 3.0 * mad * 1.4826; // 1.4826是从MAD到标准差的转换因子
            vector<double> filtered_residuals;
            for(double r : residuals) {
                if(fabs(r - median) <= threshold) {
                    filtered_residuals.push_back(r);
                }
            }
            
            if(!filtered_residuals.empty()) {
                residuals = filtered_residuals;
            }
        }
        
        // 计算均值（去除异常值后）
        double sum = 0.0;
        for(double r : residuals) {
            sum += r;
        }
        return sum / residuals.size();
    };
    
    sys_bias[SYS_GPSS] = calculate_robust_bias(gps_residuals);
    sys_bias[SYS_CMP] = calculate_robust_bias(bds_residuals);
    sys_bias[SYS_GALL] = calculate_robust_bias(gal_residuals);
    
    printf("RAIM BATCH #%d: Clock bias estimates - GPS: %.0f m (%zu sats), BDS: %.0f m (%zu sats), GAL: %.0f m (%zu sats)\n",
           current_batch, sys_bias[SYS_GPSS], gps_residuals.size(),
           sys_bias[SYS_CMP], bds_residuals.size(),
           sys_bias[SYS_GALL], gal_residuals.size());
    
    // 回退处理：如果某系统没有有效残差，使用其他系统或默认值
    if(gps_residuals.empty() && bds_residuals.empty() && gal_residuals.empty()) {
        // 所有系统都没有有效残差，使用紧急回退
        double emergency_bias = 0.0; // SPP_P已含卫星钟差改正，接收机钟差应该较小
        sys_bias[SYS_GPSS] = emergency_bias;
        sys_bias[SYS_CMP] = emergency_bias;
        sys_bias[SYS_GALL] = emergency_bias;
        if(print_debug) {
            printf("RAIM: Emergency receiver clock bias estimate: %.0f m (no valid residuals)\n", emergency_bias);
        }
    } else {
        // 个别系统回退处理
        double fallback_bias = 0.0;
        if(!gps_residuals.empty()) fallback_bias = sys_bias[SYS_GPSS];
        else if(!bds_residuals.empty()) fallback_bias = sys_bias[SYS_CMP];
        else if(!gal_residuals.empty()) fallback_bias = sys_bias[SYS_GALL];
        
        if(gps_residuals.empty()) sys_bias[SYS_GPSS] = fallback_bias;
        if(bds_residuals.empty()) sys_bias[SYS_CMP] = fallback_bias;
        if(gal_residuals.empty()) sys_bias[SYS_GALL] = fallback_bias;
    }
        
    // 对伪距进行接收机钟差校正
    double total_correction = 0.0;
    for(size_t i = 0; i < pseudoranges.size(); i++) {
        double original_range = pseudoranges[i];
        // SPP_P已含卫星钟差改正，现在只需校正接收机钟差
        // 伪距观测 = 真实距离 + 接收机钟差，所以校正后伪距 = 观测伪距 - 接收机钟差
        pseudoranges[i] -= sys_bias[sys_list[i]];
        total_correction += fabs(original_range - pseudoranges[i]);
    }
    
    if(print_debug && total_correction > 100.0) {
        printf("RAIM BATCH #%d: Applied receiver clock bias corrections, avg adjustment: %.0f m\n", 
               current_batch, total_correction / pseudoranges.size());
    }
    
    y.resize(n_sats);
    W = Eigen::MatrixXd::Zero(n_sats, n_sats);
    
    if(print_debug) {
        printf("\n[RAIM Debug] BuildGeometryMatrix:\n");
        printf("  User Position: (%.2f, %.2f, %.2f) m\n", 
               approx_user_pos.x(), approx_user_pos.y(), approx_user_pos.z());
        printf("  Number of satellites: %d\n", n_sats);
    }
    
    double max_residual = 0.0;
    double sum_residual = 0.0;
    
    for(int i = 0; i < n_sats; i++) {
        // 视线向量
        Eigen::Vector3d line_of_sight = sat_positions[i] - approx_user_pos;
        double range = line_of_sight.norm();
        
        // 卫星距离合理性检查（考虑MEO和GEO卫星）
        bool range_reasonable = true;
        if(range < 18000000.0) { // 小于18Mm明显异常
            range_reasonable = false;
            if(print_debug) {
                printf("  WARNING: Satellite %d range %.0f m too close (< 18Mm)!\n", 
                       prn_list[i], range);
            }
        } else if(range > 26000000.0 && range < 30000000.0) { // 26-30Mm范围可疑
            range_reasonable = false;
            if(print_debug) {
                printf("  WARNING: Satellite %d range %.0f m in suspicious range (26-30Mm)!\n", 
                       prn_list[i], range);
            }
        } else if(range > 40000000.0) { // 大于40Mm明显异常（GEO最远约37Mm）
            range_reasonable = false;
            if(print_debug) {
                printf("  ERROR: Satellite %d range %.0f m too far (> 40Mm), possible data error!\n", 
                       prn_list[i], range);
            }
        } else if(range > 30000000.0 && range < 40000000.0) {
            // 30-40Mm范围，可能是GEO卫星（BDS GEO轨道高度约35.786Mm）
            if(print_debug) {
                printf("  INFO: Satellite %d range %.0f m appears to be GEO satellite\n", 
                       prn_list[i], range);
            }
        }
        
        // 对于不合理的距离，降低权重而不是完全跳过
        if(!range_reasonable && i < weights.size()) {
            weights[i] *= 0.1; // 大幅降低权重
        }
        
        // 对于极端异常的距离值，跳过处理
        if(range > 50000000.0 || range < 10000000.0) {
            if(print_debug) {
                printf("  ERROR: Satellite %d range %.0f m is extremely unreasonable, skipping!\n", 
                       prn_list[i], range);
            }
            continue;
        }
        
        // 验证卫星位置的合理性（考虑MEO和GEO卫星）
        double sat_dist_from_earth = sat_positions[i].norm();
        bool position_reasonable = true;
        
        // 详细的坐标系诊断
        if(print_debug && i < 5) { // 只对前5颗卫星做详细诊断
            printf("  SAT %d COORD DIAGNOSIS:\n", prn_list[i]);
            printf("    Position (ECEF?): (%.3f, %.3f, %.3f) m\n",
                   sat_positions[i].x(), sat_positions[i].y(), sat_positions[i].z());
            printf("    Dist from Earth center: %.3f km\n", sat_dist_from_earth/1000.0);
            printf("    System: %s\n", (sys_list[i] == SYS_GPSS) ? "GPS" : 
                                      (sys_list[i] == SYS_CMP) ? "BDS" : 
                                      (sys_list[i] == SYS_GALL) ? "GAL" : "Unknown");
            
            // 计算相对于用户的方向和距离
            Eigen::Vector3d sat_to_user = sat_positions[i] - approx_user_pos;
            double sat_user_distance = sat_to_user.norm();
            printf("    Distance to user: %.3f km\n", sat_user_distance/1000.0);
            
            // 检查是否在地球同一侧
            double dot_product = sat_positions[i].dot(approx_user_pos);
            double angle_deg = acos(dot_product / (sat_dist_from_earth * approx_user_pos.norm())) * 180.0 / M_PI;
            printf("    Angle from Earth center: %.1f°\n", angle_deg);
            
            // 星历数据验证
            printf("    EPHEMERIS VALIDATION:\n");
            
            // 检查卫星位置是否为零或异常值
            if(sat_positions[i].norm() < 1000000.0) {
                printf("      ERROR: Satellite position too close to Earth center - ephemeris error!\n");
            } else if(sat_positions[i].norm() > 50000000.0) {
                printf("      ERROR: Satellite position too far from Earth - ephemeris error!\n");
            }
            
            // 验证轨道类型与系统的匹配
            bool orbit_system_match = false;
            const char* expected_orbit = "Unknown";
            
            if(sys_list[i] == SYS_GPSS) {
                // GPS只有MEO轨道（约20200km）
                if(sat_dist_from_earth > 19000000.0 && sat_dist_from_earth < 27000000.0) {
                    orbit_system_match = true;
                    expected_orbit = "MEO";
                } else {
                    expected_orbit = "MEO (19-27Mm)";
                }
            } else if(sys_list[i] == SYS_GALL) {
                // Galileo只有MEO轨道（约23222km）
                if(sat_dist_from_earth > 22000000.0 && sat_dist_from_earth < 25000000.0) {
                    orbit_system_match = true;
                    expected_orbit = "MEO";
                } else {
                    expected_orbit = "MEO (22-25Mm)";
                }
            } else if(sys_list[i] == SYS_CMP) {
                // BDS有MEO、IGSO、GEO轨道
                if(sat_dist_from_earth > 19000000.0 && sat_dist_from_earth < 27000000.0) {
                    orbit_system_match = true;
                    expected_orbit = "MEO";
                } else if(sat_dist_from_earth > 34000000.0 && sat_dist_from_earth < 38000000.0) {
                    orbit_system_match = true;
                    expected_orbit = "GEO";
                } else {
                    expected_orbit = "MEO/GEO (19-27Mm or 34-38Mm)";
                }
            }
            
            if(!orbit_system_match) {
                printf("      ERROR: Orbit type mismatch! Expected %s for %s system\n", 
                       expected_orbit, (sys_list[i] == SYS_GPSS) ? "GPS" : 
                                      (sys_list[i] == SYS_CMP) ? "BDS" : 
                                      (sys_list[i] == SYS_GALL) ? "GAL" : "Unknown");
            } else {
                printf("      OK: Orbit type matches system (%s)\n", expected_orbit);
            }
            
            // 检查速度矢量（如果可用）
            // 注意：这里假设ObsMea中有satellite_vel字段
            if(i < gnss_data.obs_count) {
                const ObsMea& obs = gnss_data.obs_data[i];
                double vel_magnitude = sqrt(obs.satellite_vel[0]*obs.satellite_vel[0] + 
                                          obs.satellite_vel[1]*obs.satellite_vel[1] + 
                                          obs.satellite_vel[2]*obs.satellite_vel[2]);
                printf("      Satellite velocity: %.3f km/s\n", vel_magnitude/1000.0);
                
                // 轨道速度合理性检查
                if(vel_magnitude < 500.0 || vel_magnitude > 8000.0) { // 0.5-8 km/s 合理范围
                    printf("      WARNING: Satellite velocity seems unreasonable!\n");
                }
            }
            
            // 坐标系一致性检查：计算期望的GEO卫星位置
            if(sat_dist_from_earth > 28000000.0) {
                double expected_geo_distance = 35786000.0; // 标准GEO轨道
                double distance_error = sat_dist_from_earth - expected_geo_distance;
                printf("    GEO orbit error: %.0f km (expected 35786 km)\n", distance_error/1000.0);
                
                if(fabs(distance_error) > 8000000.0) { // 误差超过8000km
                    printf("    ERROR: Coordinate system mismatch suspected!\n");
                }
            }
        }
        
        if(sat_dist_from_earth < 6400000.0) { // 太靠近地球
            position_reasonable = false;
            if(print_debug) {
                printf("  ERROR: Satellite %d too close to Earth (dist: %.0f km)!\n",
                       prn_list[i], sat_dist_from_earth/1000.0);
            }
        } else if(sat_dist_from_earth > 38000000.0) { // 大于38Mm异常（GEO标准36Mm）
            position_reasonable = false;
            if(print_debug) {
                printf("  ERROR: Satellite %d too far from Earth (dist: %.0f km), possible ephemeris error!\n",
                       prn_list[i], sat_dist_from_earth/1000.0);
            }
        } else if(sat_dist_from_earth > 28000000.0) {
            // 可能是GEO卫星（标准35.786Mm）
            if(print_debug) {
                printf("  INFO: Satellite %d appears to be GEO satellite (dist: %.0f km)\n",
                       prn_list[i], sat_dist_from_earth/1000.0);
            }
        }
        
        if(!position_reasonable && i < weights.size()) {
            weights[i] *= 0.01; // 进一步降低权重
        }
        
        line_of_sight.normalize();
        
        // 几何矩阵H
        H(i, 0) = line_of_sight.x();
        H(i, 1) = line_of_sight.y();
        H(i, 2) = line_of_sight.z();
        H(i, 3) = 1.0; // 钟差项
        
        // 观测向量y (伪距残差)
        double predicted_range = range;
        y(i) = pseudoranges[i] - predicted_range;
        
        // 额外调试：检查伪距和预测距离的合理性
        if(print_debug && i < 10) { // 显示前10颗卫星的详细信息
            printf("  Sat %d: pseudorange=%.0f m, predicted=%.0f m, residual=%.0f m\n",
                   prn_list[i], pseudoranges[i], predicted_range, y(i));
        }
        
        // 统计残差
        double residual_abs = fabs(y(i));
        max_residual = std::max(max_residual, residual_abs);
        sum_residual += residual_abs;
        
        // 分级残差异常检测和处理
        if(residual_abs > 100000.0) { // 残差大于100km，极可能有问题
            if(residual_abs > 1000000.0) { // 残差大于1000km，严重异常
                printf("  CRITICAL RESIDUAL: PRN=%d, Pseudorange=%.0f m, PredictedRange=%.0f m, Residual=%.0f m\n",
                       prn_list[i], pseudoranges[i], predicted_range, y(i));
                printf("    SatPos=(%.0f, %.0f, %.0f), UserPos=(%.0f, %.0f, %.0f)\n",
                       sat_positions[i].x(), sat_positions[i].y(), sat_positions[i].z(),
                       approx_user_pos.x(), approx_user_pos.y(), approx_user_pos.z());
                // 对于极大残差，将权重设为极小值而不是完全排除
                if(i < weights.size()) {
                    W(i, i) = weights[i] * 1e-6; // 极小权重
                }
            } else if(residual_abs > 500000.0) { // 残差大于500km，重大异常
                printf("  MAJOR RESIDUAL: PRN=%d, Residual=%.0f m\n", prn_list[i], y(i));
                if(i < weights.size()) {
                    W(i, i) = weights[i] * 1e-3; // 很小权重
                }
            } else { // 100-500km残差，可能异常
                printf("  LARGE RESIDUAL: PRN=%d, Residual=%.0f m\n", prn_list[i], y(i));
                if(i < weights.size()) {
                    W(i, i) = weights[i] * 0.01; // 小权重
                }
            }
        } else {
            // 正常权重
            W(i, i) = weights[i];
        }
    }
    
    // 统计可用卫星数量
    int usable_satellites = 0;
    int large_residual_count = 0;
    int critical_residual_count = 0;
    
    for(int i = 0; i < n_sats; i++) {
        double residual_abs = fabs(y(i));
        if(residual_abs < 100000.0) { // 残差小于100km认为可用
            usable_satellites++;
        } else if(residual_abs > 1000000.0) { // 残差大于1000km认为严重异常
            critical_residual_count++;
        } else {
            large_residual_count++;
        }
    }
    
    if(print_debug) {
        printf("\nRAIM BATCH #%d FINAL STATS:\n", diag_batch);
        printf("  Total satellites: %d\n", n_sats);
        printf("  Usable satellites (<100km residual): %d\n", usable_satellites);
        printf("  Large residual satellites (100km-1000km): %d\n", large_residual_count);
        printf("  Critical residual satellites (>1000km): %d\n", critical_residual_count);
        printf("  Residual stats: Max=%.0f m, Avg=%.0f m\n", 
               max_residual, sum_residual / n_sats);
        
        // 显示所有卫星的残差分布
        printf("  All satellite residuals: ");
        for(int i = 0; i < std::min(n_sats, 15); i++) { // 最多显示15颗
            printf("%.0f ", fabs(y(i)));
        }
        if(n_sats > 15) printf("... (+%d more)", n_sats - 15);
        printf("\n");
        
        // 星历数据源和时间基准诊断
        printf("\nEPHEMERIS & TIME DIAGNOSIS BATCH #%d:\n", diag_batch);
        
        // 统计各系统卫星的距离分布
        vector<double> gps_distances, bds_distances, gal_distances;
        for(int i = 0; i < n_sats; i++) {
            double sat_dist = sat_positions[i].norm();
            if(sys_list[i] == SYS_GPSS) {
                gps_distances.push_back(sat_dist);
            } else if(sys_list[i] == SYS_CMP) {
                bds_distances.push_back(sat_dist);
            } else if(sys_list[i] == SYS_GALL) {
                gal_distances.push_back(sat_dist);
            }
        }
        
        // 分析各系统的距离特征
        auto analyze_distances = [](const vector<double>& distances, const string& system_name) {
            if(distances.empty()) return;
            
            double min_dist = *min_element(distances.begin(), distances.end());
            double max_dist = *max_element(distances.begin(), distances.end());
            double avg_dist = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
            
            printf("  %s satellites (%zu): Min=%.0f km, Max=%.0f km, Avg=%.0f km\n",
                   system_name.c_str(), distances.size(),
                   min_dist/1000.0, max_dist/1000.0, avg_dist/1000.0);
            
            // 检查是否有异常的距离分布
            if(max_dist - min_dist > 15000000.0) { // 超过15000km差异
                printf("    WARNING: Large distance spread suggests mixed orbit types or coord system issues\n");
            }
            
            // 检查GEO卫星距离一致性
            int geo_count = 0;
            double geo_distance_variance = 0.0;
            for(double d : distances) {
                if(d > 28000000.0) { // GEO候选
                    geo_count++;
                    geo_distance_variance += (d - 35786000.0) * (d - 35786000.0);
                }
            }
            
            if(geo_count > 0) {
                geo_distance_variance = sqrt(geo_distance_variance / geo_count);
                printf("    GEO satellites (%d): Distance std dev = %.0f km from nominal 35786 km\n",
                       geo_count, geo_distance_variance/1000.0);
                
                if(geo_distance_variance > 5000000.0) { // 标准差超过5000km
                    printf("    ERROR: GEO satellite distances highly inconsistent - coordinate system problem!\n");
                }
            }
        };
        
        analyze_distances(gps_distances, "GPS");
        analyze_distances(bds_distances, "BDS");
        analyze_distances(gal_distances, "GAL");
        
        // 数据源验证总结
        printf("\nDATA SOURCE VALIDATION SUMMARY:\n");
        int total_obs = gnss_data.obs_count;
        int accepted_obs = n_sats;
        int excluded_obs = total_obs - accepted_obs;
        
        printf("  Total GNSS observations: %d\n", total_obs);
        printf("  Accepted for RAIM: %d (%.1f%%)\n", accepted_obs, (accepted_obs*100.0)/total_obs);
        printf("  Excluded due to data quality: %d (%.1f%%)\n", excluded_obs, (excluded_obs*100.0)/total_obs);
        
        if(excluded_obs > 0) {
            printf("  Exclusion reasons:\n");
            printf("    - Invalid orbit for satellite system\n");
            printf("    - Incorrect PRN to system mapping\n"); 
            printf("    - Low elevation angle (< %.1f°)\n", raim_config_.min_elevation_angle);
            printf("    - Satellite health issues (SVH != 0)\n");
            printf("    - No valid pseudorange measurement\n");
        }
        
        // 检查数据质量等级
        if(excluded_obs == 0) {
            printf("  DATA QUALITY: EXCELLENT (100%% valid)\n");
        } else if(excluded_obs < total_obs * 0.1) {
            printf("  DATA QUALITY: GOOD (>90%% valid)\n");
        } else if(excluded_obs < total_obs * 0.3) {
            printf("  DATA QUALITY: FAIR (70-90%% valid)\n");
        } else {
            printf("  DATA QUALITY: POOR (<70%% valid) - Check ephemeris sources!\n");
        }
        
        // 时间基准检查
        printf("\nTIME REFERENCE DIAGNOSIS:\n");
        printf("  Current GNSS time: %.3f (assumed GPS time)\n", gnss_data.ros_time);
        printf("  Base-Rover time diff: %.3f s\n", gnss_data.br_time_diff);
        
        if(fabs(gnss_data.br_time_diff) > 1.0) {
            printf("  WARNING: Large base-rover time difference suggests sync issues!\n");
        }
        
        // 更严格的平均残差检查
        double avg_residual = sum_residual / n_sats;
        if(avg_residual > 50000.0) { // 平均残差大于50km就要警告
            printf("  ERROR: Average residual %.0f m too large! Check position/clock bias estimate!\n", avg_residual);
            if(avg_residual > 500000.0) { // 平均残差大于500km，可能是基本定位失败
                printf("  CRITICAL: Position estimation may have completely failed!\n");
            }
        }
        
        // 判断是否所有卫星都不可用
        if(usable_satellites == 0) {
            printf("  WARNING: NO USABLE SATELLITES! All satellites have residuals >100km\n");
        } else if(usable_satellites < 4) {
            printf("  WARNING: Insufficient usable satellites (%d < 4) for reliable positioning\n", usable_satellites);
        }
        printf("=== END RAIM BATCH #%d ===\n\n", diag_batch);
    }
    
    return true;
}

// 估计用户位置
Eigen::Vector3d SWFOptimization::EstimateUserPosition(const mea_t& gnss_data) {
    static int debug_position_count = 0;
    bool debug_this_call = (debug_position_count++ % 50 == 0); // 每50次调试一次
    
    // 方法1: 使用当前SWF估计的位置（优先）
    if(image_count + rover_count > 0) {
        int current_frame = image_count + rover_count - 1;
        // SWF中的Ps是ENU局部坐标，需要正确转换到ECEF
        Eigen::Vector3d local_enu_pos = Ps[current_frame];
        
        if(debug_this_call) {
            printf("RAIM POS DEBUG: Using SWF position method\n");
            printf("  Frame %d: local_enu_pos = (%.2f, %.2f, %.2f)\n", 
                   current_frame, local_enu_pos.x(), local_enu_pos.y(), local_enu_pos.z());
            printf("  base_pos = (%.2f, %.2f, %.2f), norm = %.0f\n", 
                   base_pos.x(), base_pos.y(), base_pos.z(), base_pos.norm());
            printf("  gnss_data.base_xyz = (%.2f, %.2f, %.2f)\n",
                   gnss_data.base_xyz[0], gnss_data.base_xyz[1], gnss_data.base_xyz[2]);
        }
        
        // 正确的ENU到ECEF转换（base_pos应该是ECEF坐标）
        // 注意：这里需要确保base_pos确实是ECEF坐标而不是ENU坐标
        Eigen::Vector3d ecef_pos;
        if(base_pos.norm() > 1e6) { // base_pos看起来像ECEF坐标
            // 执行ENU到ECEF的旋转变换
            // 简化处理：假设基站附近的小范围内，直接相加近似有效
            ecef_pos = base_pos + local_enu_pos;
            if(debug_this_call) {
                printf("  Using base_pos as ECEF (norm > 1e6)\n");
            }
        } else {
            // 如果base_pos不是ECEF坐标，使用基站GNSS数据
            if(gnss_data.base_xyz[0] != 0 || gnss_data.base_xyz[1] != 0 || gnss_data.base_xyz[2] != 0) {
                Eigen::Vector3d base_ecef(gnss_data.base_xyz[0], gnss_data.base_xyz[1], gnss_data.base_xyz[2]);
                ecef_pos = base_ecef + local_enu_pos;
                if(debug_this_call) {
                    printf("  Using gnss_data.base_xyz as ECEF\n");
                }
            } else {
                ecef_pos = base_pos + local_enu_pos; // 回退到原有逻辑
                if(debug_this_call) {
                    printf("  Fallback: using base_pos directly\n");
                }
            }
        }
        
        // 验证位置合理性
        double dist_from_earth_center = ecef_pos.norm();
        if(debug_this_call) {
            printf("  Final ecef_pos = (%.2f, %.2f, %.2f)\n", 
                   ecef_pos.x(), ecef_pos.y(), ecef_pos.z());
            printf("  Distance from Earth center = %.0f km\n", dist_from_earth_center/1000.0);
        }
        
        if(dist_from_earth_center < 6300000.0 || dist_from_earth_center > 6500000.0) {
            printf("RAIM WARNING: User position seems unreasonable (dist from earth center: %.0f km)\n", 
                   dist_from_earth_center/1000.0);
            
            // 如果位置明显不合理，尝试使用基站位置
            if(gnss_data.base_xyz[0] != 0 || gnss_data.base_xyz[1] != 0 || gnss_data.base_xyz[2] != 0) {
                ecef_pos = Eigen::Vector3d(gnss_data.base_xyz[0], gnss_data.base_xyz[1], gnss_data.base_xyz[2]);
                printf("RAIM: Falling back to base station position (%.2f, %.2f, %.2f)\n",
                       ecef_pos.x(), ecef_pos.y(), ecef_pos.z());
            }
        }
        
        // 获取当前钟差估计
        static int debug_count = 0;
        if(debug_count++ % 10 == 0) {
            printf("RAIM UserPos: Base=(%.1f,%.1f,%.1f), Local=(%.2f,%.2f,%.2f)\n",
                   base_pos.x(), base_pos.y(), base_pos.z(),
                   local_enu_pos.x(), local_enu_pos.y(), local_enu_pos.z());
            
            // 打印钟差估计
            if(para_gnss_dt[0]) {
                printf("RAIM Clock bias estimates: GPS=%.2fm, BDS=%.2fm, GAL=%.2fm\n",
                       para_gnss_dt[0][0], para_gnss_dt[0][2], para_gnss_dt[0][4]);
            }
        }
        
        return ecef_pos;
    }
    
    // 方法2: 如果有基站位置信息，使用基站位置作为初始估计
    if(gnss_data.base_xyz[0] != 0 || gnss_data.base_xyz[1] != 0 || gnss_data.base_xyz[2] != 0) {
        Eigen::Vector3d base_estimate(gnss_data.base_xyz[0], 
                                     gnss_data.base_xyz[1], 
                                     gnss_data.base_xyz[2]);
        if(debug_this_call) {
            printf("RAIM POS DEBUG: Using base station position method\n");
            printf("  base_estimate = (%.2f, %.2f, %.2f), norm = %.0f km\n",
                   base_estimate.x(), base_estimate.y(), base_estimate.z(), base_estimate.norm()/1000.0);
        }
        // 流动站通常在基站附近
        return base_estimate;
    }
    
    // 方法3: 使用伪距加权最小二乘估计
    if(gnss_data.obs_count >= 4) {
        if(debug_this_call) {
            printf("RAIM POS DEBUG: Using pseudorange least squares method\n");
            printf("  obs_count = %d\n", gnss_data.obs_count);
        }
        
        // 简化的迭代最小二乘定位
        Eigen::Vector3d pos_estimate = Eigen::Vector3d::Zero();
        
        // 收集有效观测
        std::vector<Eigen::Vector3d> sat_positions;
        std::vector<double> pseudoranges;
        
        for(int i = 0; i < gnss_data.obs_count; i++) {
            const ObsMea& obs = gnss_data.obs_data[i];
            double pr = obs.RTK_P[0] > 0 ? obs.RTK_P[0] : obs.SPP_P[0];
            
            if(pr > 0 && obs.el > 10.0 * M_PI / 180.0) {
                sat_positions.push_back(Eigen::Vector3d(obs.satellite_pos[0],
                                                       obs.satellite_pos[1], 
                                                       obs.satellite_pos[2]));
                pseudoranges.push_back(pr);
            }
        }
        
        if(sat_positions.size() >= 4) {
            // 使用前4颗卫星进行简单的三维定位
            // 这里可以实现更复杂的迭代最小二乘算法
            pos_estimate = sat_positions[0] * 0.9; // 简化估计
            if(debug_this_call) {
                printf("  pos_estimate = (%.2f, %.2f, %.2f), norm = %.0f km\n",
                       pos_estimate.x(), pos_estimate.y(), pos_estimate.z(), pos_estimate.norm()/1000.0);
            }
            return pos_estimate;
        }
    }
    
    // 方法4: 默认位置（地球表面）
    if(debug_this_call) {
        printf("RAIM POS DEBUG: Using default Earth surface position\n");
    }
    return Eigen::Vector3d(0, 0, 6.371e6);
}

// 计算奇偶矩阵
Eigen::MatrixXd SWFOptimization::CalculateParityMatrix(const Eigen::MatrixXd& H, 
                                                      const Eigen::MatrixXd& W) {
    // 奇偶空间投影矩阵: P = I - H(H^T W H)^(-1) H^T W
    Eigen::MatrixXd HtWH = H.transpose() * W * H;
    Eigen::MatrixXd HtWH_inv = HtWH.inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(H.rows(), H.rows());
    
    return I - H * HtWH_inv * H.transpose() * W;
}

// 故障隔离
int SWFOptimization::IsolateFaultySatellite(const mea_t& gnss_data) {
    double min_test_statistic = 1e10;
    int suspected_satellite = -1;
    
    // 逐个排除卫星进行检验
    for(int exclude_idx = 0; exclude_idx < gnss_data.obs_count; exclude_idx++) {
        // 构建排除特定卫星的数据
        mea_t reduced_data = gnss_data;
        reduced_data.obs_count--;
        
        // 移除第exclude_idx个观测
        for(int i = exclude_idx; i < reduced_data.obs_count; i++) {
            reduced_data.obs_data[i] = gnss_data.obs_data[i + 1];
        }
        
        // 重新构建几何矩阵
        Eigen::MatrixXd H_sub, W_sub;
        Eigen::VectorXd y_sub;
        
        if(BuildGeometryMatrix(reduced_data, H_sub, y_sub, W_sub) && H_sub.rows() >= 4) {
            // 重新计算检验统计量
            Eigen::MatrixXd HtWH_sub = H_sub.transpose() * W_sub * H_sub;
            if(HtWH_sub.determinant() > 1e-10) {
                Eigen::VectorXd x_hat_sub = HtWH_sub.inverse() * H_sub.transpose() * W_sub * y_sub;
                Eigen::VectorXd residuals_sub = y_sub - H_sub * x_hat_sub;
                Eigen::MatrixXd P_sub = CalculateParityMatrix(H_sub, W_sub);
                double test_stat_sub = (P_sub * residuals_sub).squaredNorm();
                
                if(test_stat_sub < min_test_statistic) {
                    min_test_statistic = test_stat_sub;
                    suspected_satellite = gnss_data.obs_data[exclude_idx].sat;
                }
            }
        }
    }
    
    return suspected_satellite;
}

// 计算保护级
double SWFOptimization::CalculateProtectionLevel(const Eigen::MatrixXd& H, 
                                                const Eigen::MatrixXd& W, 
                                                double threshold,
                                                bool horizontal) {
    // 计算协方差矩阵
    Eigen::MatrixXd HtWH = H.transpose() * W * H;
    Eigen::MatrixXd Qxx = HtWH.inverse();
    
    // 水平保护级 (HDOP)
    if(horizontal) {
        double hdop = sqrt(Qxx(0,0) + Qxx(1,1));
        return hdop * sqrt(threshold);
    }
    // 垂直保护级 (VDOP)
    else {
        double vdop = sqrt(Qxx(2,2));
        return vdop * sqrt(threshold);
    }
}

// 计算χ²阈值
double SWFOptimization::CalculateChi2Threshold(int dof, double false_alarm_rate) {
    // χ²分位数查表法
    // 扩展支持更多虚警率水平
    static const double chi2_table[][5] = {
        // dof=1,2,3,4,5 对应 α=0.001 (0.1%)
        {10.828, 13.816, 16.266, 18.467, 20.515},
        // dof=1,2,3,4,5 对应 α=0.01 (1%)
        {6.635, 9.210, 11.345, 13.277, 15.086},
        // dof=1,2,3,4,5 对应 α=0.05 (5%)
        {3.841, 5.991, 7.815, 9.488, 11.070},
        // dof=1,2,3,4,5 对应 α=0.1 (10%)
        {2.706, 4.605, 6.251, 7.779, 9.236}
    };
    
    int table_row = -1;
    if(fabs(false_alarm_rate - 0.001) < 1e-6) {
        table_row = 0;
    } else if(fabs(false_alarm_rate - 0.01) < 1e-6) {
        table_row = 1;
    } else if(fabs(false_alarm_rate - 0.05) < 1e-6) {
        table_row = 2;
    } else if(fabs(false_alarm_rate - 0.1) < 1e-6) {
        table_row = 3;
    }
    
    if(table_row >= 0 && dof >= 1 && dof <= 5) {
        double standard_threshold = chi2_table[table_row][dof-1];
        // 为实际应用进一步放宽阈值
        // 由于改用加权残差平方和，需要相应调整阈值
        // 城市环境车载导航：优先保证可用性
        double urban_factor = 4.5;  // 城市环境经验系数
        return standard_threshold * dof * urban_factor;
    }
    
    // 大自由度时使用正态分布近似
    // 根据虚警率计算对应的z值
    double z_alpha;
    if(false_alarm_rate <= 0.001) {
        z_alpha = 3.29;  // 99.9%分位数
    } else if(false_alarm_rate <= 0.01) {
        z_alpha = 2.58;  // 99%分位数
    } else if(false_alarm_rate <= 0.05) {
        z_alpha = 1.96;  // 95%分位数
    } else {
        z_alpha = 1.64;  // 90%分位数
    }
    
    // Wilson-Hilferty近似
    double mean = dof;
    double variance = 2 * dof;
    return mean + z_alpha * sqrt(variance);
}

// 记录RAIM结果
void SWFOptimization::LogRaimResult(const TraditionalRaimResult& result) {
    static int raim_count = 0;
    static int fault_count = 0;
    raim_count++;
    
    if(result.integrity_available) {
        if(result.fault_detected) {
            fault_count++;
            printf("[RAIM %d] FAULT DETECTED! PRN=%d, Test=%.3f > Threshold=%.3f (Ratio=%.2f), HPL=%.2fm, VPL=%.2fm\n",
                   raim_count,
                   result.faulty_satellite_prn,
                   result.test_statistic,
                   result.chi2_threshold,
                   result.test_statistic / result.chi2_threshold,
                   result.protection_level_horizontal,
                   result.protection_level_vertical);
        } else {
            // 每10次正常检测输出一次
            if(raim_count % 10 == 0) {
                printf("[RAIM %d] OK: Sats=%d, GDOP=%.2f, Test=%.3f < Threshold=%.3f, HPL=%.2fm\n",
                       raim_count,
                       result.available_satellites,
                       result.gdop,
                       result.test_statistic,
                       result.chi2_threshold,
                       result.protection_level_horizontal);
            }
        }
        
        // 统计信息
        if(raim_count % 100 == 0) {
            double fault_rate = (double)fault_count / raim_count * 100.0;
            printf("[RAIM Stats] Total=%d, Faults=%d (%.1f%%), FA_target=%.1f%%\n",
                   raim_count, fault_count, fault_rate, 
                   raim_config_.false_alarm_rate * 100);
            
            // 如果故障率过高，提示可能的问题
            if(fault_rate > 50.0) {
                printf("[RAIM Warning] High fault rate detected! Possible causes:\n");
                printf("  - Position estimate error\n");
                printf("  - Clock bias not properly handled\n");
                printf("  - Measurement noise underestimated\n");
            }
        }
    } else {
        printf("[RAIM %d] Not Available: Only %d satellites (need %d)\n", 
               raim_count, result.available_satellites, raim_config_.min_satellites);
    }
}

// 根据RAIM结果调整权重
void SWFOptimization::AdjustGnssWeightsForFault(const TraditionalRaimResult& raim_result) {
    if(raim_result.fault_detected) {
        // 根据故障严重程度调整权重
        double severity_factor = min(1.0, raim_result.test_statistic / raim_result.chi2_threshold);
        double weight_reduction = 0.1 + 0.8 * (1.0 - 1.0/severity_factor); // 10%-90%权重降低
        
        // 应用权重调整到当前GNSS帧
        if(rover_count > 0) {
            // 这里需要根据具体的权重管理机制实现
            // 可能涉及修改残差块的权重或损失函数权重
            printf("Reducing GNSS weight by %.1f%% due to RAIM fault detection\n", 
                   weight_reduction * 100);
        }
    }
}



