#include "swf.h"
#include "../factor/gnss_factor.h"
#include "../factor/imu_factor.h"
#include "../factor/projection_factor.h"
#include "../factor/marginalization_factor.h"

// GNSS residual evaluation
void SWFOptimization::EvaluateGnssResiduals() {
    current_residuals.gnss_residuals.clear();
    current_residuals.gnss_weighted_residuals.clear();
    
    // 安全检查
    if (rover_count <= 0) {
        return; // 无GNSS数据时直接返回
    }
    
    // 遍历当前GNSS观测，同时计算原始和加权残差
    for (int ir = 0; ir < rover_count; ir++) {
        mea_t* rover = rovers[ir];
        if (!rover) {
            continue;
        }
        
        // 检查g2f索引有效性
        int frame_index = g2f[ir];
        if (frame_index < 0 || frame_index >= (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)) {
            continue;
        }
        
        if (!para_pose[frame_index]) {
            continue;
        }
        
        double globalxyz[3];
        globalxyz[0] = para_pose[frame_index][0] + rover->base_xyz[0];
        globalxyz[1] = para_pose[frame_index][1] + rover->base_xyz[1];
        globalxyz[2] = para_pose[frame_index][2] + rover->base_xyz[2];
        
        // RTK载波相位残差
        if (USE_RTK) {
            for (uint8_t i = 0; i < rover->obs_count; i++) {
                ObsMea* d = rover->obs_data + i;
                for (uint8_t f = 0; f < NFREQ; f++) {
                    if (d->RTK_Npoint[f] && d->el >= AZELMIN) {
                        // 计算原始残差（未加权）
                        double r1, e1[3];
                        r1 = distance(globalxyz, d->satellite_pos, e1);
                        double raw_residual = r1 - d->RTK_Npoint[f]->value * lams[d->sys][f] - d->RTK_L[f]*lams[d->sys][f] + para_gnss_dt[0][d->sys*2+f];
                        current_residuals.gnss_residuals.push_back(raw_residual);
                        
                        // 计算加权残差（使用因子）
                        RTKCarrierPhaseFactor factor(d->satellite_pos, d->RTK_L[f]*lams[d->sys][f], 
                                                   lams[d->sys][f], d->el, rover->br_time_diff,
                                                   pow(d->RTK_Lstd[f]*lams[d->sys][f],2), rover->base_xyz,
                                                   true, d->sys, f);
                        
                        double weighted_residual;
                        const double* parameters[] = {para_pose[frame_index], &(d->RTK_Npoint[f]->value), 
                                                     para_gnss_dt[0]+d->sys*2+f};
                        factor.Evaluate(parameters, &weighted_residual, nullptr);
                        current_residuals.gnss_weighted_residuals.push_back(weighted_residual);
                    }
                }
            }
        }
        
        // RTK伪距残差
        if (USE_RTD) {
            for (uint8_t i = 0; i < rover->obs_count; i++) {
                ObsMea* d = rover->obs_data + i;
                for (uint8_t f = 0; f < NFREQ; f++) {
                    if (d->RTK_P[f] != 0.0 && d->SVH == 0 && d->RTK_Pstd[f] <= 2 && d->el >= AZELMIN) {
                        // 计算原始残差（未加权）
                        double r1, e1[3];
                        r1 = distance(globalxyz, d->satellite_pos, e1);
                        double raw_residual = r1 - d->RTK_P[f] + para_gnss_dt[0][d->sys*2+f];
                        current_residuals.gnss_residuals.push_back(raw_residual);
                        
                        // 计算加权残差（使用因子）
                        RTKPseudorangeFactor factor(d->satellite_pos, d->RTK_P[f], d->el, 
                                                  rover->br_time_diff, pow(d->RTK_Pstd[f],2), rover->base_xyz);
                        
                        double weighted_residual;
                        const double* parameters[] = {para_pose[frame_index], para_gnss_dt[0]+d->sys*2+f};
                        factor.Evaluate(parameters, &weighted_residual, nullptr);
                        current_residuals.gnss_weighted_residuals.push_back(weighted_residual);
                    }
                }
            }
        }
        
        // 多普勒残差
        if (USE_DOPPLER) {
            for (uint8_t i = 0; i < rover->obs_count; i++) {
                ObsMea* d = rover->obs_data + i;
                if (d->SPP_D[0] != 0.0 && d->SVH == 0 && d->SPP_Dstd[0] <= 2 && d->el >= AZELMIN) {
                    // 计算原始残差（未加权）
                    double e[3];
                    double rate = velecitydistance(globalxyz, d->satellite_pos, para_speed_bias[frame_index], d->satellite_vel, e);
                    double raw_residual = rate + para_gnss_dt[0][12] + d->SPP_D[0]*lams[d->sys][0];
                    current_residuals.gnss_residuals.push_back(raw_residual);
                    
                    // 计算加权残差（使用因子）
                    double istd = sin(d->el) * sin(d->el) / (d->SPP_Dstd[0] * lams[d->sys][0]);
                    SppDopplerFactor factor(d->satellite_vel, d->satellite_pos, para_pose[0],
                                          d->SPP_D[0]*lams[d->sys][0], istd, rover->base_xyz);
                    
                    double weighted_residual;
                    const double* parameters[] = {para_speed_bias[frame_index], para_gnss_dt[0]+12, para_pose[frame_index]};
                    factor.Evaluate(parameters, &weighted_residual, nullptr);
                    current_residuals.gnss_weighted_residuals.push_back(weighted_residual);
                }
            }
        }
    }
}

// IMU residual evaluation
void SWFOptimization::EvaluateImuResiduals() {
    current_residuals.imu_residual.setZero();
    current_residuals.imu_weighted_residual.setZero();
    current_residuals.imu_factor_type = 0;
    
    // 寻找最新的IMU因子
    int index = rover_count + image_count - 1;
    if (index > 0 && pre_integrations[index]) {
        // 计算原始残差（未加权）
        Eigen::Vector3d Pi(para_pose[index-1][0], para_pose[index-1][1], para_pose[index-1][2]);
        Eigen::Quaterniond Qi(para_pose[index-1][6], para_pose[index-1][3], para_pose[index-1][4], para_pose[index-1][5]);
        Eigen::Vector3d Vi(para_speed_bias[index-1][0], para_speed_bias[index-1][1], para_speed_bias[index-1][2]);
        Eigen::Vector3d Bai(para_speed_bias[index-1][3], para_speed_bias[index-1][4], para_speed_bias[index-1][5]);
        Eigen::Vector3d Bgi(para_speed_bias[index-1][6], para_speed_bias[index-1][7], para_speed_bias[index-1][8]);
        
        Eigen::Vector3d Pj(para_pose[index][0], para_pose[index][1], para_pose[index][2]);
        Eigen::Quaterniond Qj(para_pose[index][6], para_pose[index][3], para_pose[index][4], para_pose[index][5]);
        Eigen::Vector3d Vj(para_speed_bias[index][0], para_speed_bias[index][1], para_speed_bias[index][2]);
        Eigen::Vector3d Baj(para_speed_bias[index][3], para_speed_bias[index][4], para_speed_bias[index][5]);
        Eigen::Vector3d Bgj(para_speed_bias[index][6], para_speed_bias[index][7], para_speed_bias[index][8]);
        
        // 获取原始残差
        current_residuals.imu_residual = pre_integrations[index]->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj, Pbg);
        
        // 计算加权残差（使用IMU因子）
        IMUFactor factor(pre_integrations[index]);
        double weighted_residuals[15];
        const double* parameters[] = {para_pose[index-1], para_speed_bias[index-1], 
                                     para_pose[index], para_speed_bias[index]};
        factor.Evaluate(parameters, weighted_residuals, nullptr);
        
        for (int i = 0; i < 15; i++) {
            current_residuals.imu_weighted_residual(i) = weighted_residuals[i];
        }
        current_residuals.imu_factor_type = 0;
    }
    
    // 检查是否使用IMU-GNSS联合因子
    for (int i = 0; i < image_count + 1; i++) {
        if (imu_gnss_factor[i]) {
            current_residuals.imu_factor_type = 1;
            break;
        }
    }
}

// Visual residual evaluation
void SWFOptimization::EvaluateVisualResiduals() {
    current_residuals.visual_residuals.clear();
    current_residuals.visual_weighted_residuals.clear();
    
    try {
        for (auto& it_per_id : f_manager.feature) {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < FEATURE_CONTINUE) continue;
            
            int imu_j = it_per_id.start_frame;
            
            for (auto& it_per_frame : it_per_id.feature_per_frame) {
                // 添加边界检查
                if (imu_j < 0 || imu_j >= image_count) {
                    imu_j++;
                    continue;
                }
                
                int frame_idx = i2f[imu_j];
                if (frame_idx < 0 || frame_idx >= (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)) {
                    imu_j++;
                    continue;
                }
                
                if (!para_pose[frame_idx] || !para_ex_Pose[0]) {
                    imu_j++;
                    continue;
                }
                
#if USE_INVERSE_DEPTH
                if (it_per_id.start_frame != imu_j) {
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                    Vector3d pts_j = it_per_frame.point;
                    
                    ProjectionTwoFrameOneCamFactor factor(pts_i, pts_j);
                    
                    double weighted_residuals[2];
                    const double* parameters[] = {para_pose[i2f[it_per_id.start_frame]], 
                                                 para_pose[frame_idx], 
                                                 para_ex_Pose[0], 
                                                 &it_per_id.idepth_};
                    factor.Evaluate(parameters, weighted_residuals, nullptr);
                    
                    // 获取原始残差（在因子内部保存的）
                    Eigen::Vector2d raw_residual = ProjectionTwoFrameOneCamFactor::last_raw_residual;
                    current_residuals.visual_residuals.push_back(std::make_pair(it_per_id.feature_id, raw_residual));
                    
                    // 获取加权残差
                    Eigen::Vector2d weighted_residual(weighted_residuals[0], weighted_residuals[1]);
                    current_residuals.visual_weighted_residuals.push_back(std::make_pair(it_per_id.feature_id, weighted_residual));
                }
#else
                projection_factor factor(it_per_frame.point);
                
                double weighted_residuals[2];
                const double* parameters[] = {para_pose[frame_idx], 
                                             para_ex_Pose[0], 
                                             it_per_id.ptsInWorld.data()};
                factor.Evaluate(parameters, weighted_residuals, nullptr);
                
                // 获取原始残差（在因子内部保存的）
                Eigen::Vector2d raw_residual = projection_factor::last_raw_residual;
                current_residuals.visual_residuals.push_back(std::make_pair(it_per_id.feature_id, raw_residual));
                
                // 获取加权残差
                Eigen::Vector2d weighted_residual(weighted_residuals[0], weighted_residuals[1]);
                current_residuals.visual_weighted_residuals.push_back(std::make_pair(it_per_id.feature_id, weighted_residual));
#endif
                imu_j++;
            }
        }
    } catch (const std::exception& e) {
        // 如果出现异常，清空结果并继续
        current_residuals.visual_residuals.clear();
    }
}

// Prior residual evaluation
void SWFOptimization::EvaluatePriorResiduals() {
    // 简化先验残差评估，避免可能的内存问题
    current_residuals.prior_residual.resize(0);
    
    // 暂时禁用先验残差评估以避免内存问题
    // 可以通过设置简单的默认值或跳过复杂的MarginalizationFactor评估
    if (last_marg_info && last_marg_info->m > 0 && last_marg_info->m <= 1000) {
        // 仅设置维度信息，不进行实际残差计算
        // 这避免了MarginalizationFactor中可能的内存问题
        int residual_size = std::min(last_marg_info->m, 1000);
        current_residuals.prior_residual.resize(residual_size);
        current_residuals.prior_residual.setZero(); // 设置为零向量
    }
}

// Statistics calculation functions
SWFOptimization::ResidualStats SWFOptimization::CalculateGnssStats(const std::vector<double>& gnss_residuals) {
    ResidualStats stats;
    
    if (gnss_residuals.empty()) return stats;
    
    // 计算未加权残差范数
    stats.residual_norm = sqrt(std::inner_product(gnss_residuals.begin(), gnss_residuals.end(), 
                                                  gnss_residuals.begin(), 0.0));
    
    // 计算加权残差范数
    if (!current_residuals.gnss_weighted_residuals.empty()) {
        stats.weighted_norm = sqrt(std::inner_product(current_residuals.gnss_weighted_residuals.begin(), 
                                                      current_residuals.gnss_weighted_residuals.end(), 
                                                      current_residuals.gnss_weighted_residuals.begin(), 0.0));
    } else {
        stats.weighted_norm = stats.residual_norm; // 后备方案
    }
    
    // RMS误差（基于未加权残差）
    stats.rms_error = stats.residual_norm / sqrt(gnss_residuals.size());
    
    return stats;
}

SWFOptimization::ResidualStats SWFOptimization::CalculateImuStats(const Eigen::Matrix<double, 15, 1>& imu_residual) {
    ResidualStats stats;
    
    // 计算未加权残差范数
    stats.residual_norm = imu_residual.norm();
    
    // 计算加权残差范数
    stats.weighted_norm = current_residuals.imu_weighted_residual.norm();
    
    // RMS误差（基于未加权残差）
    stats.rms_error = stats.residual_norm / sqrt(15);
    
    return stats;
}

SWFOptimization::ResidualStats SWFOptimization::CalculateVisualStats(const std::vector<std::pair<int, Eigen::Vector2d>>& visual_residuals) {
    ResidualStats stats;
    
    if (visual_residuals.empty()) return stats;
    
    double norm_squared = 0.0;
    int total_dims = 0;
    
    for (const auto& res : visual_residuals) {
        norm_squared += res.second.squaredNorm();
        total_dims += 2; // 每个特征点2维残差
    }
    
    // 残差范数
    stats.residual_norm = sqrt(norm_squared);
    
    // RMS误差
    if (total_dims > 0) {
        stats.rms_error = stats.residual_norm / sqrt(total_dims);
    } else {
        stats.rms_error = 0.0;
    }
    
    // 计算未加权残差范数
    stats.residual_norm = sqrt(norm_squared);
    
    // 计算加权残差范数（直接使用已收集的加权残差）
    if (!current_residuals.visual_weighted_residuals.empty()) {
        double weighted_norm_squared = 0.0;
        for (const auto& res : current_residuals.visual_weighted_residuals) {
            weighted_norm_squared += res.second.squaredNorm();
        }
        stats.weighted_norm = sqrt(weighted_norm_squared);
    } else {
        stats.weighted_norm = stats.residual_norm; // 后备方案
    }
    
    return stats;
}

SWFOptimization::ResidualStats SWFOptimization::CalculatePriorStats(const Eigen::VectorXd& prior_residual) {
    ResidualStats stats;
    
    if (prior_residual.size() == 0) return stats;
    
    // 残差范数
    stats.residual_norm = prior_residual.norm();
    
    // RMS误差
    stats.rms_error = stats.residual_norm / sqrt(prior_residual.size());
    
    // 先验残差的权重处理复杂，暂时简化
    // TODO: 实现完整的先验残差原始/加权分离
    stats.weighted_norm = stats.residual_norm; // 暂时相同
    
    return stats;
}

void SWFOptimization::CalculateAllStats() {
    try {
        current_residuals.gnss_stats = CalculateGnssStats(current_residuals.gnss_residuals);
    } catch (const std::exception& e) {
        current_residuals.gnss_stats = ResidualStats(); // 默认值
    }
    
    try {
        current_residuals.imu_stats = CalculateImuStats(current_residuals.imu_residual);
    } catch (const std::exception& e) {
        current_residuals.imu_stats = ResidualStats(); // 默认值
    }
    
    try {
        current_residuals.visual_stats = CalculateVisualStats(current_residuals.visual_residuals);
    } catch (const std::exception& e) {
        current_residuals.visual_stats = ResidualStats(); // 默认值
    }
    
    try {
        current_residuals.prior_stats = CalculatePriorStats(current_residuals.prior_residual);
    } catch (const std::exception& e) {
        current_residuals.prior_stats = ResidualStats(); // 默认值
    }
    
    // 计算总残差范数
    current_residuals.total_residual_norm = sqrt(
        pow(current_residuals.gnss_stats.residual_norm, 2) +
        pow(current_residuals.imu_stats.residual_norm, 2) +
        pow(current_residuals.visual_stats.residual_norm, 2) +
        pow(current_residuals.prior_stats.residual_norm, 2)
    );
}

void SWFOptimization::EvaluateAllResiduals() {
    // 安全检查并保存基本信息
    int index = rover_count + image_count - 1;
    if (index < 0 || index >= (FEATURE_WINDOW_SIZE + GNSS_WINDOW_SIZE + 1)) {
        current_residuals.timestamp = 0.0;
        current_residuals.position.setZero();
    } else {
        current_residuals.timestamp = headers[index];
        current_residuals.position = Ps[index];
    }
    
    // 分别评估各类残差，如果某个出错不影响其他的
    try {
        EvaluateGnssResiduals();
    } catch (const std::exception& e) {
        current_residuals.gnss_residuals.clear();
    }
    
    try {
        EvaluateImuResiduals();
    } catch (const std::exception& e) {
        current_residuals.imu_residual.setZero();
        current_residuals.imu_factor_type = 0;
    }
    
    try {
        EvaluateVisualResiduals();
    } catch (const std::exception& e) {
        current_residuals.visual_residuals.clear();
    }
    
    try {
        EvaluatePriorResiduals();
    } catch (const std::exception& e) {
        current_residuals.prior_residual.resize(0);
    }
    
    try {
        CalculateAllStats();
    } catch (const std::exception& e) {
        // 如果统计计算失败，设置默认值
        current_residuals.gnss_stats = ResidualStats();
        current_residuals.imu_stats = ResidualStats();
        current_residuals.visual_stats = ResidualStats();
        current_residuals.prior_stats = ResidualStats();
        current_residuals.total_residual_norm = 0.0;
    }
}

