// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "KFProcessing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_traits.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.hpp"
#include <unordered_map>
#include <algorithm>
#include <execution>
#include <unordered_set>
#include <Eigen/Dense>
#include <filesystem>

#define INIT_TIME           (0.5)
#define MAXN                (720000)
// #define VIS_VOXEL_MAP //uncomment to visualize voxel map

double s_plot[MAXN], s_plot1[MAXN], s_plot2[MAXN], s_plot3[MAXN];//, s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
bool pcd_save_en = false, path_en = true;

mutex mtx_buffer, mtx_map;
string root_dir = ROOT_DIR;
string log_dir;

double gyr_cov = 0.1, acc_cov = 0.1, LASER_POINT_COV = 0.001;
double filter_size_surf_min = 0;
double lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0;
int    feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<PointCloudXYZI> lidar_buffer;
deque<vector<double>> gt_buffer;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

pcl::UniformSampling<PointType> downSizeFilterSurf;//, downSizeFilterPCD;
bool runtime_pos_log = false;

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 6> kf;
state_ikfom state_point;

nav_msgs::Path path, path_gt;
nav_msgs::Odometry odomAftMapped;
nav_msgs::Odometry odomImu;
geometry_msgs::Quaternion geoQuat;


shared_ptr<Preprocess> p_pre;
shared_ptr<KFProcess> p_kf(new KFProcess());

/*** Voxel map ***/
double rootSurfVoxelSize;
int max_layer;
vector<unordered_map<VOXEL_LOC, OCTO_TREE*>::iterator> feat_map_update_iter;
unordered_map<VOXEL_LOC, OCTO_TREE*> surf_map;

/*** Segment point cloud ***/
PointCloudXYZI::Ptr  ptr_seg(new PointCloudXYZI());
double lidar_mean_scantime = 0.1;

V3D pos_cur(Zero3d);
V3D vel_cur(Zero3d);

std::vector<std::deque<LidarMsgGroup>> lidar_msg_buffer;
int fix_rate = 50;
ros::Publisher pubOdomAftMapped, pubPath, pubgtPath, pubLaserCloudFull, points_norm_pub, voxel_map_pub;
ros::Publisher pubLaserCloudFull_body;
FILE *fp;
std::ofstream pose_json;
bool pcs = true;
int rms_win_size = 10;
double k1 = 180;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeNorm;
pcl::PointCloud<pcl::PointXYZI>::Ptr pre_build_cloud(new pcl::PointCloud<pcl::PointXYZI>);
inline void dump_mlo_state_to_log(FILE *fp) {
    //TUM Format
    fprintf(fp, "%lf ", lidar_end_time); // Time 0
    fprintf(fp, "%lf %lf %lf ", pos_cur(0), pos_cur(1), pos_cur(2));
    fprintf(fp, "%lf %lf %lf %lf", geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointWorldToBody(PointType* pi, pcl::PointXYZI* po) {
    V3D p_world(pi->x, pi->y, pi->z);
    Eigen::Quaterniond quat(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
    V3D p_body(quat.toRotationMatrix().transpose()*(p_world - pos_cur));
    po->x = p_body(0);
    po->y = p_body(1);
    po->z = p_body(2);
    po->intensity = pi->intensity;
}

template <typename PointT>
void pointBodyToWorld(PointT* pi, PointT* po) {
    V3D p_body(pi->x, pi->y, pi->z);
    double dt = 0.0;
    if constexpr (pcl::traits::has_curvature<PointT>::value) dt = pi->curvature/double(1000);
    V3D acc_w = state_point.rot.toRotationMatrix()*state_point.acc;
    V3D p_global(state_point.rot.toRotationMatrix()*Exp(state_point.omg, dt)*(p_kf->Lidar_R_wrt_IMU*p_body + p_kf->Lidar_T_wrt_IMU)
                 + state_point.pos + state_point.vel*dt + 0.5*acc_w*dt*dt);
    *po = *pi;
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_imu(p_kf->Lidar_R_wrt_IMU*V3D(pi->x, pi->y, pi->z) + p_kf->Lidar_T_wrt_IMU);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg, const int& lidar_id) {
    mtx_buffer.lock();
    LidarMsgGroup lidar_msg;
    lidar_msg.msg_beg_time = msg->header.stamp.toSec()+p_pre->header_time_offset[lidar_id];
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, lidar_id);
    sort(ptr->points.begin(), ptr->points.end(), time_list);
    if (ptr->points.size() > 0) {
        lidar_msg.msg_end_time = lidar_msg.msg_beg_time + ptr->points[ptr->points.size()-1].curvature / double(1000);
        lidar_msg.point_beg_time = lidar_msg.msg_beg_time;
    } else {
        mtx_buffer.unlock();
        return;
    }

    lidar_msg.cloud = *ptr;
    lidar_msg.lidar_id = lidar_id;
    lidar_msg_buffer[lidar_id].push_back(lidar_msg);

    mtx_buffer.unlock();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg, const int& lidar_id) {
    mtx_buffer.lock();
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, lidar_id);
    sort(ptr->points.begin(), ptr->points.end(), time_list);
    LidarMsgGroup lidar_msg;
    lidar_msg.msg_beg_time = msg->header.stamp.toSec()+p_pre->header_time_offset[lidar_id];
    if (ptr->points.size() > 0) {
        lidar_msg.msg_end_time = lidar_msg.msg_beg_time + ptr->points[ptr->points.size()-1].curvature / double(1000);
        lidar_msg.point_beg_time = lidar_msg.msg_beg_time;
    } else {
        mtx_buffer.unlock();
        return;
    }
    lidar_msg.cloud = *ptr;
    lidar_msg.lidar_id = lidar_id;
    lidar_msg_buffer[lidar_id].push_back(lidar_msg);
    mtx_buffer.unlock();
}

void livox2_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg, const int& lidar_id) {
    mtx_buffer.lock();
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, lidar_id);
    sort(ptr->points.begin(), ptr->points.end(), time_list);
    LidarMsgGroup lidar_msg;
    lidar_msg.msg_beg_time = msg->header.stamp.toSec()+p_pre->header_time_offset[lidar_id];
    if (ptr->points.size() > 0) {
        lidar_msg.msg_end_time = lidar_msg.msg_beg_time + ptr->points[ptr->points.size()-1].curvature / double(1000);
        lidar_msg.point_beg_time = lidar_msg.msg_beg_time;
    } else {
        mtx_buffer.unlock();
        return;
    }
    lidar_msg.cloud = *ptr;
    lidar_msg.lidar_id = lidar_id;
    lidar_msg_buffer[lidar_id].push_back(lidar_msg);
    mtx_buffer.unlock();
}

bool sync_packages(MeasureGroup &meas) {
    std::vector<double> lidar_time;
    for (int i=0; i<lidar_num; i++) {
        if (lidar_msg_buffer[i].size()) {
            lidar_time.push_back(lidar_msg_buffer[i].front().point_beg_time);
        }
    }
    double win_beg_time;
    if (lidar_time.size()) {
        auto minTime = std::min_element(lidar_time.begin(), lidar_time.end());
        win_beg_time = *minTime;
    } else return false;

    /*Drop lidar scans to keep the real-time performance */
    // lidar_time.clear();
    // for (int i=0; i<lidar_num; i++) {
    //     if (lidar_msg_buffer[i].size()) {
    //         lidar_time.push_back(lidar_msg_buffer[i].back().point_beg_time);
    //     }
    // }
    // auto maxTime = std::min_element(lidar_time.begin(), lidar_time.end());
    // if (*maxTime-win_beg_time > 0.2) win_beg_time = *maxTime; 

    lidar_end_time = win_beg_time + lidar_mean_scantime;
    int pc_size = 0;
    for (int i=0; i<lidar_num; i++) {
        while(!lidar_msg_buffer[i].empty()) {
            if (lidar_msg_buffer[i].front().msg_end_time < win_beg_time) {
                lidar_msg_buffer[i].pop_front();
            } else {
                if (lidar_msg_buffer[i].front().msg_end_time >= win_beg_time && 
                    lidar_msg_buffer[i].front().point_beg_time <= lidar_end_time) 
                    pc_size++;
                break;
            }
        }
    }
    if (pc_size==0) return false;
    for (int lidar_id=0; lidar_id<lidar_num; lidar_id++) {
        meas.lidar[lidar_id].clear();
        bool loop_finish = true;        
        while(loop_finish && !lidar_msg_buffer[lidar_id].empty()) {
            for (int i = 0; i < lidar_msg_buffer[lidar_id].front().cloud.size(); i++) {                
                auto pt = lidar_msg_buffer[lidar_id].front().cloud.points[i];
                auto time_pt = lidar_msg_buffer[lidar_id].front().msg_beg_time + pt.curvature / double(1000);
                
                if (time_pt >= win_beg_time && time_pt <= lidar_end_time) {
                    pt.curvature = 1000.0*(time_pt-win_beg_time); //ms
                    ptr_seg->push_back(pt);
                    if (i==lidar_msg_buffer[lidar_id].front().cloud.size()-1) {
                        meas.lidar[lidar_id] += *ptr_seg;    
                        ptr_seg->clear();
                        lidar_msg_buffer[lidar_id].pop_front();
                        break;
                    }
                } else if (time_pt > lidar_end_time) {
                    meas.lidar[lidar_id] += *ptr_seg;    
                    ptr_seg->clear();
                    if (i!=(lidar_msg_buffer[lidar_id].front().cloud.size()-1)) {
                        std::vector<int> index(lidar_msg_buffer[lidar_id].front().cloud.size()-i);
                        std::iota(index.begin(), index.end(), i);
                        PointCloudXYZI point_tmp;
                        pcl::copyPointCloud(lidar_msg_buffer[lidar_id].front().cloud, index, point_tmp);
                        lidar_msg_buffer[lidar_id].front().cloud.swap(point_tmp);
                        lidar_msg_buffer[lidar_id].front().point_beg_time = lidar_msg_buffer[lidar_id].front().msg_beg_time + pt.curvature / double(1000);
                    } else lidar_msg_buffer[lidar_id].pop_front();
                    loop_finish = false;
                    break;
                } else if (i==lidar_msg_buffer[lidar_id].front().cloud.size()-1) {
                    lidar_msg_buffer[lidar_id].pop_front();
                    break;
                } 
            }
        }
    }
    meas.lidar_beg_time = win_beg_time;
    return true;
}

template <typename PointT>
void cut_voxel(std::unordered_map<VOXEL_LOC, OCTO_TREE*> &feat_map, typename pcl::PointCloud<PointT>::Ptr pl_feat) {
    feat_map_update_iter.clear();
    auto size = pl_feat->points.size();
    typename pcl::PointCloud<PointT>::Ptr laserCloudWorld(new pcl::PointCloud<PointT>(size, 1));
    if (PREBUILDMAP) {
        cout << "Getting Point Cloud from Pre-build Map ..." << endl;
        *laserCloudWorld = *pl_feat;
    } else {
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
        for (int i = 0; i < size; i++) {
            pointBodyToWorld<PointT>(&pl_feat->points[i], &laserCloudWorld->points[i]);
        }
    }

    
    std::for_each(std::execution::unseq, laserCloudWorld->points.begin(), laserCloudWorld->points.end(), [size](const auto& pt_w) {
        V3D pvec_tran(pt_w.x, pt_w.y, pt_w.z);
        VOXEL_LOC position(floor(pvec_tran[0]/rootSurfVoxelSize), floor(pvec_tran[1]/rootSurfVoxelSize), floor(pvec_tran[2]/rootSurfVoxelSize));
        // Find corresponding voxel
        auto iter = surf_map.find(position);
        if(iter != surf_map.end()) {
            iter->second->plvec_tran->push_back(pvec_tran);
            if (iter->second->is2opt == 0) {
                feat_map_update_iter.push_back(iter);
                iter->second->is2opt = 1;
            }
        } else {
            OCTO_TREE* ot = new OCTO_TREE();
            ot->plvec_tran->push_back(pvec_tran);
            ot->is2opt = 2;
            ot->voxel_center[0] = (0.5+position.x) * rootSurfVoxelSize;
            ot->voxel_center[1] = (0.5+position.y) * rootSurfVoxelSize;
            ot->voxel_center[2] = (0.5+position.z) * rootSurfVoxelSize;
            ot->quater_length = rootSurfVoxelSize / 4.0;
            surf_map[position] = ot;
            feat_map_update_iter.push_back(surf_map.find(position));
        }
        static size_t cnt = 0;
        if (++cnt % 10000 == 0 && PREBUILDMAP) {
            std::cout << "\rPre-build Map Processed: " << cnt << " / " << size 
                      << " (" << (100.0 * cnt / size) << "%)" << std::flush;
        }
    });
    /****************Update voxels which has new points******************/
    std::for_each(std::execution::par, feat_map_update_iter.begin(), feat_map_update_iter.end(), [](const auto& iter) {
        iter->second->root_centors.clear();
        iter->second->recut(0, max_layer, iter->second->root_centors, 5); 
        iter->second->is2opt = 0;
    });
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
        for (int i = 0; i < size; i++) {
            pointBodyToWorld<PointType>(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        if (pcd_save_en) {
            *pcl_wait_save += *laserCloudWorld;
            static int scan_wait_num = 0;
            scan_wait_num++; 
            // if (scan_wait_num % (fix_rate*300) == 0) {
            //     PointCloudXYZI::Ptr pcl_down_world(new PointCloudXYZI());
            //     downSizeFilterPCD.setInputCloud(pcl_wait_save);
            //     downSizeFilterPCD.filter(*pcl_down_world);
            //     pcl_down_world->swap(*pcl_wait_save);
            // }
            if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval) {
                std::ostringstream oss;
                oss << std::setw(5) << std::setfill('0') << pcd_index;
                string all_points_dir(log_dir + oss.str() + ".pcd");
                pcl::PCDWriter pcd_writer;
                cout << "current scan saved to " << all_points_dir << endl;
                size = pcl_wait_save->size();
                pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudBody(new pcl::PointCloud<pcl::PointXYZI>(size, 1));
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
                for (int i = 0; i < size; i++) {
                    pointWorldToBody(&pcl_wait_save->points[i], &laserCloudBody->points[i]);
                }
                pcd_writer.writeBinary(all_points_dir, *laserCloudBody);
                pcl_wait_save->clear();
                scan_wait_num = 0;
                pcd_index ++;
                pose_json << std::fixed << std::setprecision(15);
                pose_json << lidar_end_time << " " 
                          << pos_cur(0) << " " << pos_cur(1) << " " << pos_cur(2) << " "
                          << geoQuat.w << " " << geoQuat.x << " " << geoQuat.y << " " << geoQuat.z;
                pose_json << "\n";
            }
        }
    }
}


void publish_scan_body(const ros::Publisher & pubLaserCloudFull_body) {
    int size = feats_down_body->points.size();
    PointCloudXYZI laserCloudBody;

    for (int i = 0; i < size; i++) {
        PointType point_tmp;
        RGBpointBodyLidarToIMU(&feats_down_body->points[i], &point_tmp);
        laserCloudBody.push_back(point_tmp);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(laserCloudBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);

    bool vis_norm = false;
    if (pcs && vis_norm) {
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < size; ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "body";
            marker.header.stamp = ros::Time().fromSec(lidar_end_time);
            marker.ns = "direction_vectors";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = feats_down_body->points[i].x;
            marker.pose.position.y = feats_down_body->points[i].y;
            marker.pose.position.z = feats_down_body->points[i].z;
            
            Eigen::Vector3d v1_normalized(feats_down_body->points[i].normal_x, feats_down_body->points[i].normal_y, feats_down_body->points[i].normal_z);
            if (v1_normalized.isZero()) continue; 
            Eigen::Vector3d axis = Eigen::Vector3d(1,0,0).cross(v1_normalized);
            Eigen::Quaternion<double> quat_tmp(Exp(axis));
            
            marker.pose.orientation.x = quat_tmp.x();
            marker.pose.orientation.y = quat_tmp.y();
            marker.pose.orientation.z = quat_tmp.z();
            marker.pose.orientation.w = quat_tmp.w();
            marker.scale.x = 0.5;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color = std_msgs::ColorRGBA();
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
        points_norm_pub.publish(marker_array);
    }
}

template<typename T>
void set_posestamp(T & out) {
    out.pose.pose.position.x = pos_cur(0);
    out.pose.pose.position.y = pos_cur(1);
    out.pose.pose.position.z = pos_cur(2);
    out.twist.twist.linear.x = vel_cur(0);
    out.twist.twist.linear.y = vel_cur(1);
    out.twist.twist.linear.z = vel_cur(2);
    out.pose.pose.orientation.x = geoQuat.x;
    out.pose.pose.orientation.y = geoQuat.y;
    out.pose.pose.orientation.z = geoQuat.z;
    out.pose.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P_pub();
    for (int i = 0; i < 6; i ++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath) {
    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        geometry_msgs::PoseStamped msg_body_pose;
        msg_body_pose.pose.position.x = pos_cur(0);
        msg_body_pose.pose.position.y = pos_cur(1);
        msg_body_pose.pose.position.z = pos_cur(2);
        msg_body_pose.pose.orientation.x = geoQuat.x;
        msg_body_pose.pose.orientation.y = geoQuat.y;
        msg_body_pose.pose.orientation.z = geoQuat.z;
        msg_body_pose.pose.orientation.w = geoQuat.w;
        msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
        msg_body_pose.header.frame_id = "camera_init";
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear(); 
    corr_normvect->clear();
    M3D R = s.rot.toRotationMatrix();
    V3D acc_w = R*s.acc;
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
        PointType &point_body  = feats_down_body->points[i];
        /* transform to world frame */
        double dt = point_body.curvature/double(1000);        
        V3D p_global(R*Exp(s.omg,dt)*(p_kf->Lidar_R_wrt_IMU*V3D(point_body.x, point_body.y, point_body.z) + p_kf->Lidar_T_wrt_IMU) 
                    + s.pos + s.vel*dt+ 0.5*acc_w*dt*dt);

        VOXEL_LOC position(floor(p_global(0)/rootSurfVoxelSize), floor(p_global(1)/rootSurfVoxelSize), floor(p_global(2)/rootSurfVoxelSize));
        point_selected_surf[i] = false;

        for (int k=0; k<2; k++) {
            auto iter = surf_map.find(position);
            if(iter != surf_map.end() && iter->second->root_centors.size()) {
                vector<V3D> direct_vec;
                vector<double> dist_vec;
                auto root_size = iter->second->root_centors.size();
                for (int j=0; j< root_size; j++) {
                    V3D center(iter->second->root_centors[j].x, iter->second->root_centors[j].y, iter->second->root_centors[j].z);
                    V3D direct(iter->second->root_centors[j].normal_x, iter->second->root_centors[j].normal_y, iter->second->root_centors[j].normal_z);
                    direct.normalize();
                    direct_vec.push_back(direct);
                    dist_vec.push_back(direct.dot(p_global - center));
                }
                auto it = min_element(dist_vec.begin(), dist_vec.end(), [](double a, double b) {
                                return fabs(a) < fabs(b);
                            });
                auto index = static_cast<int>(distance(dist_vec.begin(), it));
                PointType &ay = iter->second->root_centors[index];
                if(fabs(dist_vec[index]) < 0.5/*min(0.5, max(rootSurfVoxelSize/pow(2,ay.intensity+1), 0.3))*/) {
                    point_selected_surf[i] = true;
                    V3D direct = direct_vec[index];
                    normvec->points[i].x = direct(0);
                    normvec->points[i].y = direct(1);
                    normvec->points[i].z = direct(2);
                    normvec->points[i].intensity = dist_vec[index];
                }
            }
            if (point_selected_surf[i]) break;
            else if (!k) {
                int64_t loc_x = position.x;
                int64_t loc_y = position.y;
                int64_t loc_z = position.z;
                if (p_global[0] > (loc_x*rootSurfVoxelSize + 0.75*rootSurfVoxelSize)) 
                    position.x = position.x + 1;
                else if (p_global[0] < (loc_x*rootSurfVoxelSize + 0.25*rootSurfVoxelSize)) 
                    position.x = position.x - 1;
                if (p_global[1] > (loc_y*rootSurfVoxelSize + 0.75*rootSurfVoxelSize)) 
                    position.y = position.y + 1;
                else if (p_global[1] < (loc_y*rootSurfVoxelSize + 0.25*rootSurfVoxelSize)) 
                    position.y = position.y - 1;
                if (p_global[2] > (loc_z*rootSurfVoxelSize + 0.75*rootSurfVoxelSize))
                    position.z = position.z + 1;
                else if (p_global[2] < (loc_z*rootSurfVoxelSize + 0.25*rootSurfVoxelSize))
                    position.z = position.z - 1;
            }
        }
    }
    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 5) {
        ekfom_data.valid = false;
        return;
    }
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 15); //23 
    ekfom_data.h.resize(effct_feat_num);  //shm: this is the Z-h(x) vector, not h(x)
    M3D a_crossmat;
    a_crossmat<<SKEW_SYM_MATRX((Eigen::Vector3d(s.acc)));
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
    for (int i = 0; i < effct_feat_num; i++) {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this = p_kf->Lidar_R_wrt_IMU * V3D(laser_p.x, laser_p.y, laser_p.z) + p_kf->Lidar_T_wrt_IMU;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        auto norm_vecT = norm_vec.transpose();
        auto normT_R = norm_vecT*R;
        
        double dt = laser_p.curvature/double(1000);
        auto dR = Exp(s.omg, dt);
        /*** calculate the Measuremnt Jacobian matrix H ***/
        
        ekfom_data.h_x.block<1, 3>(i,0) = norm_vecT;
        M3D point_dR_crossmat;
        point_dR_crossmat<<SKEW_SYM_MATRX((dR*point_this));
        ekfom_data.h_x.block<1, 3>(i,3) = -normT_R*(point_dR_crossmat+0.5*a_crossmat*dt*dt);
        ekfom_data.h_x.block<1, 3>(i,6) = norm_vecT*dt;
        ekfom_data.h_x.block<1, 3>(i,9) = -normT_R*dR*point_crossmat*dt;
        ekfom_data.h_x.block<1, 3>(i,12) = 0.5*normT_R*dt*dt;
        ekfom_data.h(i) = -norm_p.intensity;
    }
    double weight = std::sqrt(1.0/LASER_POINT_COV);
    ekfom_data.h = ekfom_data.h*weight;
    ekfom_data.h_x = ekfom_data.h_x*weight;
}

void pcl_normal_cal(PointCloudXYZI::Ptr &feats_down_body) {    
    kdtreeNorm->setInputCloud(feats_down_body);
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
    for (int i=0; i<feats_down_body->points.size(); i++) {
        auto &pt = feats_down_body->points[i];
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        auto radiusSearch = 0.5 + 0.045*std::clamp(std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) - p_pre->blind, 0.0, 100.0); //[Update]: dynamic search radius
        kdtreeNorm->radiusSearch(pt, radiusSearch, pointSearchInd, pointSearchSqDis);
        auto np_size = pointSearchInd.size();
        if (np_size<5) continue;
        pcl::PointCloud<pcl::PointXYZ> near_points;
        near_points.points.resize(np_size);
        for (int j = 0; j < np_size; ++j) {
            int id = pointSearchInd[j];
            near_points.points[j].x = feats_down_body->points[id].x;
            near_points.points[j].y = feats_down_body->points[id].y;
            near_points.points[j].z = feats_down_body->points[id].z;
        }
        Eigen::Matrix3f cov;
        Eigen::Vector4f centroid;
        pcl::computeMeanAndCovarianceMatrix(near_points, cov, centroid);
                
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(cov);
        if (saes.eigenvalues()[1]/saes.eigenvalues()[0]>=9.0 || saes.eigenvalues()[0] < 0.01) {
            Eigen::Vector3f direct_vec = saes.eigenvectors().col(0);
            pt.normal_x = direct_vec(0);
            pt.normal_y = direct_vec(1);
            pt.normal_z = direct_vec(2);
        }
    }
}

void pcl_redund_mini(PointCloudXYZI::Ptr &feats_down_body) {
    int cloud_size = feats_down_body->size();
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_p = MatrixXd::Zero(cloud_size, 3); 
    Eigen::Matrix<double, Eigen::Dynamic, 3> J_r = MatrixXd::Zero(cloud_size, 3);
    pcl_normal_cal(feats_down_body);
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif    
    for (int kk=0; kk<cloud_size; kk++) {
        Eigen::Vector3d C = Eigen::Vector3d(feats_down_body->points[kk].normal_x, feats_down_body->points[kk].normal_y, feats_down_body->points[kk].normal_z);
        if (!C.isZero()) {
            V3D P = V3D(feats_down_body->points[kk].x, feats_down_body->points[kk].y, feats_down_body->points[kk].z);
            V3D P_body = p_kf->Lidar_R_wrt_IMU*P + p_kf->Lidar_T_wrt_IMU;
            double dt = feats_down_body->points[kk].curvature/double(1000);
            M3D R_p = state_point.rot.toRotationMatrix()*Exp(state_point.omg, dt);

            M3D point_crossmat;
            point_crossmat<<SKEW_SYM_MATRX(P_body);
            J_p.block<1, 3>(kk,0) = C.transpose()*R_p.transpose(); //body frame
            Eigen::Matrix<double, 1, 3> j_r = -J_p.block<1, 3>(kk,0)*point_crossmat;
            j_r.normalize();
            J_r.block<1, 3>(kk,0) = j_r; // world frame
        }
    }
    Eigen::Matrix<double, 3, 3> H_p = J_p.transpose()*J_p;    
    Eigen::Matrix<double, 3, 3> H_r = J_r.transpose()*J_r;

    static std::deque<Eigen::Matrix<double, Eigen::Dynamic, 3>> J_p_vec;
    static std::deque<Eigen::Matrix<double, Eigen::Dynamic, 3>> J_r_vec;

    for (int kk=0; kk<J_p_vec.size(); kk++) {
        H_p += J_p_vec[kk].transpose()*J_p_vec[kk];
        H_r += J_r_vec[kk].transpose()*J_r_vec[kk];
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_p(H_p);
    auto V_p = saes_p.eigenvectors(); 
    auto I_p = J_p*V_p; //N*3
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_r(H_r);
    auto V_r = saes_r.eigenvectors(); 
    auto I_r = J_r*V_r; //N*3

    Eigen::Matrix<double, 1, 3> L_p = MatrixXd::Zero(1, 3);
    Eigen::Matrix<double, 1, 3> L_r = MatrixXd::Zero(1, 3);
    for (int kk=0; kk<J_p_vec.size(); kk++) {
        auto I_p_his = J_p_vec[kk]*V_p;
        auto I_p_his_abs = I_p_his.array().abs();
        auto I_r_his = J_r_vec[kk]*V_r;
        auto I_r_his_abs = I_r_his.array().abs();
        Eigen::Matrix<double, 1, 3> L_p_his = I_p_his_abs.colwise().sum();
        L_p += L_p_his;
        Eigen::Matrix<double, 1, 3> L_r_his = I_r_his_abs.colwise().sum();
        L_r += L_r_his;
    }

    Eigen::Matrix<double, Eigen::Dynamic, 6> I = MatrixXd::Zero(cloud_size, 6);
    I.block(0,0,cloud_size,3) = I_p;
    I.block(0,3,cloud_size,3) = I_r;
    auto I_abs = I.array().abs();
    auto I_c = (I_abs.array() < 0.34).select(0, I_abs);    //0.087=cos(85) 0.34=cos(70)
    auto I_s = (I_c.array() < 0.87).select(0, I_c);        //0.87 = cos(30)
    Eigen::Matrix<double, 1, 6> L_s = I_s.colwise().sum();
    Eigen::Matrix<double, 1, 6> L_his;
    L_his.block<1,3>(0,0) = L_p;
    L_his.block<1,3>(0,3) = L_r;
    L_s += L_his;
    
    PointCloudXYZI::Ptr feats_rms_body(new PointCloudXYZI());
    std::vector<std::vector<int>> index(6);
    std::unordered_set<int> mergedSet;
    std::vector<double> record_score(6);
    for (int kk=0; kk<6; kk++) {
        double score = L_his(0,kk);//0.0;
        record_score[kk] = L_his(0,kk);
        if (L_s(0,kk) < k1) {
            Eigen::VectorXd col = I_c.col(kk);
            for (int i = 0; i < col.size(); i++) {
                if (col(i)) {
                    index[kk].push_back(i);
                    score+=col(i);
                    record_score[kk]+=col(i);
                }
            } 
        } else {
            std::vector<std::pair<int,double>> index_tmp;
            Eigen::VectorXd col = I_s.col(kk);
            for (int i = 0; i < col.size(); i++) {
                if (col(i)) 
                    index_tmp.push_back(std::make_pair(i,col(i)));
            } 
            sort(index_tmp.begin(), index_tmp.end(),norm_list);
            index[kk].clear();
            for (int i = 0; i < index_tmp.size(); i++) {
                if(score > k1) break;
                index[kk].push_back(index_tmp[i].first);
                score += index_tmp[i].second;
                record_score[kk]+=index_tmp[i].second;
            } 
        }
        std::transform(index[kk].begin(), index[kk].end(), std::inserter(mergedSet, mergedSet.end()),
                        [](const int& idx) { return idx;});
    }    
    std::vector<int> mergedIndex(mergedSet.begin(), mergedSet.end());
    pcl::copyPointCloud(*feats_down_body, mergedIndex, *feats_rms_body);
    Eigen::MatrixXd J_p_new(mergedIndex.size(), 3);
    Eigen::MatrixXd J_r_new(mergedIndex.size(), 3);
    for (int i = 0; i < mergedIndex.size(); ++i) {
        J_p_new.row(i) = J_p.row(mergedIndex[i]);
        J_r_new.row(i) = J_r.row(mergedIndex[i]);
    }
    J_p_vec.push_back(J_p_new);
    J_r_vec.push_back(J_r_new);
    while(int(J_p_vec.size())>(rms_win_size-1)) {
        J_p_vec.pop_front();
        J_r_vec.pop_front();
    }
    feats_down_body->swap(*feats_rms_body);
    feats_down_size = feats_down_body->points.size();
}

void lioThread() {
    lidar_mean_scantime = 1.0/fix_rate;
    // ros::Rate rate(fix_rate);
    ros::Rate rate(5e3);
    bool status = ros::ok();
    while (status) {
        mtx_buffer.lock();
        double t_sync = omp_get_wtime();
        bool sync = sync_packages(Measures);
        t_sync = omp_get_wtime() - t_sync;
        mtx_buffer.unlock();
        if(sync) {
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                p_kf->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                rate.sleep();
                continue;
            }

            double t0,t1,t2,t3,t4,t5,solve_start;

            t0 = omp_get_wtime();
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            p_kf->Process(Measures, kf, feats_undistort, flg_EKF_inited);
            state_point = kf.get_x();

            if (feats_undistort->empty() || (feats_undistort == NULL)) {
                rate.sleep();
                continue;
            }
            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();
            
            if(!surf_map.size() || !flg_EKF_inited) {
                if(feats_down_size > 5 && !PREBUILDMAP) {
#ifdef VIS_VOXEL_MAP
                    mtx_map.lock();
#endif
                    cut_voxel<PointType>(surf_map, feats_down_body);
#ifdef VIS_VOXEL_MAP
                    mtx_map.unlock();
#endif
                }
                rate.sleep();
                continue;
            }

            if (pcs && ((Measures.lidar_beg_time - first_lidar_time) > 1)) 
                pcl_redund_mini(feats_down_body); 
            t1 = omp_get_wtime();
            
            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5) {
                rate.sleep();
                continue;
            }

            normvec->resize(feats_down_size);

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(solve_H_time);
            state_point = kf.get_x();

            kf.predict_pub(lidar_mean_scantime, p_kf->Q);
            auto state_pub = kf.get_x_pub();
            pos_cur = state_pub.pos;
            vel_cur = state_pub.vel;
            geoQuat.x = state_pub.rot.coeffs()[0];
            geoQuat.y = state_pub.rot.coeffs()[1];
            geoQuat.z = state_pub.rot.coeffs()[2];
            geoQuat.w = state_pub.rot.coeffs()[3];
            double t_update_end = omp_get_wtime();
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            if (!PREBUILDMAP) {
#ifdef VIS_VOXEL_MAP
                mtx_map.lock();
#endif
                cut_voxel<PointType>(surf_map, feats_down_body);
#ifdef VIS_VOXEL_MAP
                mtx_map.unlock();
#endif
            }

            t5 = omp_get_wtime();

            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_scan_body(pubLaserCloudFull_body);
            /*** Debug variables ***/
            if (runtime_pos_log) {
                s_plot[time_log_counter] = (t1-t0)*1e3; //DS
                s_plot1[time_log_counter] = (t_update_end-t_update_start)*1e3; //Estimation
                s_plot2[time_log_counter] = (t5-t3)*1e3; //Mapping
                s_plot3[time_log_counter] = (t5-t0)*1e3; //Total
                time_log_counter ++;
                dump_mlo_state_to_log(fp);
            }
        }
        status = ros::ok();
        rate.sleep();
    }
}

void visThread() {
    ros::Rate rate(10);
    while (ros::ok()) {
        mtx_map.lock();
        pubVoxelMap(surf_map, 1, voxel_map_pub);
        mtx_map.unlock();
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);

    string offline_gt_path;
    nh.param<string>("mapping/offline_gt_path",offline_gt_path,"");
    nh.param<int>("common/lidar_num", lidar_num, 1);
    std::vector<std::string> lid_topic(lidar_num);
    nh.param<std::vector<std::string>>("common/lid_topic",lid_topic,std::vector<std::string>());
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("mapping/root_surf_voxel_size",rootSurfVoxelSize,1.0);
    nh.param<int>("mapping/max_layer",max_layer, 1.0);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/pc_cov", LASER_POINT_COV,0.001);
    nh.param<bool>("mapping/PCS", pcs, false);
    nh.param<int>("mapping/rms_win_size", rms_win_size, 10);
    nh.param<int>("mapping/rate", fix_rate, 50);
    p_pre = std::make_shared<Preprocess>();
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<std::vector<int>>("preprocess/lidar_type", p_pre->lidar_type, std::vector<int>());
    nh.param<std::vector<int>>("preprocess/timestamp_unit", p_pre->time_unit, std::vector<int>());
    nh.param<std::vector<double>>("preprocess/header_time_offset", p_pre->header_time_offset, std::vector<double>());    
    nh.param<std::vector<int>>("preprocess/point_filter_num", p_pre->point_filter_num, std::vector<int>());
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<double>("mapping/k1", k1, 180);
   
    nh.param<bool>("mapping/use_prebuild_map", PREBUILDMAP, false);
    std::vector<double> init_T;
    nh.param<vector<double>>("mapping/init_T", init_T, std::vector<double>());
    p_kf->init_pose<<MAT_FROM_ARRAY(init_T);
    if (PREBUILDMAP) {
        std::string offline_map_path;
        nh.param<std::string>("mapping/offline_map_path", offline_map_path, "");
        cout << "Loading Pre-build Map PCD ..." << endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(offline_map_path, *pre_build_cloud) == -1) {
            PCL_ERROR("Couldn't read Pre-build Map.\n");
        }
        cut_voxel<pcl::PointXYZI>(surf_map, pre_build_cloud);
        PCL_ERROR("The Pre-build Map has been Voxelized.\n");
    }
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";
    path_gt.header.stamp    = ros::Time::now();
    path_gt.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0;
    double deltaT, deltaR;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    downSizeFilterSurf.setRadiusSearch(filter_size_surf_min);
    // downSizeFilterPCD.setRadiusSearch(0.2);

    double epsi[15];
    fill(epsi, epsi+15, 0.001);
    kdtreeNorm.reset(new pcl::KdTreeFLANN<PointType>());
    
    
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    
    /*** debug record ***/
    log_dir = root_dir + "Log/";
    if (!filesystem::exists(log_dir)) filesystem::create_directories(log_dir);
    string pos_log_dir = log_dir+"pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    pose_json.open(log_dir + "pose.json", std::ofstream::trunc);
    pose_json.close();
    pose_json.open(log_dir + "pose.json", std::ofstream::app);

    lidar_msg_buffer.resize(lidar_num);
    Measures.lidar.resize(lidar_num);
    std::string laserBodyTopic = "/cloud_registered_body";    
    std::vector<ros::Subscriber> sub_pcl(lidar_num);
    for (int num = 0; num<lidar_num; num++) {
        Eigen::Vector3d extrinsic_Tn(extrinT.at(num * 3), extrinT.at(num * 3 + 1), extrinT.at(num * 3 + 2));
        Textrinsic.push_back(extrinsic_Tn);
        Eigen::Quaterniond extrinsic_Qn(extrinR.at(num * 4), extrinR.at(num * 4 + 1), extrinR.at(num * 4 + 2), extrinR.at(num * 4 + 3)); //w,x,y,z
        Rextrinsic.push_back(extrinsic_Qn.toRotationMatrix());
        if (p_pre->lidar_type[num]==AVIA) 
            sub_pcl[num] = nh.subscribe<livox_ros_driver::CustomMsg>(lid_topic.at(num), 20000,
                        [num](const livox_ros_driver::CustomMsg::ConstPtr& msg) {
                            livox_pcl_cbk(msg, num);
                        });
        else if (p_pre->lidar_type[num]==AVIA2) 
            sub_pcl[num] = nh.subscribe<livox_ros_driver2::CustomMsg>(lid_topic.at(num), 20000,
                        [num](const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
                            livox2_pcl_cbk(msg, num);
                        });
        else
            sub_pcl[num] = nh.subscribe<sensor_msgs::PointCloud2>(lid_topic.at(num), 20000,
                                [num](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                                    standard_pcl_cbk(msg, num);
                                });
        
        
    }
    p_kf->set_extrinsic(Textrinsic[0], Rextrinsic[0]);
    p_kf->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_kf->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_kf->Reset();
    
    /*** ROS subscribe initialization ***/
    pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2> (laserBodyTopic, 100000);
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2> ("/Laser_map", 100000);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/Odometry", 1);
    ros::Publisher pubOffilineMap = nh.advertise<sensor_msgs::PointCloud2> ("/offline_cloud", 10000);
    ros::Publisher pubOffilineMagMap = nh.advertise<sensor_msgs::PointCloud2> ("/offline_mag_map", 1);
    pubPath = nh.advertise<nav_msgs::Path> ("/path", 100000);
    pubgtPath = nh.advertise<nav_msgs::Path> ("/path_gt", 100000);
    voxel_map_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxel_planes", 10000);
    points_norm_pub = nh.advertise<visualization_msgs::MarkerArray>("/point_norm", 1);
    
//------------------------------------------------------------------------------------------------------
    std::thread liothread(lioThread);
#ifdef VIS_VOXEL_MAP
    std::thread visthread(visThread);
#endif
    ros::MultiThreadedSpinner spinner(12);
    spinner.spin();
    liothread.join();
#ifdef VIS_VOXEL_MAP
    visthread.join();
#endif
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = "scans.pcd";
        string all_points_dir(log_dir + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << all_points_dir <<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log) {
        FILE *fp2;
        string time_log_dir = log_dir + "ctemlo_time_log.csv";
        fp2 = fopen(time_log_dir.c_str(),"w");
        fprintf(fp2,"Downsample, ICP, map_incre, total\n");
        for (int i = 0;i<time_log_counter; i++) {
            fprintf(fp2,"%0.8f,%0.8f,%0.8f,%0.8f\n",s_plot[i],s_plot1[i],s_plot2[i],s_plot3[i]);
        }
        fclose(fp2);
    }
    pose_json.close();
    return 0;
}
