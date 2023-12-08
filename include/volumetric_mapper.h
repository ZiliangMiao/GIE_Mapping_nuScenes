
#ifndef SRC_VOLUMETRIC_MAPPER_H
#define SRC_VOLUMETRIC_MAPPER_H


#include <ros/ros.h>
#include <GIE/CostMap.h>
#include "cuda_toolkit/occupancy/realsense/realsense_map_maker.h"
#include "cuda_toolkit/occupancy/hokuyo/hokuyo_map_maker.h"
#include "cuda_toolkit/occupancy/point_cloud/pntcld_map_maker.h"
#include "cuda_toolkit/occupancy/vlp16/vlp16_map_maker.h"
#include "cuda_toolkit/edt/edt_interfaces.h"
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/TransformStamped.h"
#include <chrono>
#include "parameters.h"
#include "par_wave/glb_hash_map.h"
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf_conversions/tf_eigen.h"
#include "std_srvs/SetBool.h"
#include <mutex>
#include "simple_logger.h"
#include "gt_checker.h"
#include "map_structure/local_batch.h"
#include <string>
#include <tf/transform_broadcaster.h>

#include "map_structure/pre_map.h"


/* PCL头文件 */
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class VOLMAPNODE
{
    typedef pcl::PointCloud<pcl::PointXYZ> PntCld;
    typedef pcl::PointCloud<pcl::PointXYZI> PntCldI;

    struct MsgMgr
    {
        bool got_odom = false;
        bool got_dep_img = false;
        bool got_pnt_cld = false;
        bool got_scan = false;

        bool is_ready()
        {
            return got_odom && (got_dep_img || got_pnt_cld || got_scan);
        }
    };

public:
    VOLMAPNODE();
    //---
    ~VOLMAPNODE();
    //---

private:
    //---
    void publishMap(const ros::TimerEvent&);
    void CB_odom(const nav_msgs::Odometry::ConstPtr &msg);
    void CB_scan2D(const sensor_msgs::LaserScan::ConstPtr& msg);
    void CB_depth(const sensor_msgs::Image::ConstPtr& msg);
    void CB_caminfo(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void CB_pntcld(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void CB_pntcld_odom(const sensor_msgs::PointCloud2::ConstPtr &pcld, const nav_msgs::Odometry::ConstPtr &odom);
    void CB_cow(const sensor_msgs::PointCloud2::ConstPtr &pcld, const geometry_msgs::TransformStamped::ConstPtr &trfm);
    void CB_depth_odom(const sensor_msgs::Image::ConstPtr &img, const nav_msgs::Odometry::ConstPtr &odom);
    void CB_scan_odom(const sensor_msgs::LaserScan::ConstPtr &scan, const nav_msgs::Odometry::ConstPtr &odom);
    void setupRotationPlan();
    tf::Transform odom2trans();
    void setupEDTmsg4Motion(GIE::CostMap &msg, LocMap* loc_map, bool resize);


    void clustring(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void CB_ext_cld(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void update_ext_map();

private:
    ros::NodeHandle _nh;
    ros::Subscriber _caminfo_sub;


    message_filters::Subscriber<nav_msgs::Odometry> s_odom_sub;
    message_filters::Subscriber<geometry_msgs::TransformStamped> s_trfm_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> s_pntcld_sub;
    message_filters::Subscriber<sensor_msgs::Image> s_depth_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> s_laser_sub;

    // msg sync for pntcld
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> pntcld_sync_policy;
    message_filters::Synchronizer<pntcld_sync_policy> *pntcld_sync;

    // msg sync for cow_lady
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> cow_sync_policy;
    message_filters::Synchronizer<cow_sync_policy> *cow_sync;

    // msg sync for depth cam
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> depth_sync_policy;
    message_filters::Synchronizer<depth_sync_policy> *depth_sync;

    // msg sync for laser2D
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> laser2D_sync_policy;
    message_filters::Synchronizer<laser2D_sync_policy> *laser2D_sync;

    // receive external knowledge form pyntcld
    ros::Subscriber ext_cld_sub;

    ros::Timer _mapTimer;
    ros::Publisher edt_msg_pub;
    GIE::CostMap cost_map_msg;

    // Map updater
    HokuyoMapMaker _hok_map_maker;
    RealsenseMapMaker _rea_map_maker;
    PntcldMapMaker _pnt_map_maker;
    Vlp16MapMaker _vlp_map_maker;

    // Pointer to store the laser & pose
    boost::shared_ptr<sensor_msgs::LaserScan> _laser_ptr;
    boost::shared_ptr<nav_msgs::Odometry>  _odom_ptr;
    boost::shared_ptr<sensor_msgs::Image> _depth_ptr;
    boost::shared_ptr<sensor_msgs::PointCloud2> _pntcld_ptr;
    pcl::PointCloud<pcl::PointXYZ> ext_cloud;


    // glb EDT
    GlbHashMap *_hash_map;

    // local and global map
    LocMap *_loc_map;

    // Rotation plan
    cuttHandle _rotation_plan[3];

    // Mapping cycle
    int _time;
    MsgMgr _msg_mgr;

    PntCldI::Ptr _occ_pnt_cld;
    PntCldI::Ptr _edt_pnt_cld;
    PntCldI::Ptr _glb_edt_pnt_cld;
    PntCld::Ptr _glb_ogm_pnt_cld;
    PntCldI::Ptr _dbg_pnt_cld;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pnt_cld_save;  // pnt cld for store [miaozl]
    std::vector<std::vector<float>> pnt_cld_xy {{0.0, 0.0}};  // std::vector, only save x and y coord, for indexing

    ros::Publisher  _edt_rviz_pub;
    ros::Publisher  _occ_rviz_pub;
    ros::Publisher _glb_edt_rviz_pub;
    ros::Publisher _glb_ogm_rviz_pub;
    ros::Publisher _dbg_rviz_pub;

    // input from ros
    Parameters param;


    // profiler
    csvfile *logger;
    Gnd_truth_checker *gtc;

    // for tf pub
    tf::TransformBroadcaster br;
    ros::Time cur_stamp;

    // Ext_Obs_Wrapper
    Ext_Obs_Wrapper *ext_obs;

    /* PCL bbox calculate */
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;

    // org local vis
    void publish_local_ptcld_2_rviz(int scan_idx)
    {
        if(!param.for_motion_planner)
        {
            _loc_map->copy_ogm_2_host();
            _loc_map->copy_edt_2_host();
        }
        int3 crd;
        float3 pos;
        int idx;
        for (crd.x=0;crd.x<_loc_map->_local_size.x;crd.x++)
        {
            for (crd.y=0;crd.y<_loc_map->_local_size.y;crd.y++)
            {
                for (crd.z=0;crd.z<_loc_map->_local_size.z;crd.z++)
                {
                    idx = _loc_map->id(crd.x,crd.y,crd.z);
                    if (_loc_map->glb_type_H[idx]==VOXTYPE_UNKNOWN)
                        continue;

                    pos = _loc_map->coord2pos(_loc_map->loc2glb(crd));

                    if(param.display_loc_ogm && _loc_map->glb_type_H[idx] == VOXTYPE_OCCUPIED)
                    {
                        pcl::PointXYZI occu_pt;
                        occu_pt.x = pos.x;
                        occu_pt.y = pos.y;
                        occu_pt.z = pos.z;
                        occu_pt.intensity = _loc_map->glb_type_H[idx];
                        _occ_pnt_cld->points.push_back (occu_pt);
                    }

                    if(param.display_loc_edt)
                    {
                        float dist = _loc_map->edt_H[idx]*param.voxel_width;
                        pcl::PointXYZI ptXYZI;
                        ptXYZI.x = pos.x;
                        ptXYZI.y = pos.y;
                        ptXYZI.z = pos.z;
                        ptXYZI.intensity = dist;
                        _edt_pnt_cld->points.push_back (ptXYZI);
                    }
                }
            }
        }
        
        if (param.save_loc_edt)
        {
            // save local edt to .bin file
            // _edt_pnt_cld->height = 1;
            // _edt_pnt_cld->width = _edt_pnt_cld->points.size();
            // pcl::io::savePCDFile<pcl::PointXYZI> (, *_edt_pnt_cld);
            std::string local_esdf_file = param.save_esdf_dir + "/local_esdf_" + std::to_string(scan_idx) + ".bin";
            std::ofstream fout(local_esdf_file, std::ios::binary);
            printf("Save local edt point cloud to %s\n", local_esdf_file.c_str());
            for (int i = 0; i < _edt_pnt_cld->points.size(); i++)
            {
                fout.write((char *) &_edt_pnt_cld->points[i].x, sizeof(float));
                fout.write((char *) &_edt_pnt_cld->points[i].y, sizeof(float));
                fout.write((char *) &_edt_pnt_cld->points[i].z, sizeof(float));
                fout.write((char *) &_edt_pnt_cld->points[i].intensity, sizeof(float));
            }
            fout.close();
        }

        if(param.display_loc_ogm)
        {
            std::cout << "Local OGM Cloud Size: " << _occ_pnt_cld->size() << std::endl;
            pcl_conversions::toPCL(ros::Time::now(), _occ_pnt_cld->header.stamp);
            _occ_rviz_pub.publish (_occ_pnt_cld);
            _occ_pnt_cld->clear();
        }
        if(param.display_loc_edt)
        {
            std::cout << "Local EDT Cloud Size: " << _edt_pnt_cld->size() << std::endl;
            pcl_conversions::toPCL(ros::Time::now(), _edt_pnt_cld->header.stamp);
            _edt_rviz_pub.publish (_edt_pnt_cld);
            _edt_pnt_cld->clear();
        }
    }

    // [org] global vis
    void publish_global_ptcld_2_rviz(int vis_coord_z, int scan_idx)
    {   
        for (int blk_cnt =0; blk_cnt< _hash_map->VB_cnt_H; blk_cnt++) {
            int3 blk_key = _hash_map->VB_keys_H[blk_cnt];
            if(invalid_blk_key(blk_key))
                continue;

            int3 blk_offset = make_int3(blk_key.x*VB_WIDTH, blk_key.y*VB_WIDTH, blk_key.z*VB_WIDTH);

            for(int idx_1d =0; idx_1d<VB_SIZE; idx_1d++) {
                GlbVoxel vox = _hash_map->VB_values_H[blk_cnt].voxels[idx_1d];
                if(vox.vox_type==VOXTYPE_UNKNOWN)
                    continue;
                int3 vox_crd = reconstruct_vox_crd(blk_offset,idx_1d);
                if(param.display_glb_ogm)
                {
                    if(vox.vox_type==VOXTYPE_OCCUPIED)
                    {   
                        float3 gpos = _loc_map->coord2pos(vox_crd);
                        // std::cout << "Voxel occupied: " << gpos.x << " " << gpos.y << " " << gpos.z << std::endl;
                        pcl::PointXYZ OccuPtXYZ;
                        OccuPtXYZ.x = gpos.x;
                        OccuPtXYZ.y = gpos.y;
                        OccuPtXYZ.z = gpos.z;
                        _glb_ogm_pnt_cld->points.push_back(OccuPtXYZ);
                    }
                }

                if(param.display_glb_edt) {
                    if(param.vis_and_save_slice) {
                        if(vox_crd.z != vis_coord_z)
                            continue;
                    }
                    if(invalid_dist_glb(vox.dist_sq))
                        continue;

                    float3 gpos = _loc_map->coord2pos(vox_crd);
                    pcl::PointXYZI ptXYZI;
                    ptXYZI.x = gpos.x;
                    ptXYZI.y = gpos.y;
                    ptXYZI.z = 0;
                    ptXYZI.intensity = std::sqrt(vox.dist_sq)*param.voxel_width;
                    // std::cout << "Voxel distance: " << ptXYZI.intensity << std::endl;

                    _glb_edt_pnt_cld->points.push_back(ptXYZI);
                    
                    // accumulate z-axis
                    // bool found = false;
                    // for (size_t pnt_idx = 0; pnt_idx < _glb_edt_pnt_cld->points.size(); pnt_idx++) {
                    //     float x = _glb_edt_pnt_cld->points[pnt_idx].x;
                    //     float y = _glb_edt_pnt_cld->points[pnt_idx].y;
                    //     if (x == ptXYZI.x && y == ptXYZI.y) {
                    //         // If the point was found in the point cloud, accumulate its esdf distance
                    //         found = true;
                    //         _glb_edt_pnt_cld->points[pnt_idx].intensity += ptXYZI.intensity;
                    //         break;
                    //     }
                    // }
                    // if (!found) {
                    //     // If the point was not found, push back the point
                    //     _glb_edt_pnt_cld->points.push_back(ptXYZI);
                    // }

                    // printf("debug flag 1\n");
                    // std::vector<float> pnt_xy = {gpos.x, gpos.y};
                    // std::cout << pnt_xy[0] << " " << pnt_xy[1] << std::endl;
                    // auto it = find(pnt_cld_xy.begin(), pnt_cld_xy.end(), pnt_xy);
                    // printf("debug flag 2");
                    // if (it != pnt_cld_xy.end()) {
                    //     // If the point was found in the point cloud, accumulate its esdf distance
                    //     int pnt_idx = it - pnt_cld_xy.begin(); 
                    //     std::cout << "point was found in the point cloud, accumulate esdf distance: " << pnt_idx << " " << ptXYZI.intensity << std::endl; 
                    //     pnt_cld_save->points[pnt_idx].intensity += ptXYZI.intensity;
                    // } 
                    // else { 
                    //     // If the point was not found, push back the point
                    //     pnt_cld_save->points.push_back(ptXYZI);
                    //     pnt_cld_xy.push_back(pnt_xy);
                    //     std::cout << "point was found in the point cloud, push back" << std::endl;
                    // }
                }
            }
        }

        // save global edt
        if (param.save_glb_edt) {
            std::string global_esdf_file = param.save_esdf_dir + "/global_esdf_" + std::to_string(scan_idx) + ".bin";
            std::ofstream fout(global_esdf_file, std::ios::binary);
            printf("Save global edt point cloud to %s\n", global_esdf_file.c_str());
            for (int i = 0; i < _glb_edt_pnt_cld->points.size(); i++)
            {
                fout.write((char *) &_glb_edt_pnt_cld->points[i].x, sizeof(float));
                fout.write((char *) &_glb_edt_pnt_cld->points[i].y, sizeof(float));
                fout.write((char *) &_glb_edt_pnt_cld->points[i].z, sizeof(float));
                fout.write((char *) &_glb_edt_pnt_cld->points[i].intensity, sizeof(float));
            }
            fout.close();
        }

        // display
        if(param.display_glb_ogm)
        {
            std::cout << "Global OGM Cloud Size: " << _glb_ogm_pnt_cld->size() << std::endl;
            pcl_conversions::toPCL(ros::Time::now(), _glb_ogm_pnt_cld->header.stamp);
            _glb_ogm_rviz_pub.publish(_glb_ogm_pnt_cld);
            _glb_ogm_pnt_cld->clear();
        }
        if(param.display_glb_edt)
        {
            std::cout << "Global EDT Cloud Size: " << _glb_edt_pnt_cld->size() << std::endl;
            pcl_conversions::toPCL(ros::Time::now(), _glb_edt_pnt_cld->header.stamp);
            _glb_edt_rviz_pub.publish(_glb_edt_pnt_cld);
            _glb_edt_pnt_cld->clear();
        }
    }

    void visualize(float3 vis_pos, int scan_idx)
    {
        // if(param.profile_glb_rms || param.profile_loc_rms)
        // {
        //     //pause rosbag
        //     std_srvs::SetBool pausesrv;
        //     pausesrv.request.data = true;
        //     pausesrv.response.message ="rosbag paused!";
        //     ros::service::call("/profile_bag/pause_playback",pausesrv);
        //     _mapTimer.stop();
        // }

        // rviz display
        if (param.display_loc_ogm || param.display_loc_edt)
        {   
            publish_local_ptcld_2_rviz(scan_idx);
        }
        if (param.display_glb_edt || param.display_glb_ogm)
        {
            vis_pos.z = param.vis_height;  // question mark [miaozl]
            int3 vis_coord = _loc_map->pos2coord(vis_pos);
            publish_global_ptcld_2_rviz(vis_coord.z, scan_idx);
        }

        // profile
        // if (param.profile_glb_rms || param.profile_loc_rms)
        // {
        //     float rmse = gtc->cmp_dist();
        //     (*logger)<<rmse;

        //     //resume rosbag
        //     std_srvs::SetBool resumesrv;
        //     resumesrv.request.data = false;
        //     resumesrv.response.message ="rosbag resume!";
        //     ros::service::call("/profile_bag/pause_playback",resumesrv);
        //     _mapTimer.start();
        // }
    }
};

#endif //SRC_VOLUMETRIC_MAPPER_H
