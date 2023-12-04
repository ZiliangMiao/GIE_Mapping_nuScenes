
#ifndef SRC_VLP16_MAP_MAKER_H
#define SRC_VLP16_MAP_MAKER_H

#include "multiscan_param.h"
#include <cuda_toolkit/projection.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include "map_structure/local_batch.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Vlp16MapMaker
{
public:
    Vlp16MapMaker();
    ~Vlp16MapMaker();

    void initialize(const MulScanParam &p);
    bool is_initialized(){return _initialized;}

    void setLocMap(LocMap *lMap);
    void updateLocalOGM(const Projection& proj, const sensor_msgs::PointCloud2ConstPtr& pyntcld,
                        int3* VB_keys_loc_D, const int time,  bool for_motion_planner, int rbt_r2_grids);
    void convertPyntCld(const sensor_msgs::PointCloud2ConstPtr& msg);  // VLP 16
    void convertPcl(const sensor_msgs::PointCloud2ConstPtr& msg);  // NUSC 32

private:
    MulScanParam _mul_scan_param;
    int _range_byte_sz;
    SCAN_DEPTH_TPYE *_gpu_mulscan;
    bool _initialized = false;
    sensor_msgs::LaserScan  scanlines[32];
    // const int rayid_toup[32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15, 
    // 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};  // question mark ?
    // const int rayid_toup[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};  // VLP 16
    
    LocMap * _lMap;
};

#endif //SRC_VLP16_MAP_MAKER_H
