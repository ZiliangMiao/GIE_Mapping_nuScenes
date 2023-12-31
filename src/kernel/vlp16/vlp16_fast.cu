
#include "vlp16_helper.h"
#include "vlp16_interface.h"
#include "par_wave/voxmap_utils.cuh"

namespace VLP_FAST
{
    __global__
    void setLocalOccupancy(LocMap loc_map,
                           LASER_RANGE_TPYE *detph_data,
                           Projection proj,
                           MulScanParam param,
                           int3* VB_keys_loc_D,
                           bool for_motion_planner,
                           int rbt_r2_grids)
    {
        int3 local_crd, glb_crd;
        local_crd.z = blockIdx.x;
        local_crd.y = threadIdx.x;

        float idea_depth, real_depth;  // idea depth: voxel depth; real depth: lidar depth
        int theta_idx, phi_idx;
        float3 glb_pos;

        for (local_crd.x = 0; local_crd.x < loc_map._local_size.x; ++local_crd.x)
        {   // traverse all voxels in x direction
            int idx_1d = loc_map.coord2idx_local(local_crd);  // local_crd: voxel
            glb_crd = loc_map.loc2glb(local_crd);  // glb_crd: voxel
            
            // set the self pose as known
            if(for_motion_planner)
            {
                int3 crd2center = local_crd -loc_map._half_shift;
                if(crd2center.x*crd2center.x + crd2center.y*crd2center.y+crd2center.z*crd2center.z <= rbt_r2_grids)
                {
                    loc_map.set_vox_type(local_crd,VOXTYPE_FREE);
                    VB_keys_loc_D[idx_1d] = get_VB_key(glb_crd);
                    continue;
                }
            }
            
            glb_pos = loc_map.coord2pos(glb_crd);  // glb_pos: meter

            // to get: theta_idx, phi_idx, idea_depth
            VLP_HELPER::G2L(glb_pos, proj, param ,loc_map._voxel_width, theta_idx, phi_idx, idea_depth);

            if (idea_depth < 0 || theta_idx < 0 || theta_idx >= param.scan_num)
            {
                VB_keys_loc_D[idx_1d] = EMPTY_KEY;
                continue;
            }

            // get the laserpoint id from phi_idx and theta_idx
            int pt_id_on_scans = phi_idx * param.scan_num + theta_idx;  // phi_idx: elevation; theta_idx: azimuth

            // get the actual measurement depth
            real_depth = detph_data[pt_id_on_scans];

            if (isnan(real_depth) || real_depth <= 0.1f)
            {
                VB_keys_loc_D[idx_1d] = EMPTY_KEY;
                continue;
            }

            // ray-casting to update the OGM
            if (idea_depth < real_depth - 0.1f)
            {
                loc_map.set_vox_type(local_crd, VOXTYPE_FREE);
                VB_keys_loc_D[idx_1d] = get_VB_key(glb_crd);
            }
            else if (idea_depth > real_depth + 0.1f)
            {   // Not observable: voxel type = unknown
                VB_keys_loc_D[idx_1d] = EMPTY_KEY;
            }
            else if(glb_pos.z >= loc_map._update_min_h && glb_pos.z <= loc_map._update_max_h)
            {
                loc_map.set_vox_type(local_crd, VOXTYPE_OCCUPIED);
                VB_keys_loc_D[idx_1d] = get_VB_key(glb_crd);
            }else
            {
                VB_keys_loc_D[idx_1d] = EMPTY_KEY;
            }
        }
    }

    void localOGMKernels(LocMap* loc_map, LASER_RANGE_TPYE *detph_data, Projection proj, MulScanParam param,
                         int3* VB_keys_loc_D, bool for_motion_planner, int rbt_r2_grids)
    {

        const int gridSize = loc_map->_local_size.z;
        const int blkSize = loc_map->_local_size.y;
        setLocalOccupancy<<<gridSize,blkSize>>>(*loc_map,detph_data,proj,param,VB_keys_loc_D,
                for_motion_planner,rbt_r2_grids);
    }
}