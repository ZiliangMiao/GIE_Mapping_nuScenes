
#ifndef SRC_VLP16_HELPER_H
#define SRC_VLP16_HELPER_H

#include "cuda_toolkit/cuda_macro.h"
#include "cuda_toolkit/projection.h"
#include "cuda_toolkit/occupancy/vlp16/multiscan_param.h"

namespace VLP_HELPER
{
    __device__ __forceinline__
    int positive_modulo(int i, int n)
    {
        return (i % n + n) % n;
    }

    //---
    __device__ __forceinline__
    float getDist2Line(float phi, float theta, float3 x0)
    {
        float3 unit_vector;
        unit_vector.z = sin(phi);
        unit_vector.x = cos(phi)*cos(theta);
        unit_vector.y = cos(phi)*sin(theta);

        float numerator = sqrtf((unit_vector.z*x0.y- unit_vector.y*x0.z)*(unit_vector.z*x0.y- unit_vector.y*x0.z)+
                               (unit_vector.x*x0.z- unit_vector.z*x0.x)*(unit_vector.x*x0.z- unit_vector.z*x0.x)+
                               (unit_vector.y*x0.x- unit_vector.x*x0.y)*(unit_vector.y*x0.x- unit_vector.x*x0.y));
        // denominator =1

        return numerator;
    }

    __device__ __forceinline__
    void G2L(const float3 &pos,const Projection &proj,
             const MulScanParam &param, const float &grid_width, int &theta_idx, int &phi_idx, LASER_RANGE_TPYE &depth)
    {
        // voxels: from global frame to sensor frame
        float3 local_pos = proj.G2L * pos;  // SE3: from global to local, meter

        // calculate theta and theta_idx (azimuth)
        float theta = atan2f(local_pos.y, local_pos.x);
        theta_idx = floorf((theta - param.theta_min)/param.theta_inc + 0.5f);
        theta_idx = positive_modulo(theta_idx, param.scan_num);

        // calculate phi (elevation)
        float r_horizon = sqrtf(local_pos.x * local_pos.x + local_pos.y * local_pos.y);
        float phi = atan2(local_pos.z, r_horizon);
        // phi_idx = floor((phi - param.phi_min)/param.phi_inc +0.5f);
        
        // get phi index from ring_angle // question mark [miaozl]
        if (phi >= param.ring_0 && phi < param.ring_1)
        {
            phi_idx = 0;
        }
        else if (phi >= param.ring_1 && phi < param.ring_2)
        {
            phi_idx = 1;
        }
        else if (phi >= param.ring_2 && phi < param.ring_3)
        {
            phi_idx = 2;
        }
        else if (phi >= param.ring_3 && phi < param.ring_4)
        {
            phi_idx = 3;
        }
        else if (phi >= param.ring_4 && phi < param.ring_5)
        {
            phi_idx = 4;
        }
        else if (phi >= param.ring_5 && phi < param.ring_6)
        {
            phi_idx = 5;
        }
        else if (phi >= param.ring_6 && phi < param.ring_7)
        {
            phi_idx = 6;
        }
        else if (phi >= param.ring_7 && phi < param.ring_8)
        {
            phi_idx = 7;
        }
        else if (phi >= param.ring_8 && phi < param.ring_9)
        {
            phi_idx = 8;
        }
        else if (phi >= param.ring_9 && phi < param.ring_10)
        {
            phi_idx = 9;
        }
        else if (phi >= param.ring_10 && phi < param.ring_11)
        {
            phi_idx = 10;
        }
        else if (phi >= param.ring_11 && phi < param.ring_12)
        {
            phi_idx = 11;
        }
        else if (phi >= param.ring_12 && phi < param.ring_13)
        {
            phi_idx = 12;
        }
        else if (phi >= param.ring_13 && phi < param.ring_14)
        {
            phi_idx = 13;
        }
        else if (phi >= param.ring_14 && phi < param.ring_15)
        {
            phi_idx = 14;
        }
        else if (phi >= param.ring_15 && phi < param.ring_16)
        {
            phi_idx = 15;
        }
        else if (phi >= param.ring_16 && phi < param.ring_17)
        {
            phi_idx = 16;
        }
        else if (phi >= param.ring_17 && phi < param.ring_18)
        {
            phi_idx = 17;
        }
        else if (phi >= param.ring_18 && phi < param.ring_19)
        {
            phi_idx = 18;
        }
        else if (phi >= param.ring_19 && phi < param.ring_20)
        {
            phi_idx = 19;
        }
        else if (phi >= param.ring_20 && phi < param.ring_21)
        {
            phi_idx = 20;
        }
        else if (phi >= param.ring_21 && phi < param.ring_22)
        {
            phi_idx = 21;
        }
        else if (phi >= param.ring_22 && phi < param.ring_23)
        {
            phi_idx = 22;
        }
        else if (phi >= param.ring_23 && phi < param.ring_24)
        {
            phi_idx = 23;
        }
        else if (phi >= param.ring_24 && phi < param.ring_25)
        {
            phi_idx = 24;
        }
        else if (phi >= param.ring_25 && phi < param.ring_26)
        {
            phi_idx = 25;
        }
        else if (phi >= param.ring_26 && phi < param.ring_27)
        {
            phi_idx = 26;
        }
        else if (phi >= param.ring_27 && phi < param.ring_28)
        {
            phi_idx = 27;
        }
        else if (phi >= param.ring_28 && phi < param.ring_29)
        {
            phi_idx = 28;
        }
        else if (phi >= param.ring_29 && phi < param.ring_30)
        {
            phi_idx = 29;
        }
        else if (phi >= param.ring_30 && phi < param.ring_31)
        {
            phi_idx = 30;
        }
        else if (phi >= param.ring_31 && phi <= param.ring_31_max)
        {
            phi_idx = 31;
        }
        else
        {
            phi_idx = -1;  // invalid phi, not observable
        }

        // check phi index
        // if (phi_idx == 31) {
        //     printf("Check elevation index: %f, %d\n", phi/M_PI*180.0f, phi_idx);
        // }
        // else if (phi_idx == 0) {
        //     printf("Check elevation index: %f, %d\n", phi/M_PI*180.0f, phi_idx);
        // }

        // get the depth
        if(phi_idx < 0 || phi_idx >= param.ring_num)
        {
            depth = -1.f;
            return;
        }
        float dist2ray= getDist2Line(phi, theta, local_pos);  // question mark
        if(fabs(dist2ray) >= grid_width)
        {
            depth = -1.f;
            return;
        }

        depth = sqrtf(local_pos.x * local_pos.x + local_pos.y * local_pos.y + local_pos.z * local_pos.z);  // question mark [miaozl]
    }

}

#endif //SRC_VLP16_HELPER_H
