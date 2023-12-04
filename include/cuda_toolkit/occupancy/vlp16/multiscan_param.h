

#ifndef SRC_MULTISCAN_PARAM_H
#define SRC_MULTISCAN_PARAM_H

struct MulScanParam
{
    float max_r;
    float theta_inc, theta_min;
    // float phi_inc, phi_min;
    int scan_num;
    int ring_num;

    // start elevation angle of each ring
    float ring_0,  ring_1,  ring_2,  ring_3,  ring_4;
    float ring_5,  ring_6,  ring_7,  ring_8,  ring_9;
    float ring_10, ring_11, ring_12, ring_13, ring_14;
    float ring_15, ring_16, ring_17, ring_18, ring_19;
    float ring_20, ring_21, ring_22, ring_23, ring_24;
    float ring_25, ring_26, ring_27, ring_28, ring_29;
    float ring_30, ring_31, ring_31_max;  // ring_31 = min(ring 31); ring_32 = max(ring 31) 

    MulScanParam(int scan_num_, int ring_num_,
                 float max_r_, float theta_inc_,
                 float theta_min_,
                 float ring_0_,  float ring_1_,  float ring_2_,  float ring_3_,
                 float ring_4_,  float ring_5_,  float ring_6_,  float ring_7_,
                 float ring_8_,  float ring_9_,  float ring_10_, float ring_11_,
                 float ring_12_, float ring_13_, float ring_14_, float ring_15_,
                 float ring_16_, float ring_17_, float ring_18_, float ring_19_,
                 float ring_20_, float ring_21_, float ring_22_, float ring_23_,
                 float ring_24_, float ring_25_, float ring_26_, float ring_27_,
                 float ring_28_, float ring_29_, float ring_30_, float ring_31_, float ring_31_max_):
            scan_num(scan_num_),
            ring_num(ring_num_),
            max_r(max_r_),
            theta_inc(theta_inc_),
            theta_min(theta_min_),
            ring_0(ring_0_), ring_1(ring_1_), ring_2(ring_2_), ring_3(ring_3_),
            ring_4(ring_4_), ring_5(ring_5_), ring_6(ring_6_), ring_7(ring_7_),
            ring_8(ring_8_), ring_9(ring_9_), ring_10(ring_10_), ring_11(ring_11_),
            ring_12(ring_12_), ring_13(ring_13_), ring_14(ring_14_), ring_15(ring_15_),
            ring_16(ring_16_), ring_17(ring_17_), ring_18(ring_18_), ring_19(ring_19_),
            ring_20(ring_20_), ring_21(ring_21_), ring_22(ring_22_), ring_23(ring_23_),
            ring_24(ring_24_), ring_25(ring_25_), ring_26(ring_26_), ring_27(ring_27_),
            ring_28(ring_28_), ring_29(ring_29_), ring_30(ring_30_), ring_31(ring_31_), ring_31_max(ring_31_max_)
    {}
    MulScanParam()
    {}
};

#endif //SRC_MULTISCAN_PARAM_H
