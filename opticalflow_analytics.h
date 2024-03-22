#ifndef __OPTICALLFLOW_ANALYTICS_H__
#define __OPTICALLFLOW_ANALYTICS_H__
#include "nvds_opticalflow_meta.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _Roi
{
    guint top;
    guint left;
    guint width;
    guint height;
}Roi;
typedef struct _FlowVector
{
    gfloat x;
    gfloat y;
}FlowVector;
typedef FlowVector StandardFlowVector;
struct _Param
{
    gint max_speed;
    StandardFlowVector sfv;
    gfloat standard_deviation;
    gfloat standard_mean;
    guint denoisy_size;
    Roi roi;
};

struct _nvof_output
{
    gfloat SVariance;
    gfloat SSkewness;
    gfloat SKurtosis;
    gfloat CVariance;
    gfloat CSkewness;
    gfloat CKurtosis;
};
typedef struct _threshold
{
    gfloat VarianceEscalatorStop;//0.008
    gfloat SkewnessFalldown;//-2.5
    gfloat CSkewnessTotter;//SSkewness 趋于0不触发，但CSkewneww触发，-2.5
}threshold;
typedef struct _Activation
{
    bool EStop;
    bool Falldown;
    bool Totter;
    guint EStopActivationTime;
    guint FActivationTime;
    guint TActivationTime;
}ActionPool;

enum Exception
{
    ESCALATOR_STOP=-1,NORM,FALLDOWN,TOTTER
};

typedef struct _nvof_output OpticalFlowOutput;
typedef struct _Param Param;

enum Exception DsDirectionProcess (NvDsOpticalFlowMeta * flow_meta, Param *param);

void *Normalize(FlowVector *flow, gfloat max_speed, guint element_num);

void *Denoisy(FlowVector *flow, guint denoisy_window_size, const guint height, const guint width);

void *CosineSimilarFlow(FlowVector *flow, gfloat *dst_cs_flow_mat, StandardFlowVector sfv, guint element_num);

gfloat Variance(gfloat *cs_flow_mat, gfloat mean, guint element_num);

gfloat Skewness(gfloat *cs_flow_mat, gfloat mean, gfloat sd, guint element_num);

gfloat Kurtosis(gfloat *cs_flow_mat, gfloat mean, gfloat sd, guint element_num);

float StandardDeviation(gfloat *cs_flow_mat, gfloat mean, guint element_num);

float Mean(gfloat *cs_flow_mat, guint element_num);

#ifdef __cplusplus
}
#endif

#endif