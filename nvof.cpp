#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <bits/stdc++.h>
#include <fstream>
#include <iostream>
#include "nvof.h"

#define PI 3.141592
#define PI_IN_DEGREES 180
#define ACTIVATION_TIME 20
ActionPool Condition = {false, false, false, ACTIVATION_TIME, ACTIVATION_TIME, ACTIVATION_TIME};
threshold TS = {0.008, -3.5, -3};
enum Exception E = NORM;
enum Exception DsDirectionProcess(NvDsOpticalFlowMeta *flow_meta,
                                  Param *param)
{
  OpticalFlowOutput *out =
      (OpticalFlowOutput *)calloc(1, sizeof(OpticalFlowOutput));
  NvOFFlowVector *metadata = (NvOFFlowVector *)flow_meta->data;
  guint rows = flow_meta->rows;
  guint cols = flow_meta->cols;
  Roi rect = param->roi;
  guint element_num = rect.height * rect.width;
  NvOFFlowVector *flow = (NvOFFlowVector *)calloc(1, element_num * sizeof(NvOFFlowVector));
  FlowVector *float_flow = (FlowVector *)calloc(1, element_num * sizeof(FlowVector));
  gfloat *cs_flow = (gfloat *)calloc(1, sizeof(gfloat) * element_num);

  NvOFFlowVector *dest = NULL;
  NvOFFlowVector *src = NULL;
  for (uint i = 0; i < rect.height; i++)
  {
    dest = flow + i * rect.width;
    src = metadata + (rect.top + i) * cols + rect.left;
    memcpy(dest, src, sizeof(NvOFFlowVector) * rect.width);
  }
  FlowVector *_TempFloat32 = float_flow;
  NvOFFlowVector *_TempInt16 = flow;
  for (guint i = 0; i < element_num; i++)
  {
    _TempFloat32->x = _TempInt16->flowx >> 5; // 除以32
    _TempFloat32->y = _TempInt16->flowy >> 5;
    _TempFloat32++;
    _TempInt16++;
  }
  float_flow = (FlowVector *)Denoisy(float_flow, param->denoisy_size, rect.height, rect.width);
  float_flow = (FlowVector *)Normalize(float_flow, param->max_speed, element_num);
  cs_flow = (gfloat *)CosineSimilarFlow(float_flow, cs_flow, param->sfv, element_num);

  gfloat mean = Mean(cs_flow, element_num);
  gfloat sd = StandardDeviation(cs_flow, mean, element_num);
  out->SVariance = Variance(cs_flow, param->standard_mean, element_num);
  out->SSkewness = Skewness(cs_flow, param->standard_mean, param->standard_deviation, element_num);
  out->SKurtosis = Kurtosis(cs_flow, param->standard_mean, param->standard_deviation, element_num);
  out->CVariance = Variance(cs_flow, mean, element_num);
  out->CSkewness = Skewness(cs_flow, mean, sd, element_num);
  out->CKurtosis = Kurtosis(cs_flow, mean, sd, element_num);

  if (out->SVariance < TS.VarianceEscalatorStop)
  {
    Condition.EStop = true;
    Condition.EStopActivationTime = ACTIVATION_TIME;
  }
  else if (out->SVariance >= TS.VarianceEscalatorStop && Condition.EStopActivationTime > 0)
  {
    Condition.EStopActivationTime--;
  }
  else if (out->SVariance >= TS.VarianceEscalatorStop && Condition.EStopActivationTime == 0)
  {
    Condition.EStop = false;
    Condition.EStopActivationTime = ACTIVATION_TIME;
  }

  if (!Condition.EStop && out->SSkewness <= TS.SkewnessFalldown)
  {
    Condition.Falldown = true;
    Condition.FActivationTime = ACTIVATION_TIME;
  }
  else if (!Condition.EStop && out->SSkewness > TS.SkewnessFalldown && Condition.FActivationTime > 0)
  {
    Condition.FActivationTime--;
  }
  else if (!Condition.EStop && out->SSkewness > TS.SkewnessFalldown && Condition.FActivationTime == 0)
  {
    Condition.Falldown = false;
    Condition.FActivationTime = ACTIVATION_TIME;
  }
  else
  {
    Condition.Falldown = false;
    Condition.FActivationTime = ACTIVATION_TIME;
  }

  if (!Condition.EStop && !Condition.Falldown 
      && out->CSkewness <= TS.CSkewnessTotter && out->SSkewness > TS.CSkewnessTotter && out->SSkewness < 0)
  {
    Condition.Totter = true;
    Condition.TActivationTime = ACTIVATION_TIME;
  }
  else if (!Condition.EStop && out->CSkewness > TS.CSkewnessTotter && Condition.TActivationTime > 0)
  {
    Condition.TActivationTime--;
  }
  else if (!Condition.EStop && out->CSkewness > TS.CSkewnessTotter && Condition.TActivationTime == 0)
  {
    Condition.Totter = false;
    Condition.TActivationTime = ACTIVATION_TIME;
  }
  else
  {
    Condition.Totter = false;
    Condition.TActivationTime = ACTIVATION_TIME;
  }

  if (Condition.EStop)
    E = ESCALATOR_STOP;
  else if (!Condition.EStop && Condition.Falldown)
    E = FALLDOWN;
  else if (!Condition.EStop && !Condition.Falldown && Condition.Totter)
    E = TOTTER;
  else
    E = NORM;
  if (E)
  {
    std::fstream Log;
    Log.open("log.txt", std::fstream::app);
    Log << E << " " << flow_meta->frame_num << " " << std::fixed << flow_meta->frame_num / 25.0
        << "  SV:" << out->SVariance << "  SS:" << out->SSkewness << "  SK:" << out->SKurtosis
        << "  CV:" << out->CVariance << "  CS:" << out->CSkewness << "  CK:" << out->CKurtosis << std::endl;
    Log.close();
  }

  return E;
}
void *Normalize(FlowVector *flow, gfloat max_speed, guint element_num)
{
  FlowVector *_Temp = flow;
  for (guint i = 0; i < element_num; i++, _Temp++)
  {
    if (_Temp->x > max_speed)
      _Temp->x = 1;
    else if (_Temp->x < -max_speed)
      _Temp->x = -1;
    else
      _Temp->x /= max_speed;
    if (_Temp->y > max_speed)
      _Temp->y = 1;
    else if (_Temp->y < -max_speed)
      _Temp->y = -1;
    else
      _Temp->y /= max_speed;
  }
  return flow;
}
// 窗口大小为单数，小于7
void *Denoisy(FlowVector *flow, guint denoisy_window_size, const guint height, const guint width)
{
  const guint window_size = pow(denoisy_window_size, 2);
  guint step = (denoisy_window_size - 1) >> 1;
  gfloat sort_buff_x[window_size] = {0};
  gfloat sort_buff_y[window_size] = {0};
  gint position[window_size] = {0};
  for (guint i = 0; i < window_size; i++)
  {
    position[i] = (i / denoisy_window_size - step) * width + (i % denoisy_window_size - step);
  }
  for (guint h = step; h + step < height; h++)
  {
    for (guint w = step; w + step < width; w++)
    {
      for (guint i = 0; i < window_size; i++)
      {
        sort_buff_x[i] = flow[h * width + w + position[i]].x;
        sort_buff_y[i] = flow[h * width + w + position[i]].y;
      }
      std::sort(sort_buff_x, sort_buff_x + window_size);
      std::sort(sort_buff_y, sort_buff_y + window_size);

      flow[h * width + w].x = sort_buff_x[(window_size - 1) >> 1];
      flow[h * width + w].y = sort_buff_y[(window_size - 1) >> 1];
    }
  }
  return flow;
}
void *CosineSimilarFlow(FlowVector *flow, gfloat *dst_cs_flow_mat, StandardFlowVector sfv, guint element_num)
{
  gfloat z2 = pow(sfv.x, 2) + pow(sfv.y, 2);
  z2 = sqrt(z2);
  FlowVector *flow_element = flow;
  gfloat *dst_flow_element = dst_cs_flow_mat;
  for (guint i = 0; i < element_num; i++, flow_element++, dst_flow_element++)
  {
    gfloat element_x = flow_element->x;
    gfloat element_y = flow_element->y;
    gfloat z1z2 = element_x * sfv.x + element_y * sfv.y;
    gfloat similarity = z1z2 / z2;
    *dst_flow_element = similarity;
  }
  return dst_cs_flow_mat;
}

gfloat Variance(gfloat *cs_flow_mat, gfloat mean, guint element_num)
{
  gfloat *mat_element = cs_flow_mat;
  gfloat sum = 0;
  gfloat _temp = 0;
  for (guint i = 0; i < element_num; i++, mat_element++)
  {
    _temp = *mat_element - mean;
    sum += pow(_temp, 2);
  }
  return sum / element_num;
}

gfloat Skewness(gfloat *cs_flow_mat, gfloat mean, gfloat sd, guint element_num)
{
  gfloat *mat_element = cs_flow_mat;
  gfloat sum = 0;
  gfloat _temp = 0;
  for (guint i = 0; i < element_num; i++, mat_element++)
  {
    _temp = *mat_element - mean;
    _temp /= sd;
    sum += pow(_temp, 3);
  }
  return sum / element_num;
}

gfloat Kurtosis(gfloat *cs_flow_mat, gfloat mean, gfloat sd, guint element_num)
{
  gfloat *mat_element = cs_flow_mat;
  gfloat sum = 0;
  gfloat _temp = 0;
  for (guint i = 0; i < element_num; i++, mat_element++)
  {
    _temp = *mat_element - mean;
    _temp /= sd;
    sum += pow(_temp, 4);
  }
  return sum / element_num;
}
float StandardDeviation(gfloat *cs_flow_mat, gfloat mean, guint element_num)
{
  gfloat sum_deviation = 0;
  for (guint i = 0; i < element_num; i++)
  {
    sum_deviation += pow(cs_flow_mat[i] - mean, 2);
  }
  return sqrt(sum_deviation / element_num);
}

float Mean(gfloat *cs_flow_mat, guint element_num)
{
  gfloat mean = 0;
  gfloat sum = 0;
  for (guint i = 0; i < element_num; i++)
  {
    sum += cs_flow_mat[i];
  }
  mean = sum / element_num;
  return mean;
}
