/*
 * Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Edited by Marcos Luciano
 * https://www.github.com/marcoslucianops
 */

#include <algorithm>
#include "nvdsinfer_custom_impl.h"
#include "utils.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define NMS_THRESH 0.45;

extern "C" bool
NvDsInferParseRtmPose(std::vector<NvDsInferLayerInfo> const& outputLayersInfo, NvDsInferNetworkInfo const& networkInfo,
    NvDsInferParseDetectionParams const& detectionParams, std::vector<NvDsInferInstanceMaskInfo>& objectList);

static std::vector<NvDsInferInstanceMaskInfo>
nonMaximumSuppression(std::vector<NvDsInferInstanceMaskInfo> binfo)
{
  auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
    if (x1min > x2min) {
      std::swap(x1min, x2min);
      std::swap(x1max, x2max);
    }
    return x1max < x2min ? 0 : std::min(x1max, x2max) - x2min;
  };

  auto computeIoU = [&overlap1D](NvDsInferInstanceMaskInfo& bbox1, NvDsInferInstanceMaskInfo& bbox2) -> float {
    float overlapX = overlap1D(bbox1.left, bbox1.left + bbox1.width, bbox2.left, bbox2.left + bbox2.width);
    float overlapY = overlap1D(bbox1.top, bbox1.top + bbox1.height, bbox2.top, bbox2.top + bbox2.height);
    float area1 = (bbox1.width) * (bbox1.height);
    float area2 = (bbox2.width) * (bbox2.height);
    float overlap2D = overlapX * overlapY;
    float u = area1 + area2 - overlap2D;
    return u == 0 ? 0 : overlap2D / u;
  };

  std::stable_sort(binfo.begin(), binfo.end(), [](const NvDsInferInstanceMaskInfo& b1, const NvDsInferInstanceMaskInfo& b2) {
    return b1.detectionConfidence > b2.detectionConfidence;
  });

  std::vector<NvDsInferInstanceMaskInfo> out;
  for (auto i : binfo) {
    bool keep = true;
    for (auto j : out) {
      if (keep) {
        float overlap = computeIoU(i, j);
        keep = overlap <= NMS_THRESH;
      }
      else {
        break;
      }
    }
    if (keep) {
      out.push_back(i);
    }
  }
  return out;
}
static std::vector<NvDsInferInstanceMaskInfo>
nmsAllClasses(std::vector<NvDsInferInstanceMaskInfo>& binfo)
{
  std::vector<NvDsInferInstanceMaskInfo> result = nonMaximumSuppression(binfo);
  return result;
}


static std::vector<NvDsInferInstanceMaskInfo>
decodeTensorRtmPose(const float* simcc_x, const float* simcc_y, 
                    const uint& kptNum, const uint& width,const uint& height,
                    const uint& netW, const uint& netH, const std::vector<float>& preclusterThreshold)
{
  std::vector<NvDsInferInstanceMaskInfo> binfo;
  NvDsInferInstanceMaskInfo bbi;
  bbi.mask=new float[kptNum *3];
  for (uint k = 0; k < kptNum; ++k) {
    //std::printf("%d\n",k);
    uint xkp=(std::max_element(&simcc_x[k*width],&simcc_x[k*width+width])-&simcc_x[k*width]);
    uint ykp=(std::max_element(&simcc_y[k*height],&simcc_y[k*height+height])-&simcc_y[k*height]);
    
    float xks=simcc_x[k*width+xkp];
    float yks=simcc_y[k*height+ykp];
    float score=MIN(xks,yks);

    if(score<=0.)xkp=ykp=-1;

    xkp/=2.0;ykp/=2.0;
    bbi.mask[k*3+0]=clamp(xkp,0,netW);
    bbi.mask[k*3+1]=clamp(ykp,0,netH);
    

    bbi.mask[k*3+2]=score;
    //std::printf("(%d,%d):%f\n",xkp,ykp,score);
    
  }
  bbi.mask_width = netW;
  bbi.mask_height = netH;
  bbi.mask_size = sizeof(float) * kptNum*3;
  bbi.classId=0;
  bbi.detectionConfidence=1;
  bbi.left=bbi.top=0;
  bbi.height=256;
  bbi.width=192;
  
  binfo.push_back(bbi);
  //std::printf("%d\n",binfo[0].mask_size);
  return binfo;
}

static bool
NvDsInferParseCustomRtmPose(std::vector<NvDsInferLayerInfo> const& outputLayersInfo,
    NvDsInferNetworkInfo const& networkInfo, NvDsInferParseDetectionParams const& detectionParams,
    std::vector<NvDsInferInstanceMaskInfo>& objectList)
{
  if (outputLayersInfo.empty()) {
    std::cerr << "ERROR: Could not find output layer in bbox parsing" << std::endl;
    return false;
  }

  const NvDsInferLayerInfo& simcc_x = outputLayersInfo[0];
  const NvDsInferLayerInfo& simcc_y = outputLayersInfo[1];
  const uint kptNum = simcc_x.inferDims.d[0];
  const uint width = simcc_x.inferDims.d[1];
  const uint height = simcc_y.inferDims.d[1];
  //std::cout<<width<<height<<std::endl;
  std::vector<NvDsInferInstanceMaskInfo> objects = decodeTensorRtmPose((const float*) (simcc_x.buffer),
                                                    (const float*) (simcc_y.buffer), kptNum,
                                                    width, height, networkInfo.width, networkInfo.height, 
                                                    detectionParams.perClassPreclusterThreshold);

  objectList.clear();
  objectList = nmsAllClasses(objects);

  return true;
}


extern "C" bool
NvDsInferParseRtmPose(std::vector<NvDsInferLayerInfo> const& outputLayersInfo, NvDsInferNetworkInfo const& networkInfo,
    NvDsInferParseDetectionParams const& detectionParams, std::vector<NvDsInferInstanceMaskInfo>& objectList)
{
  return NvDsInferParseCustomRtmPose(outputLayersInfo, networkInfo, detectionParams, objectList);
}


CHECK_CUSTOM_INSTANCE_MASK_PARSE_FUNC_PROTOTYPE(NvDsInferParseRtmPose);

