#ifndef _NVGSTDS_OPTICALFLOW_H_
#define _NVGSTDS_OPTICALFLOW_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <gst/gst.h>
#include "nvds_opticalflow_meta.h"
typedef struct
{
  // Create a bin for the element only if enabled
  gboolean enable;
  gint dump_of_meta;
  gint preset_level;
  gint grid_size;
  gint pool_size;
  gint gpu_id;

} NvDsOpticalFlowConfig;

// Struct to store references to the bin and elements
typedef struct
{
  GstElement *bin;
  GstElement *nvof;
  GstElement *queue;
  
} NvDsOpticalFlowBin;

// Function to create the bin and set properties
gboolean
create_opticalflow_bin (NvDsOpticalFlowConfig *config, NvDsOpticalFlowBin *bin);

#ifdef __cplusplus
}
#endif

#endif /* _NVGSTDS_OPTICALFLOW_H_ */