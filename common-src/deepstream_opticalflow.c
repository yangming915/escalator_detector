#include "deepstream_common.h"
#include "deepstream_opticalflow.h"

// Create bin, add queue and the element, link all elements and ghost pads,
// Set the element properties from the parsed config
gboolean
create_opticalflow_bin (NvDsOpticalFlowConfig * config, NvDsOpticalFlowBin * bin)
{
  //GstCaps *caps = NULL;
  gboolean ret = FALSE;

  bin->bin = gst_bin_new ("nvof_bin");
  if (!bin->bin) {
    NVGSTDS_ERR_MSG_V ("Failed to create 'nvof_bin'");
    goto done;
  }

  bin->queue = gst_element_factory_make (NVDS_ELEM_QUEUE, "nvof_queue");
  if (!bin->queue) {
    NVGSTDS_ERR_MSG_V ("Failed to create 'nvof_queue'");
    goto done;
  }
  gst_bin_add (GST_BIN (bin->bin), bin->queue);

  bin->nvof = gst_element_factory_make("nvof","nvopticalflow");
  if (!bin->nvof)
  {
    NVGSTDS_ERR_MSG_V ("Failed to create 'nvof'");
    goto done;
  }
  gst_bin_add (GST_BIN (bin->bin), bin->nvof);

  NVGSTDS_LINK_ELEMENT (bin->queue, bin->nvof);

  NVGSTDS_BIN_ADD_GHOST_PAD (bin->bin, bin->queue, "sink");
  NVGSTDS_BIN_ADD_GHOST_PAD (bin->bin, bin->nvof, "src");

  g_object_set (G_OBJECT (bin->nvof),
      "preset-level", config->preset_level,
      "grid-size", config->grid_size,
      "pool-size", config->pool_size,
      "dump-of-meta", config->dump_of_meta,
      "gpu-id", config->gpu_id, NULL);
      
  ret = TRUE;

done:
  if (!ret) {
    NVGSTDS_ERR_MSG_V ("%s failed", __func__);
  }

  return ret;
}
