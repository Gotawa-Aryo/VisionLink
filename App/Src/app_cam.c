 /**
 ******************************************************************************
 * @file    app_cam.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <assert.h>
#include "app.h"
#include "cmw_camera.h"
#include "app_cam.h"
#include "app_config.h"
#include "utils.h"

/* Camera bring-up markers, printed only in a `make UTK_TRACE=1` build. */
#if UTK_TRACE
void kdbg(const char *);
#define CAM_TRACE(s) kdbg(s)
#else
#define CAM_TRACE(s) do { } while (0)
#endif

/* Keep display output aspect ratio using crop area */
static void CAM_InitCropConfig(CMW_Manual_roi_area_t *roi, int sensor_width, int sensor_height)
{
  const float ratiox = (float)sensor_width / LCD_BG_WIDTH;
  const float ratioy = (float)sensor_height / LCD_BG_HEIGHT;
  const float ratio = MIN(ratiox, ratioy);

  assert(ratio >= 1);
  assert(ratio < 64);

  roi->width = (uint32_t) MIN(LCD_BG_WIDTH * ratio, sensor_width);
  roi->height = (uint32_t) MIN(LCD_BG_HEIGHT * ratio, sensor_height);
  roi->offset_x = (sensor_width - roi->width + 1) / 2;
  roi->offset_y = (sensor_height - roi->height + 1) / 2;
}

/* Pipe 1: camera to display, cropped to the screen's aspect ratio. */
static void DCMIPP_PipeInitDisplay(int sensor_width, int sensor_height)
{
  CMW_DCMIPP_Conf_t dcmipp_conf;
  uint32_t hw_pitch;
  int ret;

  assert(LCD_BG_WIDTH >= LCD_BG_HEIGHT);

  dcmipp_conf.output_width = LCD_BG_WIDTH;
  dcmipp_conf.output_height = LCD_BG_HEIGHT;
  dcmipp_conf.output_format = DISPLAY_FORMAT;
  dcmipp_conf.output_bpp = DISPLAY_BPP;
  dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
  dcmipp_conf.enable_swap = 1;
  dcmipp_conf.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height);
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dcmipp_conf, &hw_pitch);
  assert(ret == HAL_OK);
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);
}

/* Pipe 2: camera to the palm detector's input size. */
static void DCMIPP_PipeInitNn(int sensor_width, int sensor_height)
{
  CMW_DCMIPP_Conf_t dcmipp_conf;
  uint32_t hw_pitch;
  int ret;

  assert(LCD_BG_HEIGHT <= LCD_BG_WIDTH);
  /* Keep screen aspect ratio. Consequence is that bottom of palm detector input will be black */
  dcmipp_conf.output_width = NN_WIDTH;
  dcmipp_conf.output_height = (int) (NN_HEIGHT * ((float)LCD_BG_HEIGHT / LCD_BG_WIDTH));
  dcmipp_conf.output_format = NN_FORMAT;
  dcmipp_conf.output_bpp = NN_BPP;
  dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
  dcmipp_conf.enable_swap = 1;
  dcmipp_conf.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height);
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE2, &dcmipp_conf, &hw_pitch);
  assert(ret == HAL_OK);
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);
}

/* Share the DCMIPP's memory bandwidth between the two pipes. */
static void DCMIPP_IpPlugInit(DCMIPP_HandleTypeDef *hdcmipp)
{
  DCMIPP_IPPlugConfTypeDef ipplug_conf = { 0 };
  int ret;

  ipplug_conf.MemoryPageSize = DCMIPP_MEMORY_PAGE_SIZE_256BYTES;

  ipplug_conf.Client = DCMIPP_CLIENT2; /* aux pipe */
  ipplug_conf.Traffic = DCMIPP_TRAFFIC_BURST_SIZE_128BYTES;
  ipplug_conf.MaxOutstandingTransactions = DCMIPP_OUTSTANDING_TRANSACTION_NONE;
  ipplug_conf.DPREGStart = 0;
  ipplug_conf.DPREGEnd = 559; /* (4480 bytes / one line) */
  ipplug_conf.WLRURatio = 15; /* 16 parts of BW */
  ret = HAL_DCMIPP_SetIPPlugConfig(hdcmipp, &ipplug_conf);
  assert(ret == HAL_OK);

  ipplug_conf.Client = DCMIPP_CLIENT5; /* main rgb pipe */
  ipplug_conf.Traffic = DCMIPP_TRAFFIC_BURST_SIZE_128BYTES;
  ipplug_conf.MaxOutstandingTransactions = DCMIPP_OUTSTANDING_TRANSACTION_3;
  ipplug_conf.DPREGStart = 560;
  ipplug_conf.DPREGEnd = 639;
  ipplug_conf.WLRURatio = 0; /* 1 parts of BW */
  ret = HAL_DCMIPP_SetIPPlugConfig(hdcmipp, &ipplug_conf);
  assert(ret == HAL_OK);
}

/* Enable, then disable, pipe 1's line event, as ST's example does, to limit spurious
 * interrupts. */
static void DCMIPP_ReduceSpurious(DCMIPP_HandleTypeDef *hdcmipp)
{
  int ret;

  ret = HAL_DCMIPP_PIPE_EnableLineEvent(hdcmipp, DCMIPP_PIPE1, DCMIPP_MULTILINE_128_LINES);
  assert(ret == HAL_OK);
  ret = HAL_DCMIPP_PIPE_DisableLineEvent(hdcmipp, DCMIPP_PIPE1);
  assert(ret == HAL_OK);
}

/* Find the sensor and configure both camera pipes. */
void CAM_Init(void)
{
  CMW_Sensor_Config_t sensor_config;
  CMW_CameraInit_t cam_conf;
  CMW_Sensor_Name_t sensor;
  int ret;

  CAM_TRACE("D: pre GetSensorName");
  ret = CMW_CAMERA_GetSensorName(&sensor);
  assert(ret == CMW_ERROR_NONE);

  /* With vd66gy sensor we have peak bandwidth higher than the external memory can support. So to reduce pressure we :
   *  - Fine tune IP Plug so we can buffer at lease one line
   *  - Increase line_len so IP Plug has enought time to empty internal FIFO before next line is coming.
   */
  CAM_TRACE("D: post GetSensorName");
  sensor_config.selected_sensor = CMW_VD66GY_Sensor;
  ret = CMW_CAMERA_SetDefaultSensorValues(&sensor_config);
  assert(ret == CMW_ERROR_NONE);
  sensor_config.config.vd66gy_config.line_len = 2000;

  /* Let sensor driver choose which width/height to use */
  cam_conf.width = 0;
  cam_conf.height = 0;
  cam_conf.fps = CAMERA_FPS;
  cam_conf.pixel_format = 0; /* Default; Not implemented yet */
  cam_conf.anti_flicker = 0;
  cam_conf.mirror_flip = CAMERA_FLIP;
  CAM_TRACE("D: pre CMW_CAMERA_Init");
  ret = CMW_CAMERA_Init(&cam_conf, sensor == CMW_VD66GY_Sensor ? &sensor_config : NULL);
  assert(ret == CMW_ERROR_NONE);

  /* cam_conf.width / cam_conf.height now contains choose resolution */
  CAM_TRACE("D: post CMW_CAMERA_Init");
  DCMIPP_IpPlugInit(CMW_CAMERA_GetDCMIPPHandle());
  CAM_TRACE("D: post IpPlugInit");
  DCMIPP_PipeInitDisplay(cam_conf.width, cam_conf.height);
  CAM_TRACE("D: post PipeInitDisplay");
  DCMIPP_PipeInitNn(cam_conf.width, cam_conf.height);
  CAM_TRACE("D: post PipeInitNn");
  DCMIPP_ReduceSpurious(CMW_CAMERA_GetDCMIPPHandle());
}

/* Start pipe 1 streaming into the display buffer. */
void CAM_DisplayPipe_Start(uint8_t *display_pipe_dst, uint32_t cam_mode)
{
  int ret;

  ret = CMW_CAMERA_Start(DCMIPP_PIPE1, display_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
}

/* Start pipe 2 streaming into the NN input buffer. */
void CAM_NNPipe_Start(uint8_t *nn_pipe_dst, uint32_t cam_mode)
{
  int ret;

  ret = CMW_CAMERA_Start(DCMIPP_PIPE2, nn_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
}

/* One ISP step (exposure, white balance); runs on each vsync. */
void CAM_IspUpdate(void)
{
  int ret;

  ret = CMW_CAMERA_Run();
  assert(ret == CMW_ERROR_NONE);
}
