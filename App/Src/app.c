/**
  ******************************************************************************
  * @file    app.c
  * @author  MDG Application Team
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "app.h"

#include <stdint.h>
#include <stdio.h>

#include "app_cam.h"
#include "app_config.h"
#include "IPL_resize.h"
#include "app_postprocess.h"
#include "isp_api.h"
#include "gesture.h"

#include "app_mode.h"
#if APP_HAS_PRESENTER
#include "gesture_hagrid.h"
#include "hand_pose.h"
#include "presenter.h"
#endif
#include "ui.h"
#include "hmi.h"
#if USE_USB_HID_GAMEPAD
#include "usb_hid.h"
#endif
#include "ld.h"
#include "ll_aton_runtime.h"
#include "cmw_camera.h"
#include "scrl.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_ts.h"
#else
#include "stm32n6xx_nucleo.h"
#endif
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"
#include "stm32n6xx_hal.h"
#include "os_utk.h"

/* utk_bsp.c: CPU-load counters (micro T-Kernel has no run-time stats API) */
int _write(int file, char *ptr, int len);   /* Platform/Gcc/console.c, raw UART */
uint32_t utk_get_total_runtime(void);
uint32_t utk_get_idle_runtime(void);
#include "utils.h"

#define APP_PRIORITY(p) OS_PRIORITY(p)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if HAS_ROTATION_SUPPORT == 1
#include "nema_core.h"
#include "nema_error.h"
void nema_enable_tiling(int);
#endif

#define LCD_FG_WIDTH LCD_BG_WIDTH
#define LCD_FG_HEIGHT LCD_BG_HEIGHT

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

/* Diagnostic build levels, set with make APP_DIAG=n. Each one removes a
 * stage from underneath the display, so a dead screen can be attributed:
 *
 *   0  everything: camera -> NPU -> gestures -> control scheme -> GUI
 *   1  no inference. The Neural ART accelerator is never started and no
 *      model runs; camera, LCD, GUI and touchscreen behave normally.
 *   2  no inference and no camera. Only the LCD, the GUI and the touch
 *      controller are left, on a black background.
 *
 * Levels 1 and 2 also draw far less current, so the board can run from a PC
 * port instead of needing a charger. */
#ifndef APP_DIAG
#define APP_DIAG 0
#endif

#define APP_HAS_NN  (APP_DIAG < 1)
#define APP_HAS_CAM (APP_DIAG < 2)


#define DBG_INFO 0
#define USE_FILTERED_TS 1

#define BQUEUE_MAX_BUFFERS 2
#define CPU_LOAD_HISTORY_DEPTH 8

#define DISPLAY_BUFFER_NB (DISPLAY_DELAY + 2)

/* palm detector */
#define PD_MAX_HAND_NB 2          /* hands tracked simultaneously (one per stick) */
#define PD_MAX_CANDIDATES 4       /* palm boxes kept per detector run, best first */
/* While a tracking slot is free but the other is busy, the palm detector runs
 * every Nth frame instead of every frame. Detection costs as much as a
 * landmark pass, so this keeps single-hand use at near full frame rate at the
 * price of a short delay before the second hand is picked up. */
#ifndef PD_REDETECT_PERIOD
#define PD_REDETECT_PERIOD 4
#endif

#if HAS_ROTATION_SUPPORT == 1
typedef float app_v3_t[3];
#endif

typedef struct {
  float cx;
  float cy;
  float w;
  float h;
  float rotation;
} roi_t;

#define UTIL_LCD_COLOR_TRANSPARENT 0

#ifdef STM32N6570_DK_REV
#define LCD_FONT Font20
#define DISK_RADIUS 2
#else
#define LCD_FONT Font12
#define DISK_RADIUS 1
#endif

typedef struct
{
  uint32_t X0;
  uint32_t Y0;
  uint32_t XSize;
  uint32_t YSize;
} Rectangle_TypeDef;

typedef struct {
  os_sem_t free;
  os_sem_t ready;
  int buffer_nb;
  uint8_t *buffers[BQUEUE_MAX_BUFFERS];
  int free_idx;
  int ready_idx;
} bqueue_t;

typedef struct {
  uint64_t current_total;
  uint64_t current_thread_total;
  uint64_t prev_total;
  uint64_t prev_thread_total;
  struct {
    uint64_t total;
    uint64_t thread;
    uint32_t tick;
  } history[CPU_LOAD_HISTORY_DEPTH];
} cpuload_info_t;

typedef struct {
  int is_valid;
  pd_pp_box_t pd_hands;
  /* Backing store for pd_hands.pKps. The single-hand original left that
   * pointer NULL and copy_pd_box() silently wrote the seven key points to
   * address 0. Whoever copies a hand_info_t must re-point pKps at its own
   * pd_kps (see dp_thread_fct). */
  pd_pp_point_t pd_kps[AI_PD_MODEL_PP_NB_KEYPOINTS];
  roi_t roi;
  ld_point_t ld_landmarks[LD_LANDMARK_NB];
  int gesture_cls;      /* filtered class index, or GESTURE_NONE */
  float gesture_prob;   /* raw softmax probability of the last frame */
  int stick;            /* HMI_LEFT / HMI_RIGHT this hand drives, or -1 */
#if APP_HAS_PRESENTER
  int pose;             /* presenter mode: filtered hand_pose_t */
#endif
} hand_info_t;

typedef struct {
  float nn_period_ms;
  uint32_t pd_ms;
  uint32_t hl_ms;
  uint32_t pp_ms;
  uint32_t gesture_us;
  uint32_t disp_ms;
  int is_ld_displayed;
  int is_pd_displayed;
  int pd_hand_nb;
  float pd_max_prob;
  hand_info_t hands[PD_MAX_HAND_NB];
  hmi_state_t hmi;
#if APP_HAS_PRESENTER
  presenter_t pres;
#endif
  ui_view_t ui;
} display_info_t;

typedef struct {
  os_sem_t update;
  os_mutex_t lock;
  display_info_t info;
} display_t;

typedef struct {
  uint32_t nn_in_len;
  float *prob_out;
  uint32_t prob_out_len;
  float *boxes_out;
  uint32_t boxes_out_len;
  pd_model_pp_static_param_t static_param;
  pd_pp_out_t pd_out;
} pd_model_info_t;

typedef struct {
  uint8_t *nn_in;
  uint32_t nn_in_len;
  float *prob_out;
  uint32_t prob_out_len;
  float *landmarks_out;
  uint32_t landmarks_out_len;
} hl_model_info_t;

typedef struct {
  Button_TypeDef button_id;
  int prev_state;
  void (*on_click_handler)(void *cb_args);
  void *cb_args;
} button_t;

/* Globals */
/* Lcd Background area */
static Rectangle_TypeDef lcd_bg_area = {
  .X0 = 0,
  .Y0 = 0,
  .XSize = LCD_BG_WIDTH,
  .YSize = LCD_BG_HEIGHT,
};
/* Lcd Foreground area */
static Rectangle_TypeDef lcd_fg_area = {
  .X0 = 0,
  .Y0 = 0,
  .XSize = LCD_FG_WIDTH,
  .YSize = LCD_FG_HEIGHT,
};
/* Lcd Background Buffer */
static uint8_t lcd_bg_buffer[DISPLAY_BUFFER_NB][LCD_BG_WIDTH * LCD_BG_HEIGHT * DISPLAY_BPP] ALIGN_32 IN_PSRAM;
static int lcd_bg_buffer_disp_idx = 1;
static int lcd_bg_buffer_capt_idx = 0;
/* Lcd Foreground Buffer */
static uint8_t lcd_fg_buffer[2][LCD_FG_WIDTH * LCD_FG_HEIGHT* 2] ALIGN_32 IN_PSRAM;
static int lcd_fg_buffer_rd_idx;
static display_t disp = {
  .info.is_ld_displayed = 1,
  .info.is_pd_displayed = 0,
};
static cpuload_info_t cpu_load;
/* screen buffer */
static uint8_t screen_buffer[LCD_BG_WIDTH * LCD_BG_HEIGHT * 2] ALIGN_32 IN_PSRAM;

/* model */
 /* palm detector */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(palm_detector);
static roi_t rois[PD_MAX_CANDIDATES];
 /* hand landmark */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(hand_landmark);
static uint32_t frame_event_nb;
static volatile uint32_t frame_event_nb_for_resize;

 /* nn input buffers */
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;

 /* rtos */
static os_task_t nn_thread;
static uint8_t nn_thread_stack[2 * OS_MINIMAL_STACK_SIZE] __attribute__((aligned(8)));
static os_task_t dp_thread;
static uint8_t dp_thread_stack[2 * OS_MINIMAL_STACK_SIZE] __attribute__((aligned(8)));
static os_task_t isp_thread;
static uint8_t isp_thread_stack[2 * OS_MINIMAL_STACK_SIZE] __attribute__((aligned(8)));
static os_sem_t isp_sem;

#if HAS_ROTATION_SUPPORT == 1
static GFXMMU_HandleTypeDef hgfxmmu;
static nema_cmdlist_t cl;
#endif

static int is_cache_enable()
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

/* Wrap an angle into [-pi, pi). */
static float pd_normalize_angle(float angle)
{
  return angle - 2 * M_PI * floorf((angle - (-M_PI)) / (2 * M_PI));
}

/* Without rotation support allow limited amount of angles */
#if HAS_ROTATION_SUPPORT == 0
static float pd_cook_rotation(float angle)
{
  if (angle >= (3 * M_PI) / 4)
    angle = M_PI;
  else if (angle >= (1 * M_PI) / 4)
    angle = M_PI / 2;
  else if (angle >= -(1 * M_PI) / 4)
    angle = 0;
  else if (angle >= -(3 * M_PI) / 4)
    angle = -M_PI / 2;
  else
    angle = -M_PI;

  return angle;
}
#else
/* With GPU rotation any angle works, so use it as is. */
static float pd_cook_rotation(float angle)
{
  return angle;
}
#endif

/* Hand angle from the palm detector's wrist and middle-finger keypoints; 0 is upright. */
static float pd_compute_rotation(pd_pp_box_t *box)
{
  float x0, y0, x1, y1;
  float rotation;

  x0 = box->pKps[0].x;
  y0 = box->pKps[0].y;
  x1 = box->pKps[2].x;
  y1 = box->pKps[2].y;

  rotation = M_PI * 0.5 - atan2f(-(y1 - y0), x1 - x0);

  return pd_cook_rotation(pd_normalize_angle(rotation));
}

/* Scale a palm box and its keypoints from 0..1 to screen pixels. */
static void cvt_pd_coord_to_screen_coord(pd_pp_box_t *box)
{
  int i;

  /* This is not a typo. Since screen aspect ratio was conserved. We really want to use LCD_BG_WIDTH for
   * y positions.
   */

  box->x_center *= LCD_BG_WIDTH;
  box->y_center *= LCD_BG_WIDTH;
  box->width *= LCD_BG_WIDTH;
  box->height *= LCD_BG_WIDTH;
  for (i = 0; i < AI_PD_MODEL_PP_NB_KEYPOINTS; i++) {
    box->pKps[i].x *= LCD_BG_WIDTH;
    box->pKps[i].y *= LCD_BG_WIDTH;
  }
}

/* Shift the crop along its own rotated axes, then make it a square scaled from its long
 * side. */
static void roi_shift_and_scale(roi_t *roi, float shift_x, float shift_y, float scale_x, float scale_y)
{
  float long_side;
  float sx, sy;

  sx = (roi->w * shift_x * cos(roi->rotation) - roi->h * shift_y * sin(roi->rotation));
  sy = (roi->w * shift_x * sin(roi->rotation) + roi->h * shift_y * cos(roi->rotation));

  roi->cx += sx;
  roi->cy += sy;

  long_side = MAX(roi->w, roi->h);
  roi->w = long_side;
  roi->h = long_side;

  roi->w *= scale_x;
  roi->h *= scale_y;
}

/* Palm box to landmark crop: rotated upright, moved towards the fingers, enlarged 2.6x. */
static void pd_box_to_roi(pd_pp_box_t *box,  roi_t *roi)
{
  const float shift_x = 0;
  const float shift_y = -0.5;
  const float scale = 2.6;

  roi->cx = box->x_center;
  roi->cy = box->y_center;
  roi->w = box->width;
  roi->h = box->height;
  roi->rotation = pd_compute_rotation(box);

  roi_shift_and_scale(roi, shift_x, shift_y, scale, scale);

#if HAS_ROTATION_SUPPORT == 0
  /* In that case we can cancel rotation. This ensure corners are corrected oriented */
  roi->rotation = 0;
#endif
}

/* Deep copy of a palm box; pKps is a pointer, so the keypoints are copied one by one. */
static void copy_pd_box(pd_pp_box_t *dst, pd_pp_box_t *src)
{
  int i;

  dst->prob = src->prob;
  dst->x_center = src->x_center;
  dst->y_center = src->y_center;
  dst->width = src->width;
  dst->height = src->height;
  for (i = 0 ; i < AI_PD_MODEL_PP_NB_KEYPOINTS; i++)
    dst->pKps[i] = src->pKps[i];
}

/* Set up a board button that calls on_click_handler on each press. */
static void button_init(button_t *b, Button_TypeDef id, void (*on_click_handler)(void *), void *cb_args)
{
  int ret;

  ret = BSP_PB_Init(id, BUTTON_MODE_GPIO);
  assert(ret == BSP_ERROR_NONE);

  b->button_id = id;
  b->on_click_handler = on_click_handler;
  b->prev_state = 0;
  b->cb_args = cb_args;
}

/* Poll a button and fire its handler on the press edge only. */
static void button_process(button_t *b)
{
  int state = BSP_PB_GetState(b->button_id);

  if (state != b->prev_state && state && b->on_click_handler)
    b->on_click_handler(b->cb_args);

  b->prev_state = state;
}

static void cpuload_init(cpuload_info_t *cpu_load)
{
  memset(cpu_load, 0, sizeof(cpuload_info_t));
}

/* Sample busy and total time; once a second, shift the history for the 1 s and 5 s
 * averages. */
static void cpuload_update(cpuload_info_t *cpu_load)
{
  int i;

  cpu_load->history[1] = cpu_load->history[0];
  cpu_load->history[0].total = utk_get_total_runtime();
  cpu_load->history[0].thread = cpu_load->history[0].total - utk_get_idle_runtime();
  cpu_load->history[0].tick = HAL_GetTick();

  if (cpu_load->history[1].tick - cpu_load->history[2].tick < 1000)
    return ;

  for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
    cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
}

/* CPU load in percent over the last frame, the last second and the last five seconds. */
static void cpuload_get_info(cpuload_info_t *cpu_load, float *cpu_load_last, float *cpu_load_last_second,
                             float *cpu_load_last_five_seconds)
{
  if (cpu_load_last)
    *cpu_load_last = 100.0 * (cpu_load->history[0].thread - cpu_load->history[1].thread) /
                     (cpu_load->history[0].total - cpu_load->history[1].total);
  if (cpu_load_last_second)
    *cpu_load_last_second = 100.0 * (cpu_load->history[2].thread - cpu_load->history[3].thread) /
                     (cpu_load->history[2].total - cpu_load->history[3].total);
  if (cpu_load_last_five_seconds)
    *cpu_load_last_five_seconds = 100.0 * (cpu_load->history[2].thread - cpu_load->history[7].thread) /
                     (cpu_load->history[2].total - cpu_load->history[7].total);
}

/* Buffer queue from the camera ISR to the nn task: a ring of buffers with 'free' and
 * 'ready' semaphores. */
static int bqueue_init(bqueue_t *bq, int buffer_nb, uint8_t **buffers)
{
  int ret;
  int i;

  if (buffer_nb > BQUEUE_MAX_BUFFERS)
    return -1;

  ret = os_sem_create(&bq->free, buffer_nb, buffer_nb);
  if (ret != OS_OK)
    goto free_sem_error;
  ret = os_sem_create(&bq->ready, 0, buffer_nb);
  if (ret != OS_OK)
    goto ready_sem_error;

  bq->buffer_nb = buffer_nb;
  for (i = 0; i < buffer_nb; i++) {
    assert(buffers[i]);
    bq->buffers[i] = buffers[i];
  }
  bq->free_idx = 0;
  bq->ready_idx = 0;

  return 0;

ready_sem_error:
  tk_del_sem(bq->free);
free_sem_error:
  return -1;
}

/* Next free buffer for the camera to fill, or NULL if none (non-blocking). */
static uint8_t *bqueue_get_free(bqueue_t *bq, int is_blocking)
{
  uint8_t *res;
  int ret;

  ret = os_sem_take(bq->free, is_blocking ? OS_BLOCKING : OS_NON_BLOCKING);
  if (ret != OS_OK)
    return NULL;

  res = bq->buffers[bq->free_idx];
  bq->free_idx = (bq->free_idx + 1) % bq->buffer_nb;

  return res;
}

/* Return a consumed buffer to the free pool. */
static void bqueue_put_free(bqueue_t *bq)
{
  int ret;

  ret = os_sem_give(bq->free);
  assert(ret == OS_OK);
}

/* Block until the camera has filled a buffer, then return it. */
static uint8_t *bqueue_get_ready(bqueue_t *bq)
{
  uint8_t *res;
  int ret;

  ret = os_sem_take(bq->ready, OS_BLOCKING);
  assert(ret == OS_OK);

  res = bq->buffers[bq->ready_idx];
  bq->ready_idx = (bq->ready_idx + 1) % bq->buffer_nb;

  return res;
}

/* Mark the buffer the camera just filled as ready. */
static void bqueue_put_ready(bqueue_t *bq)
{
  int ret;

  /* The FreeRTOS version branched on interrupt context to pick between
   * xSemaphoreGive() and xSemaphoreGiveFromISR(). micro T-Kernel's
   * tk_sig_sem() is valid in both the task and task-independent portions and
   * reschedules on return by itself, so one call covers both cases. */
  ret = os_sem_give(bq->ready);
  assert(ret == OS_OK);
  (void)ret;
}

/* Point the LCD background layer at another camera frame. */
static void reload_bg_layer(int next_disp_idx)
{
  int ret;

  ret = SCRL_SetAddress_NoReload(lcd_bg_buffer[next_disp_idx], SCRL_LAYER_0);
  assert(ret == 0);
  ret = SCRL_ReloadLayer(SCRL_LAYER_0);
  assert(ret == 0);

  ret = SRCL_Update();
  assert(ret == 0);
}

/* Display pipe frame done (ISR): capture into the next buffer, show the one just
 * filled. */
static void app_main_pipe_frame_event()
{
  int next_disp_idx = (lcd_bg_buffer_disp_idx + 1) % DISPLAY_BUFFER_NB;
  int next_capt_idx = (lcd_bg_buffer_capt_idx + 1) % DISPLAY_BUFFER_NB;
  int ret;

  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t) lcd_bg_buffer[next_capt_idx]);
  assert(ret == HAL_OK);

  reload_bg_layer(next_disp_idx);
  lcd_bg_buffer_disp_idx = next_disp_idx;
  lcd_bg_buffer_capt_idx = next_capt_idx;

  frame_event_nb++;
}


/* NN pipe frame done (ISR): pass the frame to the nn task, or drop it if no buffer is
 * free. */
static void app_ancillary_pipe_frame_event()
{
  uint8_t *next_buffer;
  int ret;

  next_buffer = bqueue_get_free(&nn_input_queue, 0);
  if (next_buffer) {
    ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE2,
                                           DCMIPP_MEMORY_ADDRESS_0, (uint32_t) next_buffer);
    assert(ret == HAL_OK);
    /* minus 1 since app_main_pipe_frame_event occur before app_ancillary_pipe_frame_event() */
    frame_event_nb_for_resize = frame_event_nb - 1;
    bqueue_put_ready(&nn_input_queue);
  }
}

/* Vsync (ISR): wake the isp task. */
static void app_main_pipe_vsync_event()
{
  /* Signalled from the camera vsync interrupt. os_sem_give() tolerates the
   * count already being at its maximum, which happens when the isp task has
   * not yet consumed the previous frame's signal -- that is not an error, the
   * task is going to run either way. */
  (void)os_sem_give(isp_sem);
}

/* Clamp a point to the screen; returns 1 if it had to move. */
static int clamp_point(int *x, int *y)
{
  int xi = *x;
  int yi = *y;

  if (*x < 0)
    *x = 0;
  if (*y < 0)
    *y = 0;
  if (*x >= lcd_bg_area.XSize)
    *x = lcd_bg_area.XSize - 1;
  if (*y >= lcd_bg_area.YSize)
    *y = lcd_bg_area.YSize - 1;

  return (xi != *x) || (yi != *y);
}

/* As clamp_point(), keeping a margin from the edges. */
static int clamp_point_with_margin(int *x, int *y, int margin)
{
  int xi = *x;
  int yi = *y;

  if (*x < margin)
    *x = margin;
  if (*y < margin)
    *y = margin;
  if (*x >= lcd_bg_area.XSize - margin)
    *x = lcd_bg_area.XSize - margin - 1;
  if (*y >= lcd_bg_area.YSize - margin)
    *y = lcd_bg_area.YSize - margin - 1;

  return (xi != *x) || (yi != *y);
}

/* Draw the palm detector's box and keypoints. */
static void display_pd_hand(pd_pp_box_t *hand)
{
  int xc, yc;
  int x0, y0;
  int x1, y1;
  int w, h;
  int i;

  /* display box around palm */
  xc = (int)hand->x_center;
  yc = (int)hand->y_center;
  w = (int)hand->width;
  h = (int)hand->height;
  x0 = xc - (w + 1) / 2;
  y0 = yc - (h + 1) / 2;
  x1 = xc + (w + 1) / 2;
  y1 = yc + (h + 1) / 2;
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);
  UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);

  /* display palm key points */
  for (i = 0; i < 7; i++) {
    uint32_t color = (i != 0 && i != 2) ? UTIL_LCD_COLOR_RED : UTIL_LCD_COLOR_BLUE;

    x0 = (int)hand->pKps[i].x;
    y0 = (int)hand->pKps[i].y;
    clamp_point(&x0, &y0);
    UTIL_LCD_FillCircle(x0, y0, 2, color);
  }
}

/* Rotate a 2-D point about the origin. */
static void rotate_point(float pt[2], float rotation)
{
  float x = pt[0];
  float y = pt[1];

  pt[0] = cos(rotation) * x - sin(rotation) * y;
  pt[1] = sin(rotation) * x + cos(rotation) * y;
}

/* The four screen corners of a rotated crop. */
static void roi_to_corners(roi_t *roi, float corners[4][2])
{
  const float corners_init[4][2] = {
    {-roi->w / 2, -roi->h / 2},
    { roi->w / 2, -roi->h / 2},
    { roi->w / 2,  roi->h / 2},
    {-roi->w / 2,  roi->h / 2},
  };
  int i;

  memcpy(corners, corners_init, sizeof(corners_init));
  /* rotate */
  for (i = 0; i < 4; i++)
    rotate_point(corners[i], roi->rotation);

  /* shift */
  for (i = 0; i < 4; i++) {
    corners[i][0] += roi->cx;
    corners[i][1] += roi->cy;
  }
}

/* Clamp four corners to the screen; returns 1 if any moved. */
static int clamp_corners(float corners_in[4][2], int corners_out[4][2])
{
  int is_clamp = 0;
  int i;

  for (i = 0; i < 4; i++) {
    corners_out[i][0] = (int)corners_in[i][0];
    corners_out[i][1] = (int)corners_in[i][1];
    is_clamp |= clamp_point(&corners_out[i][0], &corners_out[i][1]);
  }

  return is_clamp;
}

/* Draw the landmark crop outline in red; skipped if it runs off the screen. */
static void display_roi(roi_t *roi)
{
  float corners_f[4][2];
  int corners[4][2];
  int is_clamp;
  int i;

  /* compute box corners */
  roi_to_corners(roi, corners_f);

  /* clamp */
  is_clamp = clamp_corners(corners_f, corners);
  if (is_clamp)
    return ;

  /* display */
  for (i = 0; i < 4; i++)
    UTIL_LCD_DrawLine(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                      UTIL_LCD_COLOR_RED);
}

/* Map a landmark from crop coordinates (0..1) back to screen pixels. */
static void decode_ld_landmark(roi_t *roi, ld_point_t *lm, ld_point_t *decoded)
{
  float rotation = roi->rotation;
  float w = roi->w;
  float h = roi->h;

  decoded->x = roi->cx + (lm->x - 0.5) * w * cos(rotation) - (lm->y - 0.5) * h * sin(rotation);
  decoded->y = roi->cy + (lm->x - 0.5) * w * sin(rotation) + (lm->y - 0.5) * h * cos(rotation);
  /* z is a depth, so the in-plane rotation does not touch it -- but it must
   * take the same scale factor as x and y, or the HaGRID classifier's
   * features, which are ratios of the three, come out wrong.
   * roi_shift_and_scale() forces w == h on every ROI this code produces, so
   * one factor covers both. */
  decoded->z = lm->z * w;
}

/* Draw one hand's 21 landmarks and its skeleton. */
static void display_ld_hand(hand_info_t *hand)
{
  const int disk_radius = DISK_RADIUS;
  roi_t *roi = &hand->roi;
  int x[LD_LANDMARK_NB];
  int y[LD_LANDMARK_NB];
  int is_clamped[LD_LANDMARK_NB];
  ld_point_t decoded;
  int i;

  for (i = 0; i < LD_LANDMARK_NB; i++) {
    decode_ld_landmark(roi, &hand->ld_landmarks[i], &decoded);
    x[i] = (int)decoded.x;
    y[i] = (int)decoded.y;
    is_clamped[i] = clamp_point_with_margin(&x[i], &y[i], disk_radius);
  }

  for (i = 0; i < LD_LANDMARK_NB; i++) {
    if (is_clamped[i])
      continue;
    UTIL_LCD_FillCircle(x[i], y[i], disk_radius, UTIL_LCD_COLOR_YELLOW);
  }

  for (i = 0; i < LD_BINDING_NB; i++) {
    if (is_clamped[ld_bindings_idx[i][0]] || is_clamped[ld_bindings_idx[i][1]])
      continue;
    UTIL_LCD_DrawLine(x[ld_bindings_idx[i][0]], y[ld_bindings_idx[i][0]],
                      x[ld_bindings_idx[i][1]], y[ld_bindings_idx[i][1]],
                      UTIL_LCD_COLOR_BLACK);
  }
}

/* Where a hand's label goes: centred under its bounding box, so it tracks the
 * hand itself.
 *
 * It used to be derived from hmi_state_t's per-stick palm coordinates, which
 * was wrong twice over. Those are only written by hmi_update_stick(), which
 * presenter mode never calls -- so every presenter label sat at the origin,
 * underneath the MENU tab. And they were subscripted by the TRACK index while
 * that array is indexed by stick, so even in HMI mode two hands could take
 * each other's position. The bounding box is carried per hand and filled in by
 * every mode, so it is the honest source. */
static void hand_label_pos(const hand_info_t *hand, int text_chars, int *x, int *y)
{
  const pd_pp_box_t *box = &hand->pd_hands;

  *x = (int)box->x_center - text_chars * (int)LCD_FONT.Width / 2;
  *y = (int)(box->y_center + box->height / 2) + 6;
}

/* Draw whichever hand overlays the menu has switched on. */
void display_hand(display_info_t *info, hand_info_t *hand)
{
  if (info->is_pd_displayed) {
    display_pd_hand(&hand->pd_hands);
    display_roi(&hand->roi);
  }
  if (info->is_ld_displayed)
    display_ld_hand(hand);
}

/* A stick gauge: outer ring = full deflection, cross = neutral, dot = stick.
 * The gauge for the LEFT stick is always on the left of the screen, whatever
 * side the hand driving it is on -- it depicts the transmitter, not the hand.
 * A ring is also drawn at the hand's neutral point on the camera image so the
 * user can see where "centre" is for each hand, with the dead zone inside. */
#define STICK_GAUGE_R   40
/* Room under the gauge for its label. Without this the label sat 10 px off the
 * bottom of the screen, which on this hardware corrupts the next frame
 * buffer rather than merely clipping -- see ui_draw_label(). */
#define STICK_LABEL_H   24
#define STICK_GAUGE_Y   (LCD_BG_HEIGHT - STICK_GAUGE_R - STICK_LABEL_H - 8)
#define STICK_NEUTRAL_R ((int)(HMI_RANGE_FRAC * LCD_BG_WIDTH))


/* Draw one stick gauge: axis position, and in assisted mode the selected axis. */
static void display_stick(hmi_stick_t *st, int stick, hmi_mode_t mode)
{
  int gx = (stick == HMI_LEFT) ? STICK_GAUGE_R + 12 : LCD_BG_WIDTH - STICK_GAUGE_R - 12;
  int gy = STICK_GAUGE_Y;
  uint32_t color = st->is_valid ? UTIL_LCD_COLOR_GREEN : UTIL_LCD_COLOR_GRAY;
  int dx, dy;

  if (mode == HMI_MODE_ACROBATIC) {
    /* the fixed control box this hand moves inside */
    float nx, ny;

    hmi_neutral_point(stick, &nx, &ny);
    UTIL_LCD_DrawCircle((int)nx, (int)ny, STICK_NEUTRAL_R, color);
    UTIL_LCD_DrawCircle((int)nx, (int)ny, (int)(HMI_DEADZONE * STICK_NEUTRAL_R), color);
  } else
  /* Assisted mode has no fixed control box -- movement is measured from where
   * the gesture began. Show that origin, and the one direction that is live,
   * so it is obvious which axis a grip has hold of. */
  if (st->selected != HMI_AXIS_NONE) {
    const int reach = (int)(HMI_ASSIST_RANGE_FRAC * LCD_BG_WIDTH);
    int ox = (int)st->grab_x;
    int oy = (int)st->grab_y;
    int lo;

    clamp_point(&ox, &oy);
    UTIL_LCD_FillCircle(ox, oy, 3, UTIL_LCD_COLOR_CYAN);
    if (st->selected == HMI_AXIS_VERT) {
      lo = MAX(oy - reach, 0);
      UTIL_LCD_DrawVLine(ox, lo, MIN(2 * reach, (int)lcd_bg_area.YSize - lo), UTIL_LCD_COLOR_CYAN);
    } else {
      lo = MAX(ox - reach, 0);
      UTIL_LCD_DrawHLine(lo, oy, MIN(2 * reach, (int)lcd_bg_area.XSize - lo), UTIL_LCD_COLOR_CYAN);
    }
  }

  /* the gauge */
  UTIL_LCD_DrawCircle(gx, gy, STICK_GAUGE_R, color);
  UTIL_LCD_DrawHLine(gx - STICK_GAUGE_R, gy, 2 * STICK_GAUGE_R, UTIL_LCD_COLOR_GRAY);
  UTIL_LCD_DrawVLine(gx, gy - STICK_GAUGE_R, 2 * STICK_GAUGE_R, UTIL_LCD_COLOR_GRAY);
  dx = (int)(st->x * STICK_GAUGE_R);
  dy = -(int)(st->y * STICK_GAUGE_R);   /* +y is up */
  UTIL_LCD_FillCircle(gx + dx, gy + dy, 5, st->is_valid ? UTIL_LCD_COLOR_RED : UTIL_LCD_COLOR_GRAY);
  if (mode == HMI_MODE_ACROBATIC) {
    ui_draw_label(gx - 8, gy + STICK_GAUGE_R + 2, "%s", stick == HMI_LEFT ? "L" : "R");
  } else {
    /* Which axis this hand has selected is the most useful thing on screen in
     * assisted mode -- without it you cannot tell a released hand from one
     * whose movement is deliberately being ignored. */
    ui_draw_label(gx - 24, gy + STICK_GAUGE_R + 2, "%s %s",
                  stick == HMI_LEFT ? "L" : "R",
                  hmi_axis_label(stick, st->selected));
  }
}

/* Presenter HUD: what the two hands are doing and what was last sent. The
 * pointer is invisible on this screen -- it lives on the host -- so the only
 * feedback the user gets that a fist is being tracked is here. */
#if APP_HAS_PRESENTER
/* Draw the presenter status: each hand's label and pose, and the last key sent. */
static void display_presenter(display_info_t *info)
{
  const presenter_t *p = &info->pres;
  int y = LCD_BG_HEIGHT - 3 * (int)LCD_FONT.Height - 14;
  int i;

  for (i = 0; i < info->pd_hand_nb; i++) {
    hand_info_t *hand = &info->hands[i];
    int lx, ly;

    if (!hand->is_valid)
      continue;
    const char *label = hand_pose_label((hand_pose_t)hand->pose);

    hand_label_pos(hand, (int)strlen(label), &lx, &ly);
    UTIL_LCD_SetTextColor(hand->pose == HAND_POSE_NONE ? UTIL_LCD_COLOR_GREEN
                                                       : UTIL_LCD_COLOR_CYAN);
    ui_draw_label(lx, ly, "%s", label);
  }

  UTIL_LCD_SetTextColor(p->pointer_active ? UTIL_LCD_COLOR_CYAN : UTIL_LCD_COLOR_GRAY);
  ui_draw_label(12, y, "pointer %s", p->pointer_active ? "ON " : "off");
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
  ui_draw_label(12, y + (int)LCD_FONT.Height + 4, "slide %s",
                p->last_key == PRES_KEY_NONE ? "----" : presenter_key_label(p->last_key));
}
#endif /* APP_HAS_PRESENTER */

static void Display_NetworkOutput(display_info_t *info)
{
  float cpu_load_one_second;
  int line_nb = 0;
  float nn_fps;
  int i;

  /* clear previous ui */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */

  if (info->ui.asleep) {
    /* No HUD, no gauges, no labels: there is nothing behind them to annotate,
     * and every pixel drawn is work the sleep exists to avoid. */
    ui_draw(&info->ui);

    return;
  }

  /* cpu load */
  cpuload_update(&cpu_load);
  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);

  /* draw metrics */
  nn_fps = 1000.0 / info->nn_period_ms;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%s", app_mode_name(info->ui.mode));
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "Cpu load");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "   %.1f%%", cpu_load_one_second);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inferences");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " pd %3ums", info->pd_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " hl %3ums", info->hl_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " gs %3uus", info->gesture_us);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "  %.1f FPS", nn_fps);
  line_nb += 2;
  if (DBG_INFO) {
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Display");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->disp_ms);
    line_nb += 1;
  }

  /* display palm detector output */
  for (i = 0; i < info->pd_hand_nb; i++) {
    if (info->hands[i].is_valid)
      display_hand(info, &info->hands[i]);
  }

  /* per-hand label next to the palm: which stick it drives + its gesture */
  for (i = 0; app_mode_is_hmi(info->ui.mode) && i < info->pd_hand_nb; i++) {
    hand_info_t *hand = &info->hands[i];
    hmi_stick_t *st;
    int x, y;

    if (!hand->is_valid || hand->stick < 0)
      continue;
    st = &info->hmi.stick[hand->stick];
    /* "L " plus the gesture name */
    hand_label_pos(hand, 2 + (int)strlen(gesture_label(hand->gesture_cls)), &x, &y);
    UTIL_LCD_SetTextColor(st->selected != HMI_AXIS_NONE ? UTIL_LCD_COLOR_CYAN : UTIL_LCD_COLOR_GREEN);
    ui_draw_label(x, y, "%s %s", hand->stick == HMI_LEFT ? "L" : "R",
                  gesture_label(hand->gesture_cls));
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
  }

  if (app_mode_is_hmi(info->ui.mode)) {
    /* the two virtual sticks, drawn where they sit on a real transmitter */
    display_stick(&info->hmi.stick[HMI_LEFT], HMI_LEFT, info->hmi.mode);
    display_stick(&info->hmi.stick[HMI_RIGHT], HMI_RIGHT, info->hmi.mode);
  }
#if APP_HAS_PRESENTER
  else {
    display_presenter(info);
  }
#endif

  if (DBG_INFO)
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "pd : %5.1f %%", info->pd_max_prob * 100);

  ui_draw(&info->ui);
}

/* Look up the palm detector's input and output buffers. */
static void palm_detector_init(pd_model_info_t *info)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_palm_detector();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_palm_detector();
  int ret;

  /* model info */
  info->nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  info->prob_out = (float *) LL_Buffer_addr_start(&nn_out_info[0]);
  info->prob_out_len = LL_Buffer_len(&nn_out_info[0]);
  assert(info->prob_out_len == AI_PD_MODEL_PP_TOTAL_DETECTIONS * sizeof(float));
  info->boxes_out = (float *) LL_Buffer_addr_start(&nn_out_info[1]);
  info->boxes_out_len = LL_Buffer_len(&nn_out_info[1]);
  assert(info->boxes_out_len == AI_PD_MODEL_PP_TOTAL_DETECTIONS * sizeof(float) * 18);

  /* post processor info */
  ret = app_postprocess_init(&info->static_param, &NN_Instance_palm_detector);
  assert(ret == AI_PD_POSTPROCESS_ERROR_NO);
}

/* Run the palm detector on one frame; returns the number of palms found. */
static int palm_detector_run(uint8_t *buffer, pd_model_info_t *info, uint32_t *pd_exec_time)
{
  uint32_t start_ts;
  int hand_nb;
  int ret;
  int i;

  start_ts = HAL_GetTick();
  /* Note that we don't need to clean/invalidate those input buffers since they are only access in hardware */
  ret = LL_ATON_Set_User_Input_Buffer_palm_detector(0, buffer, info->nn_in_len);
  assert(ret == LL_ATON_User_IO_NOERROR);

  LL_ATON_RT_Main(&NN_Instance_palm_detector);

  ret = app_postprocess_run((void * []){info->prob_out, info->boxes_out}, 2, &info->pd_out, &info->static_param);
  assert(ret == AI_PD_POSTPROCESS_ERROR_NO);
  hand_nb = MIN(info->pd_out.box_nb, PD_MAX_CANDIDATES);

  for (i = 0; i < hand_nb; i++) {
    cvt_pd_coord_to_screen_coord(&info->pd_out.pOutData[i]);
    pd_box_to_roi(&info->pd_out.pOutData[i], &rois[i]);
  }

  /* Discard nn_out region (used by pp_outputs variables) to avoid Dcache evictions during nn inference */
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->prob_out, info->prob_out_len));
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->boxes_out, info->boxes_out_len));

  *pd_exec_time = HAL_GetTick() - start_ts;

  return hand_nb;
}

/* Look up the landmark model's input and output buffers. */
static void hand_landmark_init(hl_model_info_t *info)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_hand_landmark();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_hand_landmark();

  info->nn_in = LL_Buffer_addr_start(&nn_in_info[0]);
  info->nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  info->prob_out = (float *) LL_Buffer_addr_start(&nn_out_info[2]);
  info->prob_out_len = LL_Buffer_len(&nn_out_info[2]);
  assert(info->prob_out_len == sizeof(float));
  info->landmarks_out = (float *) LL_Buffer_addr_start(&nn_out_info[3]);
  info->landmarks_out_len = LL_Buffer_len(&nn_out_info[3]);
  assert(info->landmarks_out_len == sizeof(float) * 63);
}

#if HAS_ROTATION_SUPPORT == 0
/* No GPU: crop and resize the crop into the model input on the CPU. */
static int hand_landmark_prepare_input(uint8_t *buffer, roi_t *roi, hl_model_info_t *info)
{
  float corners_f[4][2];
  int corners[4][2];
  uint8_t* out_data;
  size_t height_out;
  uint8_t *in_data;
  size_t height_in;
  size_t width_out;
  size_t width_in;
  int is_clamped;

  /* defaults when no clamping occurs */
  out_data = info->nn_in;
  width_out = LD_WIDTH;
  height_out = LD_HEIGHT;

  roi_to_corners(roi, corners_f);
  is_clamped = clamp_corners(corners_f, corners);

  /* If clamp perform a partial resize */
  if (is_clamped) {
    int offset_x;
    int offset_y;

    /* clear target memory since resize will partially write it */
    memset(info->nn_in, 0, info->nn_in_len);

    /* compute start address of output buffer */
    offset_x = (int)(((corners[0][0] - corners_f[0][0]) * LD_WIDTH) / (corners_f[2][0] - corners_f[0][0]));
    offset_y = (int)(((corners[0][1] - corners_f[0][1]) * LD_HEIGHT) / (corners_f[2][1] - corners_f[0][1]));
    out_data += offset_y * (int)LD_WIDTH * DISPLAY_BPP + offset_x * DISPLAY_BPP;

    /* compute output width and height */
    width_out = (int)((corners[2][0] - corners[0][0]) / (corners_f[2][0] - corners_f[0][0]) * LD_WIDTH);
    height_out = (int)((corners[2][1] - corners[0][1]) / (corners_f[2][1] - corners_f[0][1]) * LD_HEIGHT);

    assert(width_out > 0);
    assert(height_out > 0);
    {
      uint8_t* out_data_end;

      out_data_end = out_data + (int)LD_WIDTH * DISPLAY_BPP * (height_out - 1) + DISPLAY_BPP * width_out - 1;

      assert(out_data_end >= info->nn_in);
      assert(out_data_end < info->nn_in + info->nn_in_len);
    }
  }

  in_data = buffer + corners[0][1] * LCD_BG_WIDTH * DISPLAY_BPP + corners[0][0]* DISPLAY_BPP;
  width_in = corners[2][0] - corners[0][0];
  height_in = corners[2][1] - corners[0][1];

  assert(width_in > 0);
  assert(height_in > 0);
  {
    uint8_t* in_data_end;

    in_data_end = in_data + LCD_BG_WIDTH * DISPLAY_BPP * (height_in - 1) + DISPLAY_BPP * width_in - 1;

    assert(in_data_end >= buffer);
    assert(in_data_end < buffer + LCD_BG_WIDTH * LCD_BG_HEIGHT * DISPLAY_BPP);
  }

  IPL_resize_bilinear_iu8ou8_with_strides_RGB(in_data, out_data, LCD_BG_WIDTH * DISPLAY_BPP, LD_WIDTH * DISPLAY_BPP,
                                              width_in, height_in, width_out, height_out);

  return 0;
}
#else
/* Multiply a 3x3 matrix by a homogeneous point, in place. */
static void app_transform(nema_matrix3x3_t t, app_v3_t v)
{
  app_v3_t r;
  int i;

  for (i = 0; i < 3; i++)
    r[i] = t[i][0] * v[0] + t[i][1] * v[1] + t[i][2] * v[2];

  for (i = 0; i < 3; i++)
    v[i] = r[i];
}

/* Crop, rotate upright and resize into the model input with the GPU2D. */
static int hand_landmark_prepare_input(uint8_t *buffer, roi_t *roi, hl_model_info_t *info)
{
  app_v3_t vertex[] = {
    {           0,             0, 1},
    {LCD_BG_WIDTH,             0, 1},
    {LCD_BG_WIDTH, LCD_BG_HEIGHT, 1},
    {           0, LCD_BG_HEIGHT, 1},
  };
  GFXMMU_BuffersTypeDef buffers = { 0 };
  nema_matrix3x3_t t;
  int ret;
  int i;

  buffers.Buf0Address = (uint32_t) info->nn_in;
  ret = HAL_GFXMMU_ModifyBuffers(&hgfxmmu, &buffers);
  assert(ret == HAL_OK);

  /* bind destination texture */
  nema_bind_dst_tex(GFXMMU_VIRTUAL_BUFFER0_BASE, LD_WIDTH, LD_HEIGHT, NEMA_RGBA8888, -1);
  nema_set_clip(0, 0, LD_WIDTH, LD_HEIGHT);
  nema_clear(0);
  /* bind source texture */
  nema_bind_src_tex((uintptr_t) buffer, LCD_BG_WIDTH, LCD_BG_HEIGHT, NEMA_RGBA8888, -1, NEMA_FILTER_BL);
  nema_enable_tiling(1);
  nema_set_blend_blit(NEMA_BL_SRC);

  /* let's go */
  nema_mat3x3_load_identity(t);
  nema_mat3x3_translate(t, -roi->cx, -roi->cy);
  nema_mat3x3_rotate(t, nema_rad_to_deg(-roi->rotation));
  nema_mat3x3_scale(t, LD_WIDTH / roi->w, LD_HEIGHT / roi->h);
  nema_mat3x3_translate(t, LD_WIDTH / 2, LD_HEIGHT / 2);
  for (i = 0 ; i < 4; i++)
    app_transform(t, vertex[i]);
  nema_blit_quad_fit(vertex[0][0], vertex[0][1], vertex[1][0], vertex[1][1],
                     vertex[2][0], vertex[2][1], vertex[3][0], vertex[3][1]);

  nema_cl_submit(&cl);
  nema_cl_wait(&cl);
  HAL_ICACHE_Invalidate();

  assert(!nema_get_error());

  return 0;
}
#endif

/* Run the landmark model on one crop; returns 1 if a hand was found. */
static int hand_landmark_run(uint8_t *buffer, hl_model_info_t *info, roi_t *roi,
                             ld_point_t ld_landmarks[LD_LANDMARK_NB])
{
  int is_clamped;
  int is_valid;

  is_clamped = hand_landmark_prepare_input(buffer, roi, info);
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(info->nn_in, info->nn_in_len));
  if (is_clamped)
    return 0;

  LL_ATON_RT_Main(&NN_Instance_hand_landmark);

  is_valid = ld_post_process(info->prob_out, info->landmarks_out, ld_landmarks);

  /* Discard nn_out region (used by pp_input and pp_outputs variables) to avoid Dcache evictions during nn inference */
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->prob_out, info->prob_out_len));
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->landmarks_out, info->landmarks_out_len));

  return is_valid;
}

#if HAS_ROTATION_SUPPORT == 1
/* Set up NemaGFX and the GFXMMU used to rotate crops. */
static void app_rot_init(hl_model_info_t *info)
{
  GFXMMU_PackingTypeDef packing = { 0 };
  int ret;

  printf("init nema\n");
  nema_init();
  assert(!nema_get_error());
  nema_ext_hold_enable(2);
  nema_ext_hold_irq_enable(2);
  nema_ext_hold_enable(3);
  nema_ext_hold_irq_enable(3);
  printf("init nema DONE %s\n", nema_get_sw_device_name());

  hgfxmmu.Instance = GFXMMU;
  hgfxmmu.Init.BlockSize = GFXMMU_12BYTE_BLOCKS;
  hgfxmmu.Init.AddressTranslation = DISABLE;
  ret = HAL_GFXMMU_Init(&hgfxmmu);
  assert(ret == HAL_OK);

  packing.Buffer0Activation = ENABLE;
  packing.Buffer0Mode       = GFXMMU_PACKING_MSB_REMOVE;
  packing.DefaultAlpha      = 0xff;
  ret = HAL_GFXMMU_ConfigPacking(&hgfxmmu, &packing);
  assert(ret == HAL_OK);

  cl = nema_cl_create_sized(8192);
  nema_cl_bind_circular(&cl);
}
#endif

/* Hand angle from the wrist (0) to the middle-finger base (9); 0 is upright. */
static float ld_compute_rotation(ld_point_t lm[LD_LANDMARK_NB])
{
  float x0, y0, x1, y1;
  float rotation;

  x0 = lm[0].x;
  y0 = lm[0].y;
  x1 = lm[9].x;
  y1 = lm[9].y;

  rotation = M_PI * 0.5 - atan2f(-(y1 - y0), x1 - x0);

  return pd_cook_rotation(pd_normalize_angle(rotation));
}

/* Crop around the palm and finger-base landmarks, plus a palm box for the next frame. */
static void ld_to_roi(ld_point_t lm[LD_LANDMARK_NB], roi_t *roi, pd_pp_box_t *next_pd)
{
  const int pd_to_ld_idx[AI_PD_MODEL_PP_NB_KEYPOINTS] = {0, 5, 9, 13, 17, 1, 2};
  const int indices[] = {0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18};
  float max_x, max_y, min_x, min_y;
  int i;

  max_x = max_y = -10000;
  min_x = min_y =  10000;

  roi->rotation = ld_compute_rotation(lm);

  for (i = 0; i < ARRAY_NB(indices); i++) {
    max_x = MAX(max_x, lm[indices[i]].x);
    max_y = MAX(max_y, lm[indices[i]].y);
    min_x = MIN(min_x, lm[indices[i]].x);
    min_y = MIN(min_y, lm[indices[i]].y);
  }

  roi->cx = (max_x + min_x) / 2;
  roi->cy = (max_y + min_y) / 2;
  roi->w = (max_x - min_x);
  roi->h = (max_y - min_y);

  next_pd->x_center = roi->cx;
  next_pd->y_center = roi->cy;
  next_pd->width = roi->w;
  next_pd->height = roi->h;
  for (i = 0; i < AI_PD_MODEL_PP_NB_KEYPOINTS; i++) {
    next_pd->pKps[i].x = lm[pd_to_ld_idx[i]].x;
    next_pd->pKps[i].y = lm[pd_to_ld_idx[i]].y;
  }
}

/* Predict the next frame's crop from this frame's landmarks, so a tracked hand needs no
 * palm detection. */
static void compute_next_roi(roi_t *src, ld_point_t lm_in[LD_LANDMARK_NB], roi_t *next, pd_pp_box_t *next_pd)
{
  const float shift_x = 0;
  const float shift_y = -0.1;
  const float scale = 2.0;
  ld_point_t lm[LD_LANDMARK_NB];
  roi_t roi;
  int i;

  for (i = 0; i < LD_LANDMARK_NB; i++)
    decode_ld_landmark(src, &lm_in[i], &lm[i]);

  ld_to_roi(lm, &roi, next_pd);
  roi_shift_and_scale(&roi, shift_x, shift_y, scale, scale);

#if HAS_ROTATION_SUPPORT == 0
  /* In that case we can cancel rotation. This ensure corners are corrected oriented */
  roi.rotation = 0;
#endif

  *next = roi;
}

/* One tracked hand. The palm detector seeds a slot; from then on the slot
 * follows the hand by predicting the next ROI from the landmarks, exactly as
 * the single-hand original did -- there are simply two of them now. */
typedef struct {
  int is_tracking;
  roi_t roi;                                          /* ROI used this frame */
  roi_t roi_next;                                     /* predicted for next frame */
  pd_pp_box_t box;                                    /* palm box, screen coords */
  pd_pp_point_t box_kps[AI_PD_MODEL_PP_NB_KEYPOINTS];
  pd_pp_box_t box_next;
  pd_pp_point_t box_next_kps[AI_PD_MODEL_PP_NB_KEYPOINTS];
  ld_point_t lm[LD_LANDMARK_NB];                      /* ROI-local, from the model */
  ld_point_t lm_screen[LD_LANDMARK_NB];               /* decoded to screen coords */
  gesture_filter_t gesture_filter;      /* HMI modes: 3-class Open/Close/Pointer */
#if APP_HAS_PRESENTER
  hand_pose_filter_t pose_filter;       /* presenter mode: HaGRID -> FIST/OK    */
  hand_pose_t pose;
  int hand;                             /* presenter: PRES_LEFT/RIGHT, or -1    */
#endif
  int gesture_cls;
  float gesture_prob;
  float palm_x;                                       /* screen coords */
  float palm_y;
  int stick;                                          /* HMI_LEFT / HMI_RIGHT / -1 */
} track_t;

/* Reset a tracking slot completely. */
static void track_init(track_t *t)
{
  memset(t, 0, sizeof(*t));
  t->box.pKps = t->box_kps;
  t->box_next.pKps = t->box_next_kps;
  t->gesture_cls = GESTURE_NONE;
  t->stick = -1;
#if APP_HAS_PRESENTER
  t->hand = -1;
  hand_pose_filter_init(&t->pose_filter);
#endif
}

/* Stop tracking in a slot and forget its gesture, stick and hand. */
static void track_drop(track_t *t)
{
  t->is_tracking = 0;
  gesture_filter_init(&t->gesture_filter);
  t->gesture_cls = GESTURE_NONE;
  t->gesture_prob = 0;
  t->stick = -1;
#if APP_HAS_PRESENTER
  hand_pose_filter_init(&t->pose_filter);
  t->pose = HAND_POSE_NONE;
  t->hand = -1;
#endif
}

/* Is this fresh palm detection the hand that slot t is already following?
 * Centre-inside-ROI is enough: the ROI is 2x the hand, so a second hand that
 * close would be overlapping the first anyway. */
static int track_owns_box(track_t *t, pd_pp_box_t *box)
{
  float dx, dy;

  if (!t->is_tracking)
    return 0;
  dx = box->x_center - t->roi.cx;
  dy = box->y_center - t->roi.cy;

  return fabsf(dx) < t->roi.w * 0.5f && fabsf(dy) < t->roi.h * 0.5f;
}

/* Give every fresh detection that is not already tracked to a free slot,
 * best-scoring first. */
static void track_seed_from_pd(track_t tracks[PD_MAX_HAND_NB], pd_model_info_t *pd_info, int cand_nb)
{
  int c, t;

  for (c = 0; c < cand_nb; c++) {
    pd_pp_box_t *box = &pd_info->pd_out.pOutData[c];
    int is_owned = 0;

    for (t = 0; t < PD_MAX_HAND_NB; t++)
      is_owned |= track_owns_box(&tracks[t], box);
    if (is_owned)
      continue;

    for (t = 0; t < PD_MAX_HAND_NB; t++) {
      if (tracks[t].is_tracking)
        continue;
      tracks[t].is_tracking = 1;
      tracks[t].roi = rois[c];
      copy_pd_box(&tracks[t].box, box);
      gesture_filter_init(&tracks[t].gesture_filter);
#if APP_HAS_PRESENTER
      hand_pose_filter_init(&tracks[t].pose_filter);
      tracks[t].hand = -1;          /* assigned by presenter_run() this frame */
#endif
      break;
    }
  }
}

/* Third pipeline stage. Runs on the CPU in a few microseconds, so unlike the
 * two NPU stages it is not worth its own timing filter or thread. */
static int gesture_run(ld_point_t lm_screen[LD_LANDMARK_NB], gesture_filter_t *filter,
                       float *prob, uint32_t *exec_us)
{
  gesture_result_t res;
  uint32_t cycles;
  uint32_t start;

  start = DWT->CYCCNT;

  /* The classifier was trained on image-space landmarks, where orientation is
   * real information -- so it gets the screen-space points, not the
   * rotation-normalised ROI-local ones the landmark model produced. */
  gesture_classify(lm_screen, &res);
  *prob = res.prob;

  cycles = DWT->CYCCNT - start;
  *exec_us = (SystemCoreClock >= 1000000) ? cycles / (SystemCoreClock / 1000000) : 0;

  return gesture_filter_push(filter, &res);
}

#if APP_HAS_PRESENTER
/* Presenter mode uses a different classifier entirely: 19 HaGRID classes,
 * of which only "fist" and "ok" mean anything here. Both models are compiled
 * in and only the active mode's runs, so neither costs the other anything. */
static hand_pose_t pose_run(ld_point_t lm_screen[LD_LANDMARK_NB], track_t *t,
                            uint32_t *exec_us)
{
  gh_result_t res;
  uint32_t cycles;
  uint32_t start;

  start = DWT->CYCCNT;

  /* Trained on axis-aligned crops with the hand's real orientation left in --
   * several HaGRID classes differ only by which way the hand points -- so it
   * gets the screen-space landmarks, not the rotation-normalised ROI-local
   * ones the landmark model produced. */
  gesture_hagrid_classify(lm_screen, &res);
  t->gesture_cls = res.cls;
  t->gesture_prob = res.prob;

  cycles = DWT->CYCCNT - start;
  *exec_us = (SystemCoreClock >= 1000000) ? cycles / (SystemCoreClock / 1000000) : 0;

  return hand_pose_filter_push(&t->pose_filter, hand_pose_from_class(res.cls));
}
#endif /* APP_HAS_PRESENTER */

#if USE_USB_HID_GAMEPAD
/* Same data the serial frame carries, as a HID gamepad report. HID Y axes
 * grow downwards, so pitch and throttle are negated. The gesture buttons
 * are one-hot per hand; bits 6/7 say whether a hand is on the stick at all,
 * so the host can tell "centred" from "nobody there". */
static int8_t stick_to_axis(float v)
{
  return (int8_t)lroundf(v * 127.0f);
}

/* Send the drone sticks as a gamepad report; HID Y axes grow downwards, so Y is
 * negated. */
static void hmi_send_usb_report(const hmi_state_t *hmi)
{
  const hmi_stick_t *l = &hmi->stick[HMI_LEFT];
  const hmi_stick_t *r = &hmi->stick[HMI_RIGHT];
  usb_hid_gamepad_report_t rep;

  rep.x  = stick_to_axis(r->x);      /* roll */
  rep.y  = stick_to_axis(-r->y);     /* pitch */
  rep.rx = stick_to_axis(l->x);      /* yaw */
  rep.ry = stick_to_axis(-l->y);     /* throttle */

  /* -1 just means the host has not configured us yet: keep going. */
  (void)usb_hid_gamepad_send(&rep);
}
#endif

#if APP_HAS_PRESENTER
/* Everything the board tells the host in presenter mode, and the only place
 * it does. The device is an ordinary mouse and keyboard, so there is nothing
 * to encode and no protocol to agree on: a relative step moves the pointer, a
 * key tap turns the slide. */
static void presenter_send_usb(const presenter_action_t *act)
{
  if (act->move && (act->dx || act->dy))
    (void)usb_hid_mouse_move((int8_t)act->dx, (int8_t)act->dy);

  if (act->key == PRES_KEY_NEXT)
    (void)usb_hid_key_tap(USB_HID_KEY_PAGEDOWN);
  else if (act->key == PRES_KEY_PREV)
    (void)usb_hid_key_tap(USB_HID_KEY_PAGEUP);

  /* USB interrupts only arrive while transfers are in flight, so with an idle
   * pointer there may be nothing else to wake the service task. */
  usb_hid_poll();
}

/* Fourth stage, presenter mode: hands -> pointer steps and slide keys.
 *
 * Which slot is the user's left hand and which is the right is decided ONCE,
 * when a slot acquires a hand, and then held until it loses it. The HMI modes
 * re-decide every frame from screen position, which is the better choice when
 * the two hands drive symmetric sticks and a dropped track has to re-acquire
 * cleanly. Here the two hands mean different commands, and the right hand
 * swiping right travels towards the middle of the image: re-deciding every
 * frame would let a hand change identity in the middle of its own swipe and
 * turn "next slide" into nothing at all. */
_Static_assert(PD_MAX_HAND_NB == 2 && PRES_HAND_NB == 2,
               "presenter_run() pairs exactly two slots with two hands");

/* Presenter step: give each tracked hand an identity, run the control law, send the
 * result. */
static void presenter_run(track_t tracks[PD_MAX_HAND_NB], presenter_t *pres,
                          uint32_t now_ms, presenter_action_t *act, int ui_owns)
{
  presenter_hand_in_t hands[PRES_HAND_NB];
  int used[PRES_HAND_NB] = {0, 0};
  int t;

  memset(hands, 0, sizeof(hands));

  /* Slots that already know which hand they are keep it, and claim that side
   * first so a newly acquired slot cannot take it from them. */
  for (t = 0; t < PD_MAX_HAND_NB; t++) {
    track_t *cur = &tracks[t];

    if (!cur->is_tracking || cur->hand < 0)
      continue;
    if (used[cur->hand]) {
      cur->hand = -1;          /* both slots claim the same side: re-decide below */
      continue;
    }
    used[cur->hand] = 1;
  }

  for (t = 0; t < PD_MAX_HAND_NB; t++) {
    track_t *cur = &tracks[t];
    track_t *other = &tracks[PD_MAX_HAND_NB - 1 - t];
    float other_x;
    int hand;

    if (!cur->is_tracking) {
      cur->hand = -1;
      continue;
    }
    if (cur->hand >= 0)
      continue;                /* already settled above */

    other_x = other->is_tracking ? other->palm_x : -1.0f;
    hand = presenter_assign_hand(cur->palm_x, other_x);
    if (used[hand]) {
      hand = 1 - hand;         /* the obvious side is taken; the other is free */
      if (used[hand])
        continue;
    }
    used[hand] = 1;
    cur->hand = hand;
  }

  for (t = 0; t < PD_MAX_HAND_NB; t++) {
    track_t *cur = &tracks[t];

    if (!cur->is_tracking || cur->hand < 0)
      continue;
    hands[cur->hand].is_valid = 1;
    hands[cur->hand].pose = cur->pose;
    hands[cur->hand].palm_x = cur->palm_x;
    hands[cur->hand].palm_y = cur->palm_y;
  }

  presenter_update(pres, hands, now_ms, act);

#if USE_USB_HID_GAMEPAD
  /* While the chooser or the menu is up the board must stay silent, or a tap
   * on a row would also nudge the host's pointer. */
  if (!ui_owns)
    presenter_send_usb(act);
#else
  (void)ui_owns;
#endif
}
#endif /* APP_HAS_PRESENTER */

/* Fourth stage, HMI modes: hands -> sticks. Decides which hand is which, then
 * feeds the palm positions into the virtual transmitter. */
_Static_assert(PD_MAX_HAND_NB == 2 && HMI_HAND_NB == 2, "hmi_run() pairs exactly two slots with two sticks");

/* Drone step: assign tracked hands to sticks and update the control law. */
static void hmi_run(track_t tracks[PD_MAX_HAND_NB], hmi_state_t *hmi)
{
  int used[HMI_HAND_NB] = {0, 0};
  int t;

  for (t = 0; t < PD_MAX_HAND_NB; t++) {
    track_t *cur = &tracks[t];
    track_t *other = &tracks[PD_MAX_HAND_NB - 1 - t];
    float other_x = other->is_tracking ? other->palm_x : -1.0f;
    int stick;

    cur->stick = -1;
    if (!cur->is_tracking)
      continue;

    stick = hmi_assign_stick(cur->palm_x, other_x);
    if (used[stick]) {
      /* Both hands in the same half with one lost mid-frame: the second one
       * takes the remaining stick rather than being ignored. */
      stick = 1 - stick;
      if (used[stick])
        continue;
    }
    used[stick] = 1;
    cur->stick = stick;
    hmi_update_stick(hmi, stick, cur->palm_x, cur->palm_y, cur->gesture_cls);
  }

  for (t = 0; t < HMI_HAND_NB; t++) {
    if (!used[t])
      hmi_clear_stick(hmi, t);
  }
  hmi->seq++;
  /* The USB report is NOT sent here: the calibration wizard overrides these
   * axes after hmi_run() returns, and the report has to carry what the wizard
   * produced or the simulator never sees the sweep. The pipeline task sends it
   * once both stages have had their say. */
}

/* Pipeline task. Per frame: palm detection when a slot is free, landmarks per hand,
 * gestures, then the active scheme. */
static void nn_thread_fct(void *arg)
{
  float nn_period_filtered_ms = 0;
  float pd_filtered_ms = 0;
  float ld_filtered_ms = 0;
  hl_model_info_t hl_info;
  pd_model_info_t pd_info;
  uint32_t nn_period_ms;
  uint32_t nn_period[2];
  uint8_t *nn_pipe_dst;
  track_t tracks[PD_MAX_HAND_NB];
  hmi_state_t hmi;
#if APP_HAS_PRESENTER
  presenter_t pres;
  presenter_action_t pres_act;
#endif
  uint32_t frame_idx = 0;
  uint32_t pd_ms;
  uint32_t hl_ms;
  uint32_t gesture_us;
  ui_view_t ui_view;
  int ui_owns = 1;
  int cam_suspended = 0;
  int ret;
  int i, j;

  /* setup models buffer info */
#if APP_HAS_NN
  palm_detector_init(&pd_info);
  hand_landmark_init(&hl_info);
#endif
  for (i = 0; i < PD_MAX_HAND_NB; i++)
    track_init(&tracks[i]);
  hmi_init(&hmi, HMI_MODE_ASSISTED);
#if APP_HAS_PRESENTER
  presenter_init(&pres);
  memset(&pres_act, 0, sizeof(pres_act));
#endif
  ui_get_view(&ui_view);        /* the mode the chooser starts on */

#if HAS_ROTATION_SUPPORT == 1 && APP_HAS_NN
  app_rot_init(&hl_info);
#endif

  /*** App Loop ***************************************************************/
  nn_period[1] = HAL_GetTick();
#if APP_HAS_CAM
  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
#else
  (void)nn_pipe_dst;
#endif
  while (1)
  {
    uint8_t *capture_buffer;
    int idx_for_resize;
    int tracking_nb = 0;
    int run_pd;

    /* ---- power saving --------------------------------------------------
     * Nothing below may block on the camera while asleep: the pipes are
     * suspended so no frame will ever arrive, and the display task only runs
     * when this one signals it -- which is also the only thing that polls the
     * touchscreen. Blocking here would make the sleep unwakeable. */
    if (ui_is_asleep()) {
      if (!cam_suspended) {
        CMW_CAMERA_Suspend(DCMIPP_PIPE1);
        CMW_CAMERA_Suspend(DCMIPP_PIPE2);
        /* Otherwise the last camera frame sits frozen on screen. */
        memset(lcd_bg_buffer, 0, sizeof(lcd_bg_buffer));
        CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_bg_buffer, sizeof(lcd_bg_buffer)));
        cam_suspended = 1;
      }

      os_delay_ms(UI_SLEEP_POLL_MS);
      ui_apply(&hmi);                 /* parks the axes */
      ui_get_view(&ui_view);

#if USE_USB_HID_GAMEPAD
      /* Keep telling the host the sticks are parked. Going quiet instead
       * would leave it acting on the last report sent before the board
       * dozed off, which for throttle is exactly the wrong thing. */
      if (app_mode_is_hmi(ui_view.mode))
        hmi_send_usb_report(&hmi);
#endif

      if (!ui_is_asleep()) {
        CMW_CAMERA_Resume(DCMIPP_PIPE1);
        CMW_CAMERA_Resume(DCMIPP_PIPE2);
        cam_suspended = 0;
      }

      ret = os_mutex_lock(disp.lock);
      assert(ret == OS_OK);
      disp.info.ui = ui_view;
      ret = os_mutex_unlock(disp.lock);
      assert(ret == OS_OK);
      os_sem_give(disp.update);     /* keeps the display task polling touch */

      continue;
    }

    nn_period[0] = nn_period[1];
    nn_period[1] = HAL_GetTick();
    nn_period_ms = nn_period[1] - nn_period[0];
    nn_period_filtered_ms = USE_FILTERED_TS ? (15 * nn_period_filtered_ms + nn_period_ms) / 16 : nn_period_ms;

#if APP_HAS_CAM
    /* Blocks until the camera delivers the next NN frame: this is what paces
     * the whole pipeline to the camera frame rate. */
    capture_buffer = bqueue_get_ready(&nn_input_queue);
    assert(capture_buffer);
    idx_for_resize = frame_event_nb_for_resize % DISPLAY_BUFFER_NB;
#else
    /* Nothing signals this loop without a camera. Tick at roughly the frame
     * rate instead, which is all the GUI needs to stay responsive. */
    os_delay_ms(33);
    capture_buffer = NULL;
    idx_for_resize = 0;
#endif

    /* Slots that survived last frame move on to the ROI they predicted. */
    for (i = 0; i < PD_MAX_HAND_NB; i++) {
      if (!tracks[i].is_tracking)
        continue;
      tracks[i].roi = tracks[i].roi_next;
      copy_pd_box(&tracks[i].box, &tracks[i].box_next);
      tracking_nb++;
    }

    /* Palm detector: every frame while nothing is tracked, every Nth frame
     * while one slot is free, never while both are busy. */
    run_pd = (tracking_nb == 0) ||
             (tracking_nb < PD_MAX_HAND_NB && (frame_idx % PD_REDETECT_PERIOD) == 0);
#if APP_HAS_NN
    if (run_pd) {
      int cand_nb = palm_detector_run(capture_buffer, &pd_info, &pd_ms);

      track_seed_from_pd(tracks, &pd_info, cand_nb);
    } else {
      pd_ms = 0;
    }
#else
    (void)run_pd;
    (void)capture_buffer;
    pd_ms = 0;
#endif
    pd_filtered_ms = USE_FILTERED_TS ? (7 * pd_filtered_ms + pd_ms) / 8 : pd_ms;
#if APP_HAS_CAM
    bqueue_put_free(&nn_input_queue);
#endif

    /* Hand landmarks, once per tracked hand. The NPU runs them back to back;
     * the display buffer is shared read-only so no copy is needed. */
    hl_ms = HAL_GetTick();
    gesture_us = 0;
#if APP_HAS_NN
    for (i = 0; i < PD_MAX_HAND_NB; i++) {
      track_t *t = &tracks[i];
      uint32_t us;

      if (!t->is_tracking)
        continue;

      t->is_tracking = hand_landmark_run(lcd_bg_buffer[idx_for_resize], &hl_info, &t->roi, t->lm);
      if (!t->is_tracking) {
        track_drop(t);
        continue;
      }
      compute_next_roi(&t->roi, t->lm, &t->roi_next, &t->box_next);
      for (j = 0; j < LD_LANDMARK_NB; j++)
        decode_ld_landmark(&t->roi, &t->lm[j], &t->lm_screen[j]);
      /* Third stage. Which classifier runs depends on the mode: the HMI needs
       * Open/Close/Pointer, the presenter needs fist/ok out of HaGRID. */
#if APP_HAS_PRESENTER
      if (app_mode_is_hmi(ui_view.mode))
        t->gesture_cls = gesture_run(t->lm_screen, &t->gesture_filter, &t->gesture_prob, &us);
      else
        t->pose = pose_run(t->lm_screen, t, &us);
#else
      t->gesture_cls = gesture_run(t->lm_screen, &t->gesture_filter, &t->gesture_prob, &us);
#endif
      gesture_us += us;
      hmi_palm_center(t->lm_screen, &t->palm_x, &t->palm_y);
    }
    CACHE_OP(SCB_InvalidateDCache_by_Addr(lcd_bg_buffer[idx_for_resize], sizeof(lcd_bg_buffer[idx_for_resize])));
#else
    (void)idx_for_resize;
#endif
    hl_ms = HAL_GetTick() - hl_ms;
    ld_filtered_ms = USE_FILTERED_TS ? (7 * ld_filtered_ms + hl_ms) / 8 : hl_ms;

    /* A hand in frame is what keeps the board awake. */
    {
      int hands_now = 0;

      for (i = 0; i < PD_MAX_HAND_NB; i++)
        hands_now += tracks[i].is_tracking ? 1 : 0;
      ui_note_activity(hands_now);
    }

    /* Fourth stage, then the GUI. ui_apply() returns non-zero while the
     * chooser, the menu or the wizard owns the controls -- in that state the
     * board must send the host nothing except the wizard's synthetic sweep. */
    if (app_mode_is_hmi(ui_view.mode)) {
      hmi_run(tracks, &hmi);
      ui_owns = ui_apply(&hmi);
      ui_get_view(&ui_view);
#if USE_USB_HID_GAMEPAD
      /* After the override, so the calibration sweep reaches the host too,
       * but never while the chooser or the plain menu is up. */
      if (!ui_owns || ui_view.state == UI_CAL_CENTER || ui_view.state == UI_CAL_SWEEP ||
          ui_view.state == UI_CAL_DONE)
        hmi_send_usb_report(&hmi);
#endif
    }
#if APP_HAS_PRESENTER
    else {
      ui_owns = ui_apply(&hmi);
      ui_get_view(&ui_view);
      presenter_run(tracks, &pres, HAL_GetTick(), &pres_act, ui_owns);
      pres.seq++;
    }
#endif
    frame_idx++;

    /* update display stats */
    ret = os_mutex_lock(disp.lock);
    assert(ret == OS_OK);
    disp.info.pd_ms = (int)pd_filtered_ms;
    disp.info.hl_ms = (int)ld_filtered_ms;
    disp.info.nn_period_ms = nn_period_filtered_ms;
    disp.info.pd_hand_nb = PD_MAX_HAND_NB;
    disp.info.pd_max_prob = pd_info.pd_out.box_nb ? pd_info.pd_out.pOutData[0].prob : 0;
    disp.info.gesture_us = gesture_us;
    for (i = 0; i < PD_MAX_HAND_NB; i++) {
      hand_info_t *h = &disp.info.hands[i];
      track_t *t = &tracks[i];

      h->is_valid = t->is_tracking;
      h->gesture_cls = t->gesture_cls;
      h->gesture_prob = t->gesture_prob;
      h->stick = t->stick;
#if APP_HAS_PRESENTER
      h->pose = (int)t->pose;
#endif
      h->pd_hands.pKps = h->pd_kps;
      copy_pd_box(&h->pd_hands, &t->box);
      h->roi = t->roi;
      for (j = 0; j < LD_LANDMARK_NB; j++)
        h->ld_landmarks[j] = t->lm[j];
    }
    disp.info.hmi = hmi;
#if APP_HAS_PRESENTER
    disp.info.pres = pres;
#endif
    disp.info.ui = ui_view;
    disp.info.is_ld_displayed = ui_view.show_landmarks;
    disp.info.is_pd_displayed = ui_view.show_palmbox;
    ret = os_mutex_unlock(disp.lock);
    assert(ret == OS_OK);

    /* It's possible xqueue is empty if display is slow. So don't check error code that may by pdFALSE in that case */
    os_sem_give(disp.update);
  }
}

/* Point the overlay layer at the buffer being drawn. */
static void dp_update_drawing_area()
{
  int ret;

  __disable_irq();
  ret = SCRL_SetAddress_NoReload(lcd_fg_buffer[lcd_fg_buffer_rd_idx], SCRL_LAYER_1);
  assert(ret == HAL_OK);
  __enable_irq();
}

/* SCRL_ReloadLayer() only *arms* the swap: the LTDC copies its shadow
 * registers at the next vertical blanking and clears LxRCR.VBR when it has.
 * Until that happens the layer's address register still holds what we wrote,
 * so if the next iteration called SetAddress_NoReload() first, the pending
 * reload would latch the buffer we had only just started drawing into -- a
 * half-built panel on screen, intermittently, depending on how the display
 * task and the camera's frame rate drifted against each other.
 *
 * Waiting for the swap to complete makes the double buffering real: the
 * buffer being displayed is always the one that was finished. */
#define LTDC_FG_LAYER LTDC_Layer2   /* SCRL_LAYER_1 == LTDC layer index 1 */
#define LTDC_RELOAD_TIMEOUT_MS 50

/* Show the finished overlay, waiting (bounded) for the display to take it. */
static void dp_commit_drawing_area()
{
  uint32_t start;
  int ret;

  __disable_irq();
  ret = SCRL_ReloadLayer(SCRL_LAYER_1);
  assert(ret == HAL_OK);
  __enable_irq();

  /* Bounded: a display that stops blanking must not take the pipeline with
   * it. One frame at the slowest plausible rate is far less than 50 ms. */
  start = HAL_GetTick();
  while ((LTDC_FG_LAYER->RCR & LTDC_LxRCR_VBR) != 0U) {
    if ((HAL_GetTick() - start) > LTDC_RELOAD_TIMEOUT_MS)
      break;
  }

  lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;
}

/* Both buttons now feed the menu instead of toggling one overlay each: the
 * landmark and palm-box toggles moved into it, which is what freed a button up
 * for the calibration wizard. These run on the display task and only bump a
 * counter -- calib_apply() does the work on the pipeline task. */
static void on_next_button_click(void *args)
{
  (void)args;
  ui_event_next();
}

static void on_select_button_click(void *args)
{
  (void)args;
  ui_event_select();
}

/* The touchscreen is the primary way into the GUI; the two buttons stay as a
 * fallback for when a hand is in front of the panel. Polled here rather than
 * on its interrupt because the I2C read must not run from an ISR. */
/* Set only when BSP_TS_Init() succeeded. Polling a controller that failed to
 * initialise blocks forever inside the I2C read -- that took the display task
 * down, and the pipeline with it, leaving a black screen that looked for all
 * the world like a camera fault. Nothing below may run unless the device
 * actually answered. */
static int touch_ok;

/* A controller that stops answering mid-run must not be able to do the same,
 * so repeated read failures retire the touchscreen instead of being retried
 * thirty times a second. */
#define TOUCH_MAX_FAILURES 10
static int touch_failures;

/* Start the touchscreen; if it fails, the buttons still work. */
static void touch_init(void)
{
  TS_Init_t init;
  int ret;

  init.Width = LCD_BG_WIDTH;
  init.Height = LCD_BG_HEIGHT;
  init.Orientation = TS_SWAP_NONE;
  init.Accuracy = 5;
  ret = BSP_TS_Init(0, &init);
  touch_ok = (ret == BSP_ERROR_NONE);
  if (!touch_ok)
    printf("touchscreen init failed (%d), buttons still work\n", ret);
}

/* Poll the touchscreen and feed the UI; give up after repeated errors. */
static void touch_process(void)
{
  TS_State_t st = { 0 };

  if (!touch_ok)
    return;

  if (BSP_TS_GetState(0, &st) != BSP_ERROR_NONE) {
    if (++touch_failures >= TOUCH_MAX_FAILURES) {
      touch_ok = 0;
      printf("touchscreen stopped responding, buttons still work\n");
    }

    return;
  }
  touch_failures = 0;

  ui_touch_sample((int)st.TouchDetected, (int)st.TouchX, (int)st.TouchY);
}

/* Display task, lowest priority: buttons, touch, UI, overlays and serial output. */
static void dp_thread_fct(void *arg)
{
  button_t next_button;
  button_t select_button;
  uint32_t disp_ms = 0;
  uint32_t last_hmi_seq = 0;
  display_info_t info;
  uint32_t ts;
  int ret;
  int i;

#ifdef STM32N6570_DK_REV
  button_init(&next_button, BUTTON_USER1, on_next_button_click, NULL);
  button_init(&select_button, BUTTON_TAMP, on_select_button_click, NULL);
#else
  button_init(&next_button, BUTTON_USER, on_next_button_click, NULL);
  button_init(&select_button, BUTTON_USER, on_select_button_click, NULL);
#endif
  while (1)
  {
    ret = os_sem_take(disp.update, OS_BLOCKING);
    assert(ret == OS_OK);

    button_process(&next_button);
    button_process(&select_button);
    touch_process();

    ret = os_mutex_lock(disp.lock);
    assert(ret == OS_OK);
    info = disp.info;
    ret = os_mutex_unlock(disp.lock);
    assert(ret == OS_OK);
    info.disp_ms = disp_ms;
    for (i = 0; i < PD_MAX_HAND_NB; i++)   /* struct copy kept the old pointers */
      info.hands[i].pd_hands.pKps = info.hands[i].pd_kps;

    ts = HAL_GetTick();
    dp_update_drawing_area();
    Display_NetworkOutput(&info);
    SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_WIDTH * LCD_FG_HEIGHT* 2);
    dp_commit_drawing_area();
    disp_ms = HAL_GetTick() - ts;

    /* Ship the sticks to the host. Done here, in the lowest-priority task,
     * so the blocking UART write (~4 ms at 115200) never delays the NPU
     * pipeline. One frame per pipeline iteration, dropped when the display
     * falls behind -- a stale stick position is worse than a missing one. */
    if (info.hmi.seq != last_hmi_seq) {
      char frame[64];
      int n = hmi_format_frame(&info.hmi, frame, sizeof(frame));

      if (n > 0)
        _write(1, frame, n);
      last_hmi_seq = info.hmi.seq;
    }
  }
}

/* Run the camera ISP (exposure, white balance) once per vsync. */
static void isp_thread_fct(void *arg)
{
  int ret;

  while (1) {
    ret = os_sem_take(isp_sem, OS_BLOCKING);
    assert(ret == OS_OK);

    CAM_IspUpdate();
  }
}

/* Two LCD layers: the camera image below, the overlay above. */
static void Display_init()
{
  SCRL_LayerConfig layers_config[2] = {
    {
      .origin = {lcd_bg_area.X0, lcd_bg_area.Y0},
      .size = {lcd_bg_area.XSize, lcd_bg_area.YSize},
#if HAS_ROTATION_SUPPORT == 0
      .format = SCRL_RGB888,
#else
      .format = SCRL_ARGB8888,
#endif
      .address = lcd_bg_buffer[lcd_bg_buffer_disp_idx],
    },
    {
      .origin = {lcd_fg_area.X0, lcd_fg_area.Y0},
      .size = {lcd_fg_area.XSize, lcd_fg_area.YSize},
      .format = SCRL_ARGB4444,
      .address = lcd_fg_buffer[1],
    },
  };
  SCRL_ScreenConfig screen_config = {
    .size = {lcd_bg_area.XSize, lcd_bg_area.YSize},
#ifdef SCR_LIB_USE_SPI
    .format = SCRL_RGB565,
#else
     .format = SCRL_YUV422, /* Use SCRL_RGB565 if host support this format to reduce cpu load */
#endif
    .address = screen_buffer,
    .fps = CAMERA_FPS,
  };
  int ret;

  ret = SCRL_Init((SCRL_LayerConfig *[2]){&layers_config[0], &layers_config[1]}, &screen_config);
  assert(ret == 0);

  UTIL_LCD_SetLayer(SCRL_LAYER_1);
  UTIL_LCD_Clear(UTIL_LCD_COLOR_TRANSPARENT);
  UTIL_LCD_SetFont(&LCD_FONT);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
}

/* Set up display, camera, queues and USB, then start the nn, dp and isp tasks. */
void app_run()
{
  PRI isp_priority = APP_PRIORITY(2);
  PRI dp_priority = APP_PRIORITY(-2);
  PRI nn_priority = APP_PRIORITY(1);
  int ret;

  printf("Init application (mode: %s, APP_DIAG=%d)\n",
         app_mode_name(APP_DEFAULT_MODE), APP_DIAG);
  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* screen init */
  memset(lcd_bg_buffer, 0, sizeof(lcd_bg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_bg_buffer, sizeof(lcd_bg_buffer)));
  memset(lcd_fg_buffer, 0, sizeof(lcd_fg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_fg_buffer, sizeof(lcd_fg_buffer)));
  /* Before CAM_Init(): the GT911 comes up cleanly on its own but fails once
   * the camera has been brought up. Both sit on VddIO4 and the camera drives
   * GPIOC8 / GPIOD2 around it, so ordering is the cheapest thing to try --
   * and nothing needs the touchscreen earlier than this anyway. */
  touch_init();
  Display_init();

  /* create buffer queues */
  ret = bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
  assert(ret == 0);

  cpuload_init(&cpu_load);
  ui_init();

  /*** Camera Init ************************************************************/  
#if APP_HAS_CAM
  CAM_Init();
#endif

  /* sems + mutex init */
  ret = os_sem_create(&isp_sem, 0, 1);
  assert(ret == OS_OK);
  ret = os_sem_create(&disp.update, 0, 1);
  assert(ret == OS_OK);
  ret = os_mutex_create(&disp.lock);
  assert(ret == OS_OK);

  /* Start LCD Display camera pipe stream */
#if APP_HAS_CAM
  CAM_DisplayPipe_Start(lcd_bg_buffer[0], CMW_MODE_CONTINUOUS);
#endif

#if USE_USB_HID_GAMEPAD
  /* USB gamepad on CN18. Failure here is not fatal: the serial frames still
   * go out, so the bridge script keeps working without USB. */
  ret = usb_hid_init();
  printf("usb hid gamepad: %s\n", ret ? "FAILED" : "ready");
#endif

  /* threads init */
  ret = os_task_create(&nn_thread, nn_thread_fct, "nn", nn_thread_stack, sizeof(nn_thread_stack),
                       NULL, nn_priority);
  assert(ret == OS_OK);
  ret = os_task_create(&dp_thread, dp_thread_fct, "dp", dp_thread_stack, sizeof(dp_thread_stack),
                       NULL, dp_priority);
  assert(ret == OS_OK);
#if APP_HAS_CAM
  ret = os_task_create(&isp_thread, isp_thread_fct, "isp", isp_thread_stack, sizeof(isp_thread_stack),
                       NULL, isp_priority);
  assert(ret == OS_OK);
#else
  (void)isp_priority;
#endif
}

/* Camera middleware callback (ISR): route frame-done events by pipe. */
int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_frame_event();
  else if (pipe == DCMIPP_PIPE2)
    app_ancillary_pipe_frame_event();

  return HAL_OK;
}

/* Camera middleware callback (ISR): route vsync events. */
int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_vsync_event();

  return HAL_OK;
}
