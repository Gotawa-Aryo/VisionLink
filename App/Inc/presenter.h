/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    presenter.h
  * @brief   Hand poses in, presenter-remote actions out.
  *
  * Fourth stage of the pipeline:
  *   palm detection -> hand landmarks -> gesture -> presenter
  *
  * The control scheme, and it is the whole specification:
  *
  *   right hand, FIST, then move        -> the pointer moves with the hand
  *   right hand, OK,   then swipe right -> next slide
  *   left  hand, OK,   then swipe left  -> previous slide
  *
  * Slide changes share one cooldown (PRES_COOLDOWN_MS, 1 s) so a two-handed
  * flail cannot walk through the deck.
  *
  * Everything in this file is pure: no HAL, no USB, no display. It takes the
  * two hands and a millisecond timestamp, and returns what to send. That is
  * what makes Test/presenter_test.c possible without a board.
  ******************************************************************************
  */

#ifndef PRESENTER_H
#define PRESENTER_H

#include <stdint.h>

#include "app_config.h"
#include "hand_pose.h"
#include "ld.h"

#define PRES_HAND_NB    2
#define PRES_HAND_LEFT  0   /* the user's LEFT hand  -> previous slide */
#define PRES_HAND_RIGHT 1   /* the user's RIGHT hand -> pointer, next slide */

/* --- tunables, overridable with -D from the Makefile --------------------- */

/* 0: the camera image is not mirrored, so a user facing the camera has their
 *    LEFT hand on the RIGHT of the screen.
 * 1: the image is mirrored, selfie style, left hand on the left.
 * This is the one knob to flip if left and right come out swapped on the
 * board. Same meaning and same default as the parent project's macro. */
#ifndef PRES_CAMERA_IS_MIRRORED
#define PRES_CAMERA_IS_MIRRORED 0
#endif

/* Pointer travel per unit of hand travel. The camera image is 800 px across
 * and a typical presentation screen is 1920, so 2.4 makes a sweep of the
 * image a sweep of the screen. Raise it for a twitchier pointer. */
#ifndef PRES_POINTER_GAIN
#define PRES_POINTER_GAIN 2.4f
#endif

/* Exponential smoothing on the palm position before it is differenced into
 * pointer deltas, 0 < alpha <= 1. Landmark jitter is a couple of pixels per
 * frame, which the gain would otherwise multiply into visible shake. */
#ifndef PRES_POINTER_ALPHA
#define PRES_POINTER_ALPHA 0.45f
#endif

/* Largest pointer step in one report. HID relative axes are signed bytes, so
 * 127 is the hard ceiling; staying under it also stops a tracking glitch from
 * throwing the cursor across the screen in one frame. */
#ifndef PRES_POINTER_MAX_STEP
#define PRES_POINTER_MAX_STEP 100
#endif

/* Hand travel that counts as a swipe, as a fraction of screen width.
 * 0.15 * 800 px = 120 px -- a deliberate movement, not a drift. */
#ifndef PRES_SWIPE_MIN_FRAC
#define PRES_SWIPE_MIN_FRAC 0.15f
#endif

/* The same threshold in pixels. Exposed because the display draws it and the
 * host test measures against it. */
#define PRES_SWIPE_MIN_PX (PRES_SWIPE_MIN_FRAC * LCD_BG_WIDTH)

/* The swipe has to be finished within this long of starting, or the anchor
 * resets. Without it, a hand held in OK and slowly drifting would eventually
 * accumulate enough travel to fire. */
#ifndef PRES_SWIPE_WINDOW_MS
#define PRES_SWIPE_WINDOW_MS 1000
#endif

/* Horizontal travel must exceed vertical travel by this factor, so a diagonal
 * wave or a hand being lowered does not read as a swipe. */
#ifndef PRES_SWIPE_AXIS_RATIO
#define PRES_SWIPE_AXIS_RATIO 1.5f
#endif

/* Dead time after a slide change, shared by both hands. The specification
 * asks for one second. */
#ifndef PRES_COOLDOWN_MS
#define PRES_COOLDOWN_MS 1000
#endif

/* --- interface ------------------------------------------------------------ */

/* What the board decided to send the host this frame. */
typedef enum {
  PRES_KEY_NONE = 0,
  PRES_KEY_NEXT,      /* Page Down */
  PRES_KEY_PREV,      /* Page Up   */
} pres_key_t;

typedef struct {
  int        move;    /* 1 if the pointer should move this frame */
  int        dx;      /* pointer delta, screen pixels, -127..127 */
  int        dy;
  pres_key_t key;     /* a slide change fired this frame, else PRES_KEY_NONE */
} presenter_action_t;

/* One hand as the pipeline sees it, after assignment to a side. */
typedef struct {
  int         is_valid;   /* a hand is on this side right now */
  hand_pose_t pose;
  float       palm_x;     /* palm centre, screen pixels */
  float       palm_y;
} presenter_hand_in_t;

/* Per-hand state the caller does not need to look at. */
typedef struct {
  /* pointer clutch */
  int   has_prev;
  float sm_x, sm_y;       /* smoothed palm position */
  float resid_x, resid_y; /* sub-pixel remainder, so slow drags still move */

  /* swipe */
  int      armed;
  float    anchor_x, anchor_y;
  uint32_t anchor_ms;
} presenter_hand_t;

typedef struct {
  presenter_hand_t hand[PRES_HAND_NB];
  uint32_t         cooldown_until_ms;  /* shared by both hands */
  uint32_t         seq;                /* pipeline iterations, for the display */

  /* last action, latched for the display only */
  pres_key_t last_key;
  uint32_t   last_key_ms;
  int        pointer_active;
} presenter_t;

void presenter_init(presenter_t *p);

/**
  * @brief  Palm centre of a hand: the mean of the wrist and the four finger
  *         MCP joints. Steady when the fingers move, which matters here
  *         because the pointer is driven while the hand is closing into a
  *         fist and opening out of one.
  */
void presenter_palm_center(const ld_point_t lm[LD_LANDMARK_NB], float *cx, float *cy);

/**
  * @brief  Which side of the user a hand on screen belongs to.
  * @param  palm_x   palm centre, screen pixels
  * @param  other_x  palm centre of the other tracked hand, or negative when
  *                  only one hand is visible
  * @retval PRES_HAND_LEFT or PRES_HAND_RIGHT
  *
  * Only consulted when a tracking slot first acquires a hand. Unlike the
  * parent project, which re-decides every frame, the assignment then sticks
  * for as long as the hand is tracked -- see presenter.c for why that matters
  * when the two hands mean different commands.
  */
int presenter_assign_hand(float palm_x, float other_x);

/**
  * @brief  One pipeline iteration.
  * @param  p       state
  * @param  hands   both sides, indexed by PRES_HAND_LEFT / _RIGHT
  * @param  now_ms  a millisecond clock, monotonic
  * @param  out     what to send the host; never NULL
  */
void presenter_update(presenter_t *p, const presenter_hand_in_t hands[PRES_HAND_NB],
                      uint32_t now_ms, presenter_action_t *out);

/** @brief "NEXT", "PREV" or "-". */
const char *presenter_key_label(pres_key_t key);

/**
  * @brief  Remaining cooldown in milliseconds, 0 when ready. Display only.
  */
uint32_t presenter_cooldown_left(const presenter_t *p, uint32_t now_ms);

#endif /* PRESENTER_H */
