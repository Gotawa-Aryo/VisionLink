/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    hmi.h
  * @brief   Two-hand "virtual transmitter": maps hand position and gesture to
  *          four flight axes and serialises the result.
  *
  * Fourth stage of the pipeline:
  *   palm detection -> hand landmarks -> gesture -> HMI axes
  *
  * Two control laws live in one build and are chosen from the on-screen menu.
  *
  * ACROBATIC -- the original. Palm position *is* the stick: each hand's
  *   displacement from the centre of its half of the screen drives two axes at
  *   once. Direct and fast; hard to fly.
  *
  * ASSISTED (default) -- gesture selects ONE axis per hand, and hand movement
  *   trims that axis relative to where the gesture started (a clutch, not a
  *   stick):
  *
  *     left  hand  closed   throttle  <- vertical movement    (yaw -> mid)
  *     left  hand  pointer  yaw       <- horizontal movement  (throttle holds)
  *     right hand  closed   pitch     <- vertical movement    (roll -> mid)
  *     right hand  pointer  roll      <- horizontal movement  (pitch -> mid)
  *     open, or hand lost:  throttle holds, every other axis returns to mid
  *
  *   Throttle is the only axis that persists: it starts at MINIMUM and is only
  *   ever changed by a deliberate closed-hand movement, which is what stops the
  *   unintended take-off on simulator restart. Because movement is measured
  *   from where the gesture began, releasing and re-gripping ratchets the
  *   throttle further instead of snapping it back.
  *
  * Both laws produce the same output shape, so the USB report and the serial
  * frame are identical between them:
  *     stick[HMI_LEFT ].x = yaw    .y = throttle
  *     stick[HMI_RIGHT].x = roll   .y = pitch
  ******************************************************************************
  */

#ifndef HMI_H
#define HMI_H

#include <stddef.h>
#include <stdint.h>

#include "ld.h"

#define HMI_HAND_NB   2
#define HMI_LEFT      0   /* the user's LEFT hand drives stick 0 */
#define HMI_RIGHT     1

typedef enum {
  HMI_MODE_ACROBATIC = 0,
  HMI_MODE_ASSISTED  = 1,
  HMI_MODE_NB
} hmi_mode_t;

/* Which law the application powers up in. The menu can switch at any time;
 * this only decides where it starts. */
#ifndef HMI_DEFAULT_MODE
#define HMI_DEFAULT_MODE HMI_MODE_ASSISTED
#endif

/* All tunables can be overridden from the Makefile with -D. */

/* 0: the camera image is not mirrored, so the user's left hand appears on
 *    the RIGHT of the screen (a person facing the camera).
 * 1: the image is mirrored (selfie style), left hand appears on the left. */
#ifndef HMI_CAMERA_IS_MIRRORED
#define HMI_CAMERA_IS_MIRRORED 0
#endif

/* ACROBATIC: half-range of the control box around each neutral point, as a
 * fraction of the screen width. Moving the palm this far from neutral gives
 * full stick deflection. 0.18 * 800 px = 144 px of travel in each direction. */
#ifndef HMI_RANGE_FRAC
#define HMI_RANGE_FRAC 0.18f
#endif

/* ASSISTED: hand travel, as a fraction of screen width, that moves the
 * selected axis through its full -1..+1 range. Larger than the acrobatic
 * range on purpose -- lower gain is what makes it flyable. Throttle can be
 * ratcheted with repeated grips, so this need not cover the whole range in
 * one movement. */
#ifndef HMI_ASSIST_RANGE_FRAC
#define HMI_ASSIST_RANGE_FRAC 0.25f
#endif

/* Movement below this fraction of full travel is ignored, so a hand held
 * roughly still does not creep the axis it has selected. */
#ifndef HMI_DEADZONE
#define HMI_DEADZONE 0.10f
#endif

/* Exponential smoothing on the axis output, 0 < alpha <= 1. 1 disables it.
 * Landmark jitter is a few pixels; at 0.5 an axis settles in ~3 frames. */
#ifndef HMI_SMOOTH_ALPHA
#define HMI_SMOOTH_ALPHA 0.5f
#endif

/* Throttle's resting value in assisted mode: the bottom of the range. */
#define HMI_THROTTLE_MIN (-1.0f)

/* Which axis of a hand is under active control this frame (assisted only). */
#define HMI_AXIS_NONE  0
#define HMI_AXIS_VERT  1   /* throttle (left hand) or pitch (right hand) */
#define HMI_AXIS_HORZ  2   /* yaw (left hand) or roll (right hand) */

typedef struct {
  int   is_valid;   /* a hand is on this stick (whatever its gesture) */
  float x;          /* -1 .. +1  left: yaw,      right: roll  */
  float y;          /* -1 .. +1  left: throttle, right: pitch; +y = UP */
  int   gesture;    /* gesture class index, or GESTURE_NONE */
  float palm_x;     /* palm centre, screen pixels -- for drawing only */
  float palm_y;

  /* ASSISTED only, for the on-screen display */
  int   selected;   /* HMI_AXIS_NONE / HMI_AXIS_VERT / HMI_AXIS_HORZ */
  float grab_x;     /* where the current gesture began, screen pixels */
  float grab_y;
} hmi_stick_t;

/* ASSISTED: per-hand clutch state. The axis is driven relative to the palm
 * position at the moment the selecting gesture began. */
typedef struct {
  int   is_grabbed;
  int   grab_gesture;   /* the gesture that opened this grab */
  float origin_x;       /* palm position when it opened */
  float origin_y;
  float origin_val;     /* the selected axis's value at that moment */
} hmi_grab_t;

typedef struct {
  uint32_t    seq;                 /* incremented once per pipeline iteration */
  hmi_mode_t  mode;
  hmi_stick_t stick[HMI_HAND_NB];
  hmi_grab_t  grab[HMI_HAND_NB];   /* ASSISTED only */
} hmi_state_t;

/** @brief Reset into @p mode, axes at rest. */
void hmi_init(hmi_state_t *s, hmi_mode_t mode);

/** @brief Switch control law. Axes are parked at rest, so a mode change can
  *        never hand the simulator a live throttle. */
void hmi_set_mode(hmi_state_t *s, hmi_mode_t mode);

hmi_mode_t hmi_get_mode(const hmi_state_t *s);

/** @brief "assisted" / "acrobatic". */
const char *hmi_mode_name(hmi_mode_t mode);

/**
  * @brief  Park the axes where it is safe to leave them: in assisted mode
  *         throttle at minimum and the rest centred, in acrobatic mode all
  *         centred (the hands drive it absolutely, so it is overwritten at
  *         once anyway). Releases any clutch.
  */
void hmi_rest(hmi_state_t *s);

/**
  * @brief  1 when every axis is sitting at its resting value -- throttle at
  *         minimum in assisted mode, centred in acrobatic, everything else
  *         centred either way.
  *
  * The question this answers is "is the aircraft under power?". Assisted mode
  * holds throttle when the hands let go, by design, so hands-off is a normal
  * flying state and cannot be used to decide whether the board is idle.
  */
int hmi_is_at_rest(const hmi_state_t *s);

/**
  * @brief  Palm centre of a hand from its screen-space landmarks: the mean of
  *         the wrist and the four finger MCP joints. Much steadier than any
  *         single landmark.
  */
void hmi_palm_center(const ld_point_t lm[LD_LANDMARK_NB], float *cx, float *cy);

/**
  * @brief  Decide which stick a hand should drive from where it is on screen.
  * @param  palm_x     palm centre, screen pixels
  * @param  other_x    palm centre of the other tracked hand, or a negative
  *                    value when only one hand is visible
  * @retval HMI_LEFT or HMI_RIGHT
  */
int hmi_assign_stick(float palm_x, float other_x);

/** @brief Centre of a stick's control box, and the drawing reference for it. */
void hmi_neutral_point(int stick, float *nx, float *ny);

/**
  * @brief  Feed one tracked hand's palm centre and gesture into its stick.
  *         ACROBATIC: position maps straight onto both axes.
  *         ASSISTED: the gesture selects an axis and drives it relative to
  *         where the gesture began; a gesture of Open (or anything not
  *         Closed/Pointer) releases, exactly as losing the hand does.
  */
void hmi_update_stick(hmi_state_t *s, int stick, float palm_x, float palm_y, int gesture);

/**
  * @brief  No hand on this stick this frame.
  *         ACROBATIC: recentre both axes.
  *         ASSISTED: release, return the self-centring axes to mid, and leave
  *         throttle frozen where it stands.
  */
void hmi_clear_stick(hmi_state_t *s, int stick);

/** @brief Short label for the axis a hand currently drives ("THR", "YAW", …). */
const char *hmi_axis_label(int stick, int selected);

/**
  * @brief  Serialise the state as one NMEA-style line:
  *
  *           $HMI,<seq>,<Lv>,<Lx>,<Ly>,<Lg>,<Rv>,<Rx>,<Ry>,<Rg>*<cs>\r\n
  *
  *         v  : 1 if a hand is on the stick, else 0
  *         x,y: axis value scaled to -1000..1000 (integers, no float printf)
  *         g  : gesture class index, -1 for none
  *         cs : XOR of every byte between '$' and '*', as two upper-case hex
  *              digits -- exactly the NMEA 0183 checksum.
  * @retval number of bytes written (excluding the NUL), or -1 if len is too small.
  */
int hmi_format_frame(const hmi_state_t *s, char *buf, size_t len);

#endif /* HMI_H */
