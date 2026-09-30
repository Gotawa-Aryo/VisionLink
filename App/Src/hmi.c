/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    hmi.c
  * @brief   Two-hand virtual transmitter, acrobatic and assisted. See hmi.h.
  *
  * Both control laws are compiled in and selected at run time from the menu,
  * so the two halves below are plain functions rather than #if branches.
  ******************************************************************************
  */

#include "hmi.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "gesture.h"

#define HMI_RANGE_PX        (HMI_RANGE_FRAC * LCD_BG_WIDTH)
#define HMI_ASSIST_RANGE_PX (HMI_ASSIST_RANGE_FRAC * LCD_BG_WIDTH)

/* Which half of the SCREEN each stick lives in, given the mirroring. */
static int stick_screen_side_is_left(int stick)
{
#if HMI_CAMERA_IS_MIRRORED
  return stick == HMI_LEFT;
#else
  return stick == HMI_RIGHT;
#endif
}

/* Acrobatic mode's stick centre: the middle of that hand's half of the screen. */
void hmi_neutral_point(int stick, float *nx, float *ny)
{
  *nx = stick_screen_side_is_left(stick) ? LCD_BG_WIDTH * 0.25f : LCD_BG_WIDTH * 0.75f;
  *ny = LCD_BG_HEIGHT * 0.5f;
}

const char *hmi_mode_name(hmi_mode_t mode)
{
  return mode == HMI_MODE_ACROBATIC ? "acrobatic" : "assisted";
}

hmi_mode_t hmi_get_mode(const hmi_state_t *s)
{
  return s->mode;
}

/* Park every axis: centred, nothing selected or grabbed, throttle at its rest value. */
void hmi_rest(hmi_state_t *s)
{
  int i;

  for (i = 0; i < HMI_HAND_NB; i++) {
    s->stick[i].x = 0;
    s->stick[i].y = 0;
    s->stick[i].selected = HMI_AXIS_NONE;
    s->grab[i].is_grabbed = 0;
  }

  /* Assisted mode's throttle rests at the BOTTOM, not the middle. A simulator
   * that reads the axes before any hand appears -- on restart, or while the
   * pipeline is still acquiring -- then sees zero throttle and stays on the
   * ground. Acrobatic mode has no persistent throttle: the hands drive it
   * absolutely and overwrite this on the next frame either way. */
  if (s->mode == HMI_MODE_ASSISTED)
    s->stick[HMI_LEFT].y = HMI_THROTTLE_MIN;
}

/* Generous enough to ignore the smoothing filter's last few thousandths, tight
 * enough that any deliberate stick input counts as live. */
#define HMI_REST_EPSILON 0.02f

/* True when every axis is at rest; power saving waits for this. */
int hmi_is_at_rest(const hmi_state_t *s)
{
  float throttle_rest = (s->mode == HMI_MODE_ASSISTED) ? HMI_THROTTLE_MIN : 0.0f;

  return fabsf(s->stick[HMI_LEFT].y - throttle_rest) < HMI_REST_EPSILON &&
         fabsf(s->stick[HMI_LEFT].x) < HMI_REST_EPSILON &&
         fabsf(s->stick[HMI_RIGHT].x) < HMI_REST_EPSILON &&
         fabsf(s->stick[HMI_RIGHT].y) < HMI_REST_EPSILON;
}

/* Start in the given scheme with all axes parked. */
void hmi_init(hmi_state_t *s, hmi_mode_t mode)
{
  int i;

  memset(s, 0, sizeof(*s));
  s->mode = mode;
  for (i = 0; i < HMI_HAND_NB; i++)
    s->stick[i].gesture = GESTURE_NONE;
  hmi_rest(s);
}

/* Switch drone scheme, parking the axes first. */
void hmi_set_mode(hmi_state_t *s, hmi_mode_t mode)
{
  if (mode >= HMI_MODE_NB || mode == s->mode)
    return;

  s->mode = mode;
  /* Park before handing over. Switching from acrobatic -- where throttle may
   * be anywhere -- straight into assisted would otherwise freeze that value
   * as the new resting throttle, which is exactly the take-off hazard
   * assisted mode exists to remove. */
  hmi_rest(s);
}

/* Palm centre, the point that drives the stick. */
void hmi_palm_center(const ld_point_t lm[LD_LANDMARK_NB], float *cx, float *cy)
{
  /* wrist + index/middle/ring/pinky MCP: the rigid part of the hand, so the
   * centre does not move when the fingers open or close. */
  const int idx[] = {0, 5, 9, 13, 17};
  float sx = 0, sy = 0;
  size_t i;

  for (i = 0; i < sizeof(idx) / sizeof(idx[0]); i++) {
    sx += lm[idx[i]].x;
    sy += lm[idx[i]].y;
  }
  *cx = sx / 5.0f;
  *cy = sy / 5.0f;
}

/* Which stick a hand drives, from its position and the other hand's. */
int hmi_assign_stick(float palm_x, float other_x)
{
  int is_screen_left;

  /* Two hands: the one further left on screen is the screen-left hand.
   * One hand: whichever half of the screen it is in. Hands crossing over
   * each other swaps the sticks -- that is by design, the alternative
   * (sticky identity) is worse when tracking drops and re-acquires. */
  if (other_x >= 0)
    is_screen_left = palm_x < other_x;
  else
    is_screen_left = palm_x < LCD_BG_WIDTH * 0.5f;

  if (stick_screen_side_is_left(HMI_LEFT))
    return is_screen_left ? HMI_LEFT : HMI_RIGHT;
  else
    return is_screen_left ? HMI_RIGHT : HMI_LEFT;
}

/* Short label for the selected axis, such as THR or ROL. */
const char *hmi_axis_label(int stick, int selected)
{
  if (selected == HMI_AXIS_VERT)
    return stick == HMI_LEFT ? "THR" : "PIT";
  if (selected == HMI_AXIS_HORZ)
    return stick == HMI_LEFT ? "YAW" : "ROL";

  return "--";
}

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/* Approach a target by one smoothing step. */
static void ease(float *v, float target)
{
  *v += HMI_SMOOTH_ALPHA * (target - *v);
}

/* --------------------------------------------------------------------------
 * ACROBATIC: palm position is the stick
 * ----------------------------------------------------------------------- */

/* Radial dead zone with re-scaling, so the output is continuous at the edge
 * of the dead zone instead of jumping from 0 to HMI_DEADZONE. */
static void apply_deadzone_2d(float *x, float *y)
{
  float mag = sqrtf(*x * *x + *y * *y);
  float scale;

  if (mag < HMI_DEADZONE) {
    *x = 0;
    *y = 0;
    return;
  }
  scale = (mag - HMI_DEADZONE) / (1.0f - HMI_DEADZONE) / mag;
  *x = clampf(*x * scale, -1.0f, 1.0f);
  *y = clampf(*y * scale, -1.0f, 1.0f);
}

/* Acrobatic law: offset from the neutral point, with dead zone and easing, drives both
 * axes. */
static void acro_update(hmi_state_t *s, int stick, float palm_x, float palm_y, int gesture)
{
  hmi_stick_t *st = &s->stick[stick];
  float nx, ny, x, y;

  hmi_neutral_point(stick, &nx, &ny);

  x = clampf((palm_x - nx) / HMI_RANGE_PX, -1.0f, 1.0f);
  y = clampf((ny - palm_y) / HMI_RANGE_PX, -1.0f, 1.0f);  /* screen y grows downwards */
  apply_deadzone_2d(&x, &y);

  if (!st->is_valid) {
    /* hand just appeared: start from its real position, not from zero */
    st->x = x;
    st->y = y;
  } else {
    ease(&st->x, x);
    ease(&st->y, y);
  }
  st->is_valid = 1;
  st->gesture = gesture;
  st->palm_x = palm_x;
  st->palm_y = palm_y;
  st->selected = HMI_AXIS_NONE;
}

/* Hand gone (acrobatic): the stick springs back to centre. */
static void acro_clear(hmi_state_t *s, int stick)
{
  hmi_stick_t *st = &s->stick[stick];

  st->is_valid = 0;
  st->x = 0;
  st->y = 0;
  st->gesture = GESTURE_NONE;
  st->selected = HMI_AXIS_NONE;
}

/* --------------------------------------------------------------------------
 * ASSISTED: gesture selects one axis, movement trims it
 * ----------------------------------------------------------------------- */

/* One-dimensional dead zone, re-scaled so the axis is continuous at its edge.
 * Applied to the movement SINCE the gesture began, so it stops the axis
 * creeping while a hand is merely held in place. */
static float deadzone_1d(float v)
{
  float mag = fabsf(v);
  float scaled;

  if (mag < HMI_DEADZONE)
    return 0.0f;

  scaled = (mag - HMI_DEADZONE) / (1.0f - HMI_DEADZONE);

  return v < 0 ? -scaled : scaled;
}

/* Which axis a gesture selects, for either hand. */
static int gesture_selects(int gesture)
{
  switch (gesture) {
  case GESTURE_CLOSE:   return HMI_AXIS_VERT;
  case GESTURE_POINTER: return HMI_AXIS_HORZ;
  default:              return HMI_AXIS_NONE;   /* Open, or no class */
  }
}

/* Let go: the axes that spring back do so, and throttle stays put.
 * Rules 2 and 3. The left hand's y IS throttle, so it is never touched here. */
static void assist_release(hmi_state_t *s, int stick)
{
  hmi_stick_t *st = &s->stick[stick];

  s->grab[stick].is_grabbed = 0;
  st->selected = HMI_AXIS_NONE;

  ease(&st->x, 0.0f);                 /* yaw / roll return to mid */
  if (stick == HMI_RIGHT)
    ease(&st->y, 0.0f);               /* pitch returns to mid */
  /* stick[HMI_LEFT].y is throttle: frozen, deliberately not eased */
}

/* Assisted law: the gesture picks one axis; hand travel since the grab trims it. */
static void assist_update(hmi_state_t *s, int stick, float palm_x, float palm_y, int gesture)
{
  hmi_stick_t *st = &s->stick[stick];
  hmi_grab_t *g = &s->grab[stick];
  int selected = gesture_selects(gesture);
  float *axis;
  float travel;
  float target;

  /* A hand is present whatever it is doing, so the display can show it. */
  st->is_valid = 1;
  st->gesture = gesture;
  st->palm_x = palm_x;
  st->palm_y = palm_y;

  if (selected == HMI_AXIS_NONE) {
    /* Open hand: identical to having no hand at all, except that we know it
     * is there. Rules 2 and 3. */
    assist_release(s, stick);

    return;
  }

  /* (Re-)arm the clutch when the grip starts, or when the gesture changes to
   * select a different axis. The reference is where the hand is NOW and what
   * the axis reads NOW, so an axis is never disturbed by gripping. */
  if (!g->is_grabbed || g->grab_gesture != gesture) {
    g->is_grabbed = 1;
    g->grab_gesture = gesture;
    g->origin_x = palm_x;
    g->origin_y = palm_y;
    g->origin_val = (selected == HMI_AXIS_VERT) ? st->y : st->x;
  }

  st->selected = selected;
  st->grab_x = g->origin_x;
  st->grab_y = g->origin_y;

  if (selected == HMI_AXIS_VERT) {
    /* Rules 4 and 6: vertical movement only. Screen y grows downwards, so
     * moving the hand UP is a positive axis change. */
    axis = &st->y;
    travel = (g->origin_y - palm_y) / HMI_ASSIST_RANGE_PX;
  } else {
    /* Rules 5 and 7: horizontal movement only. */
    axis = &st->x;
    travel = (palm_x - g->origin_x) / HMI_ASSIST_RANGE_PX;
  }

  target = clampf(g->origin_val + deadzone_1d(clampf(travel, -1.0f, 1.0f)), -1.0f, 1.0f);
  ease(axis, target);

  /* The axis this hand is NOT driving. Throttle holds; everything else
   * springs back. Rules 4-7's "does nothing" for the other direction. */
  if (selected == HMI_AXIS_VERT)
    ease(&st->x, 0.0f);                       /* yaw / roll to mid */
  else if (stick == HMI_RIGHT)
    ease(&st->y, 0.0f);                       /* pitch to mid */
  /* left hand + pointer: throttle (st->y) holds, untouched */
}

/* Hand gone (assisted): release the grab; throttle keeps its value. */
static void assist_clear(hmi_state_t *s, int stick)
{
  s->stick[stick].is_valid = 0;
  s->stick[stick].gesture = GESTURE_NONE;
  assist_release(s, stick);
}

/* --------------------------------------------------------------------------
 * dispatch
 * ----------------------------------------------------------------------- */

void hmi_update_stick(hmi_state_t *s, int stick, float palm_x, float palm_y, int gesture)
{
  if (s->mode == HMI_MODE_ACROBATIC)
    acro_update(s, stick, palm_x, palm_y, gesture);
  else
    assist_update(s, stick, palm_x, palm_y, gesture);
}

/* A hand has left its stick: apply the active scheme's release rule. */
void hmi_clear_stick(hmi_state_t *s, int stick)
{
  if (s->mode == HMI_MODE_ACROBATIC)
    acro_clear(s, stick);
  else
    assist_clear(s, stick);
}

/* Format the $HMI serial line, with its NMEA-style XOR checksum. */
int hmi_format_frame(const hmi_state_t *s, char *buf, size_t len)
{
  const hmi_stick_t *l = &s->stick[HMI_LEFT];
  const hmi_stick_t *r = &s->stick[HMI_RIGHT];
  uint8_t cs = 0;
  int n, i;

  n = snprintf(buf, len, "$HMI,%lu,%d,%d,%d,%d,%d,%d,%d,%d*",
               (unsigned long)s->seq,
               l->is_valid, (int)lroundf(l->x * 1000), (int)lroundf(l->y * 1000), l->gesture,
               r->is_valid, (int)lroundf(r->x * 1000), (int)lroundf(r->y * 1000), r->gesture);
  if (n < 0 || (size_t)n + 5 > len)   /* "XX\r\n" + NUL */
    return -1;

  for (i = 1; i < n - 1; i++)         /* between '$' and '*' */
    cs ^= (uint8_t)buf[i];

  n += snprintf(buf + n, len - n, "%02X\r\n", cs);

  return n;
}
