/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    presenter.c
  * @brief   Hand poses in, presenter-remote actions out. See presenter.h.
  ******************************************************************************
  */

#include "presenter.h"

#include <math.h>
#include <string.h>

/* Which half of the SCREEN a given side of the user appears in. */
static int hand_screen_side_is_left(int hand)
{
#if PRES_CAMERA_IS_MIRRORED
  return hand == PRES_HAND_LEFT;
#else
  return hand == PRES_HAND_RIGHT;
#endif
}

/* Left or right hand, from the palm's position and the other hand's. */
int presenter_assign_hand(float palm_x, float other_x)
{
  int is_screen_left;

  if (other_x >= 0)
    is_screen_left = palm_x < other_x;
  else
    is_screen_left = palm_x < LCD_BG_WIDTH * 0.5f;

  if (hand_screen_side_is_left(PRES_HAND_LEFT))
    return is_screen_left ? PRES_HAND_LEFT : PRES_HAND_RIGHT;
  else
    return is_screen_left ? PRES_HAND_RIGHT : PRES_HAND_LEFT;
}

/* Palm centre, the point the pointer and swipes follow. */
void presenter_palm_center(const ld_point_t lm[LD_LANDMARK_NB], float *cx, float *cy)
{
  /* wrist + index/middle/ring/pinky MCP: the rigid part of the hand. */
  static const int idx[] = { 0, 5, 9, 13, 17 };
  float sx = 0, sy = 0;
  unsigned i;

  for (i = 0; i < sizeof(idx) / sizeof(idx[0]); i++) {
    sx += lm[idx[i]].x;
    sy += lm[idx[i]].y;
  }
  *cx = sx / 5.0f;
  *cy = sy / 5.0f;
}

/* Start with no swipe armed and no key sent. */
void presenter_init(presenter_t *p)
{
  memset(p, 0, sizeof(*p));
  p->last_key = PRES_KEY_NONE;
}

static int clampi(int v, int lo, int hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/* --- pointer -------------------------------------------------------------- */

/* The right fist is a clutch, exactly like lifting a mouse off the desk: the
 * cursor follows the hand while the fist is held and stays put when it opens.
 * So there is no absolute mapping to undo, and the user can re-centre by
 * opening the hand, moving, and closing it again.
 *
 * The palm position is smoothed first and then differenced, rather than
 * smoothing the deltas. Differencing a smoothed signal gives a clean velocity
 * with no steady-state error; smoothing the deltas instead would need a
 * dead band to kill jitter, and a dead band is what makes a pointer feel
 * sticky at low speed.
 *
 * The sub-pixel remainder is carried between frames. Without it a slow drag
 * producing 0.4 px per frame would truncate to zero every frame and the
 * cursor would never move at all -- the most annoying possible failure. */
static void pointer_update(presenter_hand_t *h, const presenter_hand_in_t *in,
                           presenter_action_t *out)
{
  float fdx, fdy;
  float px, py;

  if (!in->is_valid || in->pose != HAND_POSE_FIST) {
    h->has_prev = 0;
    h->resid_x = 0;
    h->resid_y = 0;
    return;
  }

  if (!h->has_prev) {
    /* Fist just closed: take the hand where it is and emit nothing, so the
     * cursor does not jump by however far the hand moved while opening. */
    h->has_prev = 1;
    h->sm_x = in->palm_x;
    h->sm_y = in->palm_y;
    h->resid_x = 0;
    h->resid_y = 0;
    out->move = 1;
    out->dx = 0;
    out->dy = 0;
    return;
  }

  px = h->sm_x;
  py = h->sm_y;
  h->sm_x += PRES_POINTER_ALPHA * (in->palm_x - h->sm_x);
  h->sm_y += PRES_POINTER_ALPHA * (in->palm_y - h->sm_y);

  fdx = (h->sm_x - px) * PRES_POINTER_GAIN + h->resid_x;
  fdy = (h->sm_y - py) * PRES_POINTER_GAIN + h->resid_y;

  out->dx = clampi((int)truncf(fdx), -PRES_POINTER_MAX_STEP, PRES_POINTER_MAX_STEP);
  out->dy = clampi((int)truncf(fdy), -PRES_POINTER_MAX_STEP, PRES_POINTER_MAX_STEP);

  /* Keep only what the clamp did not already deliver, and never more than one
   * pixel of it, so a clamped glitch cannot queue up a long slide afterwards. */
  h->resid_x = fdx - (float)out->dx;
  h->resid_y = fdy - (float)out->dy;
  h->resid_x = fmaxf(-1.0f, fminf(1.0f, h->resid_x));
  h->resid_y = fmaxf(-1.0f, fminf(1.0f, h->resid_y));

  out->move = 1;
}

/* --- swipe ---------------------------------------------------------------- */

/* Screen x and user-facing "right" are the same direction only when the image
 * is mirrored. Unmirrored, the user moving their hand to their own right moves
 * it towards screen left. The swipe direction the user performs is what the
 * specification talks about, so convert here rather than at the call site. */
static float to_user_right(float screen_dx)
{
#if PRES_CAMERA_IS_MIRRORED
  return screen_dx;
#else
  return -screen_dx;
#endif
}

/* Arm a swipe: remember where and when the OK pose began. */
static void swipe_anchor(presenter_hand_t *h, const presenter_hand_in_t *in, uint32_t now_ms)
{
  h->armed = 1;
  h->anchor_x = in->palm_x;
  h->anchor_y = in->palm_y;
  h->anchor_ms = now_ms;
}

/* @retval the key this hand wants to send, or PRES_KEY_NONE. */
static pres_key_t swipe_update(presenter_t *p, presenter_hand_t *h,
                               const presenter_hand_in_t *in, int hand, uint32_t now_ms)
{
  pres_key_t want;
  float travel, dy;

  if (!in->is_valid || in->pose != HAND_POSE_OK) {
    h->armed = 0;
    return PRES_KEY_NONE;
  }

  if (!h->armed) {
    swipe_anchor(h, in, now_ms);
    return PRES_KEY_NONE;
  }

  /* Too slow to be a swipe: start a fresh window from here. This is what
   * stops a hand parked in OK from drifting its way into a slide change. */
  if (now_ms - h->anchor_ms > PRES_SWIPE_WINDOW_MS) {
    swipe_anchor(h, in, now_ms);
    return PRES_KEY_NONE;
  }

  travel = to_user_right(in->palm_x - h->anchor_x);
  dy = in->palm_y - h->anchor_y;

  if (fabsf(travel) < PRES_SWIPE_MIN_PX)
    return PRES_KEY_NONE;
  if (fabsf(travel) < PRES_SWIPE_AXIS_RATIO * fabsf(dy))
    return PRES_KEY_NONE;   /* more vertical than horizontal: not a swipe */

  /* Far enough, fast enough, flat enough. Whether it counts depends on which
   * hand and which way: each hand only answers to its own direction, so a
   * right hand swiping left and a left hand swiping right both do nothing. */
  if (hand == PRES_HAND_RIGHT)
    want = (travel > 0) ? PRES_KEY_NEXT : PRES_KEY_NONE;
  else
    want = (travel < 0) ? PRES_KEY_PREV : PRES_KEY_NONE;

  /* Wrong direction for this hand, or still inside the cooldown: consume the
   * travel either way. Leaving the anchor where it was would let the same
   * movement fire the instant the cooldown expired. */
  if (want == PRES_KEY_NONE || now_ms < p->cooldown_until_ms) {
    swipe_anchor(h, in, now_ms);
    return PRES_KEY_NONE;
  }

  p->cooldown_until_ms = now_ms + PRES_COOLDOWN_MS;
  swipe_anchor(h, in, now_ms);

  return want;
}

/* --- per-frame entry point ------------------------------------------------ */

void presenter_update(presenter_t *p, const presenter_hand_in_t hands[PRES_HAND_NB],
                      uint32_t now_ms, presenter_action_t *out)
{
  pres_key_t key;

  memset(out, 0, sizeof(*out));
  out->key = PRES_KEY_NONE;

  /* Pointer: right hand only. */
  pointer_update(&p->hand[PRES_HAND_RIGHT], &hands[PRES_HAND_RIGHT], out);

  /* A hand cannot be in FIST and OK at once, so the pointer and the swipe
   * never contend for the same hand in the same frame. */
  p->hand[PRES_HAND_LEFT].has_prev = 0;

  /* Swipes: right hand first, so that if both hands somehow complete a swipe
   * on the same frame the cooldown lets exactly one through and it is the
   * one on the hand that also drives the pointer. */
  key = swipe_update(p, &p->hand[PRES_HAND_RIGHT], &hands[PRES_HAND_RIGHT],
                     PRES_HAND_RIGHT, now_ms);
  if (key == PRES_KEY_NONE)
    key = swipe_update(p, &p->hand[PRES_HAND_LEFT], &hands[PRES_HAND_LEFT],
                       PRES_HAND_LEFT, now_ms);
  out->key = key;

  /* Latched for the display. */
  p->pointer_active = out->move;
  if (key != PRES_KEY_NONE) {
    p->last_key = key;
    p->last_key_ms = now_ms;
  }
  p->seq++;
}

const char *presenter_key_label(pres_key_t key)
{
  switch (key) {
  case PRES_KEY_NEXT: return "NEXT";
  case PRES_KEY_PREV: return "PREV";
  default:            return "-";
  }
}

/* Milliseconds until another slide change is allowed. */
uint32_t presenter_cooldown_left(const presenter_t *p, uint32_t now_ms)
{
  if (now_ms >= p->cooldown_until_ms)
    return 0;

  return p->cooldown_until_ms - now_ms;
}
