/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    hand_pose.c
  * @brief   Classifier classes in, presenter poses out. See hand_pose.h.
  ******************************************************************************
  */

#include "hand_pose.h"

#include "gesture_hagrid.h"

/* Reduce a HaGRID class to the two poses the presenter uses. */
hand_pose_t hand_pose_from_class(int cls)
{
  if (cls == GH_CLS_FIST)
    return HAND_POSE_FIST;
  if (cls == GH_CLS_OK)
    return HAND_POSE_OK;

  return HAND_POSE_NONE;
}

const char *hand_pose_label(hand_pose_t pose)
{
  switch (pose) {
  case HAND_POSE_FIST: return "FIST";
  case HAND_POSE_OK:   return "OK";
  default:             return "-";
  }
}

/* --- temporal filter ------------------------------------------------------ */

#define POSE_NB 3   /* NONE, FIST, OK */

void hand_pose_filter_init(hand_pose_filter_t *f)
{
  int i;

  for (i = 0; i < HAND_POSE_FILTER_DEPTH; i++)
    f->history[i] = HAND_POSE_NONE;
  f->idx = 0;
  f->filled = 0;
  f->stable = HAND_POSE_NONE;
}

/* Majority vote over recent frames, so one wrong frame cannot fire an action. */
hand_pose_t hand_pose_filter_push(hand_pose_filter_t *f, hand_pose_t raw)
{
  int counts[POSE_NB] = { 0, 0, 0 };
  int depth;
  int i;

  f->history[f->idx] = raw;
  f->idx = (f->idx + 1) % HAND_POSE_FILTER_DEPTH;
  if (f->filled < HAND_POSE_FILTER_DEPTH)
    f->filled++;
  depth = f->filled;

  for (i = 0; i < depth; i++)
    counts[f->history[i]]++;

  /* A real pose needs a majority to take over. NONE does not: it is the
   * absence of a pose, and the moment nothing else holds a majority the
   * device should stop acting. Requiring a majority of NONE as well would
   * leave the pointer clutch engaged for several frames after the hand
   * opened, which reads as the cursor sticking. */
  for (i = HAND_POSE_FIST; i < POSE_NB; i++) {
    if (counts[i] >= HAND_POSE_FILTER_MAJORITY) {
      f->stable = (hand_pose_t)i;
      return f->stable;
    }
  }

  f->stable = HAND_POSE_NONE;

  return f->stable;
}
