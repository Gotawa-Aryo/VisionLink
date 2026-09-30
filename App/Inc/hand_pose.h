/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    hand_pose.h
  * @brief   The two poses this application acts on, and the filter that makes
  *          them stable enough to act on.
  *
  * The classifier (gesture_hagrid.c) knows nineteen HaGRID classes. The
  * presenter knows two. This is the layer in between: it names the poses the
  * control scheme is written in, maps the model's class indices onto them, and
  * holds the temporal filter.
  *
  * Keeping it separate means presenter.c never mentions a HaGRID class index,
  * and swapping the classifier again later -- for the MobileNetV2 image model
  * from the same project, say -- touches this file and nothing else.
  ******************************************************************************
  */

#ifndef HAND_POSE_H
#define HAND_POSE_H

/* The poses the presenter reacts to. Everything else the model can report --
 * palm, peace, rock, no_gesture, a hand halfway between poses -- is
 * HAND_POSE_NONE and does nothing, which is the behaviour you want from a
 * device that is always watching. */
typedef enum {
  HAND_POSE_NONE = 0,
  HAND_POSE_FIST,
  HAND_POSE_OK,
} hand_pose_t;

/**
  * @brief  Map a classifier class index onto a pose.
  * @param  cls  index into gh_labels, or GH_NONE
  * @retval HAND_POSE_FIST for "fist", HAND_POSE_OK for "ok", else
  *         HAND_POSE_NONE.
  */
hand_pose_t hand_pose_from_class(int cls);

/** @brief "FIST", "OK" or "-". */
const char *hand_pose_label(hand_pose_t pose);

/* --- temporal filter ------------------------------------------------------ */

/* OK fires a one-shot action, so a single flickering frame is not a cosmetic
 * problem here the way it is for a label on screen -- it is a slide the
 * presenter did not ask for. FIST gates the pointer, where a dropout mid-drag
 * shows up as the cursor sticking. Both get the same majority filter the
 * parent project used on its own classifier.
 *
 * Five frames at roughly 25 fps is about 200 ms of window and needs three
 * frames to agree, so a pose takes ~120 ms to take hold. That is well under
 * the time it takes to start a swipe, and unnoticeable on a clutch. */
#ifndef HAND_POSE_FILTER_DEPTH
#define HAND_POSE_FILTER_DEPTH 5
#endif

#ifndef HAND_POSE_FILTER_MAJORITY
#define HAND_POSE_FILTER_MAJORITY 3
#endif

typedef struct {
  hand_pose_t history[HAND_POSE_FILTER_DEPTH];
  int         idx;
  int         filled;
  hand_pose_t stable;
} hand_pose_filter_t;

void hand_pose_filter_init(hand_pose_filter_t *f);

/**
  * @brief  Feed one raw pose in, get the stable pose out.
  * @retval The pose holding a majority of the window, or HAND_POSE_NONE while
  *         none does.
  */
hand_pose_t hand_pose_filter_push(hand_pose_filter_t *f, hand_pose_t raw);

#endif /* HAND_POSE_H */
