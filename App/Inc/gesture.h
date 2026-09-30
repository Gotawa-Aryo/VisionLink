/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    gesture.h
  * @brief   Hand gesture classification from 21 hand landmarks.
  *
  * Third stage of the pipeline: palm detection -> hand landmarks -> gesture.
  *
  * The classifier is a 1103-parameter MLP taking the 21 landmarks as a
  * wrist-relative, scale-normalised 42-vector. It runs on the Cortex-M55, not
  * on the NPU -- see gesture.c for why.
  ******************************************************************************
  */

#ifndef GESTURE_H
#define GESTURE_H

#include "ld.h"
#include "gesture_model_data.h"

/* All three tunables can be overridden from the Makefile with -D. */

/* Below this softmax probability the frame is reported as GESTURE_NONE. */
#ifndef GESTURE_PROB_THRESHOLD
#define GESTURE_PROB_THRESHOLD  0.60f
#endif

/* Number of past frames the majority filter considers. Set to 1 to disable
 * smoothing and report the raw per-frame argmax. */
#ifndef GESTURE_FILTER_DEPTH
#define GESTURE_FILTER_DEPTH    7
#endif

/* Frames of agreement required before the filtered class changes. Must be
 * <= GESTURE_FILTER_DEPTH. */
#ifndef GESTURE_FILTER_MAJORITY
#define GESTURE_FILTER_MAJORITY 4
#endif

#define GESTURE_NONE (-1)

/* Class indices, in the order of Model/gesture/keypoint_classifier_label.csv.
 * The assisted HMI uses Close and Pointer as axis selectors, so relabelling
 * the model would silently change the controls -- Test/hmi_test.c checks these
 * against gesture_label() to catch that. */
#define GESTURE_OPEN    0
#define GESTURE_CLOSE   1
#define GESTURE_POINTER 2

_Static_assert(GESTURE_CLASS_NB == 3, "gesture class indices assume Open/Close/Pointer");

typedef struct {
  int   cls;   /* class index, or GESTURE_NONE */
  float prob;  /* softmax probability of cls, 0 when GESTURE_NONE */
} gesture_result_t;

typedef struct {
  int history[GESTURE_FILTER_DEPTH];
  int idx;
  int filled;
  int stable_cls;
} gesture_filter_t;

/**
  * @brief  Classify one frame of landmarks.
  * @param  lm  21 landmarks in a consistent, square-pixel coordinate space.
  *             Screen coordinates work; ROI-local coordinates do not, because
  *             the ROI crop is rotation-normalised and the classifier was
  *             trained on image-space landmarks.
  * @param  out receives the class index and its probability.
  * @retval 1 if a class above threshold was found, 0 otherwise.
  */
int gesture_classify(const ld_point_t lm[LD_LANDMARK_NB], gesture_result_t *out);

/** @brief Reset a majority filter to the empty state. */
void gesture_filter_init(gesture_filter_t *f);

/**
  * @brief  Feed one raw result into the majority filter.
  * @retval The stable class index, or GESTURE_NONE while no class holds a
  *         majority. Suppresses the single-frame flicker that raw argmax
  *         produces when a hand is between poses.
  */
int gesture_filter_push(gesture_filter_t *f, const gesture_result_t *raw);

/** @brief Human-readable label, or "-" for GESTURE_NONE / out of range. */
const char *gesture_label(int cls);

#endif /* GESTURE_H */
