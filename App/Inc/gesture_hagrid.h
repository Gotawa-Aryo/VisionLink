/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    gesture_hagrid.h
  * @brief   The VisionLink HaGRID landmark gesture classifier, 19 classes.
  *
  * Third stage of the pipeline: palm detection -> hand landmarks -> gesture.
  *
  * This replaces the three-class Open/Close/Pointer classifier the parent
  * project inherited from the hand-gesture-recognition-using-mediapipe repo.
  * That model has no OK class, and OK is half of this application's control
  * scheme. This one was trained in the VisionLink project on HaGRID
  * sample-30k, split per user, and has both classes we need -- as its two
  * strongest, in fact:
  *
  *     fist   precision 0.9686  recall 0.9625  F1 0.9655
  *     ok     precision 0.9778  recall 0.9832  F1 0.9805
  *
  * (Overall accuracy is 91.16% across all nineteen classes. That number is
  * held down by call / dislike / like, which are mutually confusable and which
  * this application never uses. The per-class figures above are the ones that
  * govern how well the presenter works.)
  *
  * It is a 6611-parameter MLP over the 60 wrist-relative, palm-scaled landmark
  * features of VisionLink's landmark_features.py, standardised by the scaler
  * saved with the model. Like the classifier it replaces it runs on the
  * Cortex-M55, not the NPU: 6611 multiply-accumulates is microseconds next to
  * two NPU passes costing milliseconds each, and keeping it on the CPU avoids
  * a third ST Edge AI network, a third memory pool and a third weights blob.
  *
  * Weights and the scaler live in the generated gesture_hagrid_model_data.h.
  * Regenerate both, and the host cross-check vectors, with
  * Model/gesture_hagrid/export_weights.py.
  ******************************************************************************
  */

#ifndef GESTURE_HAGRID_H
#define GESTURE_HAGRID_H

#include "gesture_hagrid_model_data.h"
#include "ld.h"

/* Below this softmax probability the frame is reported as GH_NONE. The
 * majority filter in hand_pose.h does the temporal work on top of this, so
 * this threshold only has to reject frames the model itself is unsure about. */
#ifndef GH_PROB_THRESHOLD
#define GH_PROB_THRESHOLD 0.60f
#endif

/* Floor for the palm-size divisor, in whatever unit the landmarks arrive in.
 *
 * VisionLink uses 1.0, in pixels of the 224x224 training crop, after two rows
 * out of 31,095 came back with the wrist and the middle MCP predicted on top
 * of each other and blew their features up to 6.3e7 -- enough to wreck the
 * scaler and stop the MLP learning at all. The firmware sees screen pixels
 * rather than crop pixels, so this is not the same number in the same space;
 * it is the same argument, that a palm spanning less than one pixel is not a
 * detection. Either way it only ever fires on a collapsed skeleton, never on
 * a real hand. */
#ifndef GH_MIN_PALM
#define GH_MIN_PALM 1.0f
#endif

#define GH_NONE (-1)

typedef struct {
  int   cls;   /* class index into gh_labels, or GH_NONE */
  float prob;  /* softmax probability of cls, 0 when GH_NONE */
} gh_result_t;

/**
  * @brief  Turn 21 landmarks into the 60 model features.
  * @param  lm  landmarks carrying x, y and z, in one consistent unit, with
  *             the hand's true orientation in the image. Wrist-relative and
  *             palm-scaled, so the unit itself cancels -- the 224-crop pixels
  *             of the training set and the screen pixels of the firmware give
  *             identical features for the same hand.
  * @param  f   receives 60 features, ordered x1,y1,z1, x2,y2,z2, ... x20,y20,z20
  */
void gesture_hagrid_features(const ld_point_t lm[LD_LANDMARK_NB], float f[GH_IN_DIM]);

/**
  * @brief  Classify one frame of landmarks.
  * @param  lm   as for gesture_hagrid_features()
  * @param  out  receives the class index and its probability
  * @retval 1 if a class above threshold was found, 0 otherwise.
  *
  * The landmarks must carry the hand's real orientation. The model was trained
  * on axis-aligned crops with rotation deliberately left in -- several HaGRID
  * classes differ only by which way the hand points -- so it must be given
  * screen-space landmarks, not the rotation-normalised ROI-local ones the
  * landmark model itself produced.
  */
int gesture_hagrid_classify(const ld_point_t lm[LD_LANDMARK_NB], gh_result_t *out);

/** @brief Human-readable label, or "-" for GH_NONE / out of range. */
const char *gesture_hagrid_label(int cls);

#endif /* GESTURE_HAGRID_H */
