/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    gesture.c
  * @brief   Hand gesture classification from 21 hand landmarks.
  *
  * DATA PREPROCESSING. Matches the reference implementation the weights come from:
  *   1. translate so the wrist (landmark 0) is the origin
  *   2. flatten to [x0,y0, x1,y1, ... x20,y20]
  *   3. divide every element by the largest absolute value in the vector
  * The preprocessing makes the result invariant to where the hand is and how big it
  * appears, so the same weights work at any distance from the camera.
  *
  * Reference implementation is keypoint_classifier.tflite from
  *   https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe
  * Which is distributed under the Apache License 2.0.
  ******************************************************************************
  */

#include "gesture.h"

#include <math.h>
#include <string.h>

/* One fully-connected layer: out = W.in + b, with optional ReLU. */
static void dense(const float *w, const float *b, const float *in, float *out, int n_in, int n_out, int relu){
  int i, j;

  for (i = 0; i < n_out; i++) {
    const float *row = w + (size_t)i * n_in;
    float acc = b[i];

    for (j = 0; j < n_in; j++){
      acc += row[j] * in[j];
    }

    out[i] = (relu && acc < 0.0f) ? 0.0f : acc;
  }
}

/* Softmax in place, shifted by the maximum to avoid overflow. */
static void softmax(float *v, int n){
  float max_v = v[0];
  float sum = 0.0f;
  int i;

  for (i = 1; i < n; i++){
    if (v[i] > max_v){
      max_v = v[i];
    }
  }

  for (i = 0; i < n; i++) {
    v[i] = expf(v[i] - max_v);
    sum += v[i];
  }

  for (i = 0; i < n; i++){
    v[i] /= sum;
  }
}

/* Landmarks relative to the wrist, scaled so the largest offset is 1. Returns 0 if
 * degenerate. */
static int gesture_prepare_input(const ld_point_t lm[LD_LANDMARK_NB], float in[GESTURE_IN_DIM]){
  float base_x = lm[0].x;
  float base_y = lm[0].y;
  float max_abs = 0.0f;
  int i;

  for (i = 0; i < LD_LANDMARK_NB; i++) {
    float dx = lm[i].x - base_x;
    float dy = lm[i].y - base_y;

    in[2 * i + 0] = dx;
    in[2 * i + 1] = dy;

    if (fabsf(dx) > max_abs){
      max_abs = fabsf(dx);
    }

    if (fabsf(dy) > max_abs){
      max_abs = fabsf(dy);
    }
  }

  if (max_abs < 1e-6f) return 0;

  for (i = 0; i < GESTURE_IN_DIM; i++){
    in[i] /= max_abs;
  }

  return 1;
}

/* Classify one hand as Open, Close or Pointer; GESTURE_NONE if not confident. */
int gesture_classify(const ld_point_t lm[LD_LANDMARK_NB], gesture_result_t *out){
  float in[GESTURE_IN_DIM];
  float h1[GESTURE_H1_DIM];
  float h2[GESTURE_H2_DIM];
  float logits[GESTURE_CLASS_NB];
  int best = 0;
  int i;

  out->cls = GESTURE_NONE;
  out->prob = 0.0f;

  if (!gesture_prepare_input(lm, in)) return 0;

  dense(gesture_w0, gesture_b0, in, h1, GESTURE_IN_DIM, GESTURE_H1_DIM, 1);
  dense(gesture_w1, gesture_b1, h1, h2, GESTURE_H1_DIM, GESTURE_H2_DIM, 1);
  dense(gesture_w2, gesture_b2, h2, logits, GESTURE_H2_DIM, GESTURE_CLASS_NB, 0);
  softmax(logits, GESTURE_CLASS_NB);

  for (i = 1; i < GESTURE_CLASS_NB; i++){
    if (logits[i] > logits[best]){
      best = i;
    }
  }

  if (logits[best] < GESTURE_PROB_THRESHOLD) return 0;

  out->cls = best;
  out->prob = logits[best];

  return 1;
}

/* Empty the vote history. */
void gesture_filter_init(gesture_filter_t *f){
  int i;

  for (i = 0; i < GESTURE_FILTER_DEPTH; i++){
    f->history[i] = GESTURE_NONE;
  }

  f->idx = 0;
  f->filled = 0;
  f->stable_cls = GESTURE_NONE;
}

/* Add this frame's class and return the majority; with no majority, keep the last
 * class. */
int gesture_filter_push(gesture_filter_t *f, const gesture_result_t *raw){
  int counts[GESTURE_CLASS_NB];
  int depth;
  int i;

  f->history[f->idx] = raw ? raw->cls : GESTURE_NONE;
  f->idx = (f->idx + 1) % GESTURE_FILTER_DEPTH;
  if (f->filled < GESTURE_FILTER_DEPTH){
    f->filled++;
  }
  depth = f->filled;

  memset(counts, 0, sizeof(counts));
  for (i = 0; i < depth; i++){
    if (f->history[i] >= 0 && f->history[i] < GESTURE_CLASS_NB){
      counts[f->history[i]]++;
    }
  }

  for (i = 0; i < GESTURE_CLASS_NB; i++) {
    if (counts[i] >= GESTURE_FILTER_MAJORITY) {
      f->stable_cls = i;
      return f->stable_cls;
    }
  }

  int any = 0;

  for (i = 0; i < GESTURE_CLASS_NB; i++){
    any += counts[i];
  }

  if (any == 0){
    f->stable_cls = GESTURE_NONE;
  }

  return f->stable_cls;
}

const char *gesture_label(int cls){
  if (cls < 0 || cls >= GESTURE_CLASS_NB) return "-";

  return gesture_labels[cls];
}
