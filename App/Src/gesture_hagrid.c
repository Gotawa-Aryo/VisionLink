/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    gesture_hagrid.c
  * @brief   The VisionLink HaGRID landmark gesture classifier. See the header.
  *
  * Every step here mirrors one step of the Python that trained it, in the same
  * order and with the same constants:
  *
  *   landmark_features.wrist_relative_scaled()  -> gesture_hagrid_features()
  *   StandardScaler.transform()                 -> standardise()
  *   Dense(64, relu)                            -> dense(..., 1)
  *   Dense(32, relu)                            -> dense(..., 1)
  *   Dense(19, softmax)                         -> dense(..., 0) + softmax()
  *
  * Test/gesture_hagrid_test.c checks that mirror holds, against probabilities
  * the Python reference produced for the same landmark sets. Preprocessing
  * mismatch is the classic way a model that scored well in training turns into
  * noise on the device, and it fails silently -- nothing errors, the numbers
  * are just wrong. That test is the guard.
  ******************************************************************************
  */

#include "gesture_hagrid.h"

#include <math.h>
#include <stddef.h>

#define GH_WRIST      0
#define GH_MIDDLE_MCP 9

/* out[n_out] = relu?(W[n_out][n_in] . in[n_in] + b[n_out]) */
static void dense(const float *w, const float *b, const float *in, float *out,
                  int n_in, int n_out, int relu)
{
  int i, j;

  for (i = 0; i < n_out; i++) {
    const float *row = w + (size_t)i * n_in;
    float acc = b[i];

    for (j = 0; j < n_in; j++)
      acc += row[j] * in[j];

    out[i] = (relu && acc < 0.0f) ? 0.0f : acc;
  }
}

/* In-place softmax, max-shifted so expf() cannot overflow. */
static void softmax(float *v, int n)
{
  float max_v = v[0];
  float sum = 0.0f;
  int i;

  for (i = 1; i < n; i++)
    if (v[i] > max_v)
      max_v = v[i];

  for (i = 0; i < n; i++) {
    v[i] = expf(v[i] - max_v);
    sum += v[i];
  }

  for (i = 0; i < n; i++)
    v[i] /= sum;
}

/* 60 features: landmarks 1 to 20 relative to the wrist, divided by palm size. */
void gesture_hagrid_features(const ld_point_t lm[LD_LANDMARK_NB], float f[GH_IN_DIM])
{
  float wx = lm[GH_WRIST].x;
  float wy = lm[GH_WRIST].y;
  float wz = lm[GH_WRIST].z;
  float dx = lm[GH_MIDDLE_MCP].x - wx;
  float dy = lm[GH_MIDDLE_MCP].y - wy;
  float palm;
  int i;

  /* Palm size from x and y only. z is a relative depth, so folding it in
   * would make the scale reference change as the hand tilts towards the
   * camera -- exactly the variation the normalisation is meant to remove. */
  palm = sqrtf(dx * dx + dy * dy);
  if (palm < GH_MIN_PALM)
    palm = GH_MIN_PALM;

  /* Point 0 is dropped: after the translation it is always (0,0,0) and
   * carries nothing. 20 points x 3 coordinates = 60 features. */
  for (i = 1; i < LD_LANDMARK_NB; i++) {
    f[(i - 1) * 3 + 0] = (lm[i].x - wx) / palm;
    f[(i - 1) * 3 + 1] = (lm[i].y - wy) / palm;
    f[(i - 1) * 3 + 2] = (lm[i].z - wz) / palm;
  }
}

/* The scaler is applied as its own visible step rather than folded into the
 * first layer's weights. Folding would save 60 subtracts and 60 divides per
 * inference, which is nothing, and would cost the one-to-one correspondence
 * with scaler.json that makes this file checkable against the Python. */
static void standardise(float f[GH_IN_DIM])
{
  int i;

  for (i = 0; i < GH_IN_DIM; i++)
    f[i] = (f[i] - gh_scaler_mean[i]) / gh_scaler_scale[i];
}

/* Classify one hand into the 19 HaGRID classes. */
int gesture_hagrid_classify(const ld_point_t lm[LD_LANDMARK_NB], gh_result_t *out)
{
  float in[GH_IN_DIM];
  float h1[GH_H1_DIM];
  float h2[GH_H2_DIM];
  float probs[GH_CLASS_NB];
  int best = 0;
  int i;

  out->cls = GH_NONE;
  out->prob = 0.0f;

  gesture_hagrid_features(lm, in);
  standardise(in);

  dense(gh_w0, gh_b0, in, h1, GH_IN_DIM, GH_H1_DIM, 1);
  dense(gh_w1, gh_b1, h1, h2, GH_H1_DIM, GH_H2_DIM, 1);
  dense(gh_w2, gh_b2, h2, probs, GH_H2_DIM, GH_CLASS_NB, 0);
  softmax(probs, GH_CLASS_NB);

  for (i = 1; i < GH_CLASS_NB; i++)
    if (probs[i] > probs[best])
      best = i;

  if (probs[best] < GH_PROB_THRESHOLD)
    return 0;

  out->cls = best;
  out->prob = probs[best];

  return 1;
}

const char *gesture_hagrid_label(int cls)
{
  if (cls < 0 || cls >= GH_CLASS_NB)
    return "-";

  return gh_labels[cls];
}
