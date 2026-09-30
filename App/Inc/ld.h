/**
  ******************************************************************************
  * @file    ld.h
  * @author  MDG Application Team
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef _HAND_LANDMARK_
#define _HAND_LANDMARK_

/* model hard coded values */
#define LD_WIDTH              224.0f
#define LD_HEIGHT             224.0f
#define LD_LANDMARK_NB        21
#define LD_BINDING_NB         21
/* you can tune this */
#define LD_PROB_THRESHOLD     0.25f

typedef struct {
  float x;
  float y;
  /* Relative depth, in the same units as x and y. The model has always
   * produced it -- raw_landmarks[i * 3 + 2] -- and the original post-process
   * simply dropped it, because nothing downstream wanted it. The HaGRID
   * gesture classifier does: twenty of its sixty features are z, and
   * VisionLink measured z's between-class spread (0.599) as on a par with
   * x's (0.604) before deciding to keep it. */
  float z;
} ld_point_t;

extern const int ld_bindings_idx[LD_BINDING_NB][2];

int ld_post_process(float *probs, float *raw_landmarks, ld_point_t landmarks[LD_LANDMARK_NB]);

#endif
