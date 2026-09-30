/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    app_mode.c
  * @brief   Names and wording for the three control schemes. See app_mode.h.
  ******************************************************************************
  */

#include "app_mode.h"

/* Short scheme name for the console. */
const char *app_mode_name(app_mode_t mode)
{
  switch (mode) {
  case APP_MODE_ACROBATIC: return "acrobatic";
  case APP_MODE_PRESENTER: return "presenter";
  case APP_MODE_ASSISTED:
  default:                 return "assisted";
  }
}

/* Scheme title shown in the chooser. */
const char *app_mode_title(app_mode_t mode)
{
  switch (mode) {
  case APP_MODE_ACROBATIC: return "Drone HMI  -  acrobatic";
  case APP_MODE_PRESENTER: return "Presenter remote";
  case APP_MODE_ASSISTED:
  default:                 return "Drone HMI  -  assisted";
  }
}

/* One-line description shown under the title. */
const char *app_mode_blurb(app_mode_t mode)
{
  switch (mode) {
  case APP_MODE_ACROBATIC: return "palm position is the stick";
  case APP_MODE_PRESENTER: return "fist points, ok swipes slides";
  case APP_MODE_ASSISTED:
  default:                 return "gesture picks one axis at a time";
  }
}

/* True for the two drone schemes. */
int app_mode_is_hmi(app_mode_t mode)
{
  return mode == APP_MODE_ASSISTED || mode == APP_MODE_ACROBATIC;
}
