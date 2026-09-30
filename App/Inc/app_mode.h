/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    app_mode.h
  * @brief   Which application the board is being right now.
  *
  * Three control schemes share one binary, one camera pipeline and one USB
  * device. The user picks at power-on and can change it from the menu at any
  * time; nothing about the USB device changes, because its report descriptor
  * declares mouse, keyboard and gamepad in every mode and each scheme simply
  * sends the reports it needs.
  ******************************************************************************
  */

#ifndef APP_MODE_H
#define APP_MODE_H

/* Presenter mode can be left out of a build: it is the only user of the
 * 19-class HaGRID model, so dropping it also drops ~100 KB of weights and the
 * whole second classifier. The enumerator keeps its value either way, so the
 * two drone modes never renumber. */
#ifndef APP_HAS_PRESENTER
#define APP_HAS_PRESENTER 1
#endif

typedef enum {
  APP_MODE_ASSISTED = 0,   /* drone HMI, gesture selects one axis at a time */
  APP_MODE_ACROBATIC,      /* drone HMI, palm position is the stick         */
  APP_MODE_PRESENTER,      /* slide clicker: pointer + Page Up/Down         */
} app_mode_t;

/* How many modes the chooser offers and the menu cycles through. */
#if APP_HAS_PRESENTER
#define APP_MODE_NB 3
#else
#define APP_MODE_NB 2
#endif

/* Where the chooser's highlight starts at power-on. */
#ifndef APP_DEFAULT_MODE
#define APP_DEFAULT_MODE APP_MODE_ASSISTED
#endif

/** @brief "assisted" / "acrobatic" / "presenter". */
const char *app_mode_name(app_mode_t mode);

/** @brief "Drone HMI - assisted", etc: the chooser's wording. */
const char *app_mode_title(app_mode_t mode);

/** @brief One line saying what the mode does, for the chooser. */
const char *app_mode_blurb(app_mode_t mode);

/** @brief 1 for the two drone-HMI modes, 0 for presenter. */
int app_mode_is_hmi(app_mode_t mode);

#endif /* APP_MODE_H */
