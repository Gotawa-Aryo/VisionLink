/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    ui.h
  * @brief   On-screen GUI: the power-on mode chooser, mode switching, display
  *          toggles, and the simulator calibration wizard. Driven by the
  *          touchscreen, with the two board buttons as a fallback.
  *
  * The board boots into the chooser and sends the host nothing until a scheme
  * is picked, so a drone simulator or a slide deck already open on the PC
  * cannot be driven by a hand that happened to be in frame at power-on.
  *
  * Why the calibration wizard exists: assisted mode makes the simulator's
  * "centre the sticks, now move them through their full range" step impossible
  * by hand, because only one axis per hand is live at a time and throttle has
  * to be ratcheted. The wizard synthesises the sweep instead.
  *
  * Input:
  *   touch   tap a row or a button
  *   USER1   move the highlight; "next" inside the wizard
  *   TAMP    activate the highlighted row; open the menu; back out
  *
  * Threading: touches and buttons are collected by the display task and only
  * queued. Every state transition happens in the pipeline task inside
  * ui_apply(), so the state machine lives in one thread and needs no lock.
  ******************************************************************************
  */

#ifndef UI_H
#define UI_H

#include <stdint.h>

#include "app_mode.h"
#include "hmi.h"

/* Idle time before the board stops the camera and blanks the screen. */
#ifndef UI_SLEEP_MS
#define UI_SLEEP_MS 60000u
#endif

/* How often the pipeline task wakes while asleep. It is the only thing that
 * signals the display task, which is in turn the only thing that polls the
 * touchscreen -- so this is also the wake-up latency. 50 ms is imperceptible
 * and still leaves the CPU idle for ~98% of the time. */
#ifndef UI_SLEEP_POLL_MS
#define UI_SLEEP_POLL_MS 50u
#endif

/* How long the full-range sweep takes. The simulator asked for 3 seconds. */
#ifndef UI_SWEEP_MS
#define UI_SWEEP_MS 3000u
#endif

typedef enum {
  UI_STARTUP = 0,  /* power-on chooser; nothing is sent to the host yet */
  UI_HIDDEN,       /* running: hands drive it, only the open tab is drawn */
  UI_MENU,         /* GUI open, axes parked at rest */
  UI_CAL_CENTER,   /* wizard 1/2: every axis forced to exact centre */
  UI_CAL_SWEEP,    /* wizard 2/2: every axis sweeping its full range */
  UI_CAL_DONE,     /* wizard finished, axes centred, waiting to hand back */
} ui_state_t;

typedef enum {
  UI_ITEM_CALIBRATE = 0,
  UI_ITEM_MODE,
  UI_ITEM_LANDMARKS,
  UI_ITEM_PALMBOX,
  UI_ITEM_CLOSE,
  UI_ITEM_NB
} ui_item_t;

/** @brief Snapshot handed to the display task each frame. */
typedef struct {
  ui_state_t state;
  int        cursor;          /* highlighted row, or chooser entry */
  app_mode_t mode;
  int        show_landmarks;
  int        show_palmbox;
  int        sweep_pct;       /* 0..100 progress through the sweep */
  int        asleep;          /* power saving: camera stopped, screen blanked */
} ui_view_t;

void ui_init(void);

/** @brief USER1: move the highlight, or advance the wizard. */
void ui_event_next(void);

/** @brief TAMP: activate, open the menu, or back out. */
void ui_event_select(void);

/** @brief A finger went down at screen pixel (@p x, @p y). Press edge only. */
void ui_event_touch(int x, int y);

/**
  * @brief  Feed one raw touchscreen sample; calls ui_event_touch() on the
  *         press edge only. @p pressed is the controller's touch-detected flag.
  */
void ui_touch_sample(int pressed, int x, int y);

/**
  * @brief  Advance the state machine and, while the GUI owns the controls,
  *         park or override the HMI axes in @p s.
  *
  *         Call once per pipeline iteration, after the control scheme has
  *         computed its output and before anything is sent to the host.
  * @retval 1 if the GUI owns the controls, so the caller must send the host
  *         nothing this frame; 0 if the hands are in control.
  */
int ui_apply(hmi_state_t *s);

/**
  * @brief  Tell the idle timer whether a hand is in frame. Call once per
  *         pipeline iteration; any hand, any open panel, or any touch or
  *         button press counts as activity and restarts the clock.
  */
void ui_note_activity(int hands_tracked);

/**
  * @brief  1 while the board is in power saving: the caller must suspend the
  *         camera pipes and skip inference, but must NOT stop servicing the
  *         GUI -- touch is the only way back out.
  */
int ui_is_asleep(void);

/** @brief Snapshot for the display task. */
void ui_get_view(ui_view_t *out);

/** @brief Draw the GUI. Call from the display task, after the flying HUD. */
void ui_draw(const ui_view_t *v);

/**
  * @brief  Nudge a label clear of the GUI's own chrome.
  *
  * The MENU tab sits in the top-left corner and is drawn last, so anything the
  * pipeline puts there -- a hand's pose label, say -- ends up underneath it.
  * Labels follow the hand and the tab does not move, so the label is what
  * gives way: if the box would overlap the tab it is pushed below it.
  *
  * @param  x,y  top-left of the text, updated in place
  * @param  w,h  size of the text box
  */
void ui_avoid_chrome(int *x, int *y, int w, int h);

/**
  * @brief  Draw one line of text at a pixel position, guaranteed to land on
  *         screen.
  *
  * UTIL_LCD bounds-checks nothing: DrawChar() emits Width columns and Height
  * rows straight into a linear frame buffer, so text that runs off the right
  * edge reappears at the left one row down, and text past the bottom edge
  * lands in the NEXT buffer -- which, the two being adjacent in memory, is
  * whatever is on screen. Both have been seen on hardware.
  *
  * This formats first so the box is known, clamps it inside the screen,
  * truncates anything still too wide, and keeps it clear of the GUI chrome.
  * Every label positioned by pixel rather than by LINE() goes through here.
  */
void ui_draw_label(int x, int y, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/** @brief Row text, including the state of the toggles. */
const char *ui_item_text(int item, const ui_view_t *v);

#endif /* UI_H */
