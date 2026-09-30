/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    ui.c
  * @brief   Touch GUI: power-on mode chooser, menu, calibration wizard.
  *          See ui.h.
  *
  * Red and white throughout: red is the only saturated colour on a camera
  * image, so it reads as chrome rather than as part of the scene, and a white
  * panel keeps text legible over whatever the camera is pointing at.
  ******************************************************************************
  */

#include "ui.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"
#include "stm32n6xx_hal.h"

/* Same face the HUD uses, so the GUI lines up with it. */
#ifdef STM32N6570_DK_REV
#define LCD_FONT Font20
#else
#define LCD_FONT Font12
#endif

/* ---------------------------------------------------------------- palette */

#define UI_RED        0xFFC62828u   /* primary: header, highlight, accents   */
#define UI_RED_DARK   0xFF8E1F1Fu   /* pressed / borders                     */
#define UI_RED_PALE   0xFFF6DADAu   /* selected row fill, still readable     */
#define UI_WHITE      0xFFFFFFFFu   /* panel ground                          */
#define UI_OFFWHITE   0xFFF2F2F2u   /* alternate row ground                  */
#define UI_INK        0xFF2A1212u   /* body text: near-black, warmed to red  */
#define UI_INK_SOFT   0xFF7A6060u   /* hints                                 */

/* ----------------------------------------------------------------- layout */

#define UI_TAB_W      104
#define UI_TAB_H      34
#define UI_TAB_X      12
#define UI_TAB_Y      12

#define UI_PANEL_W    460
#define UI_PANEL_H    306
#define UI_PANEL_X    ((LCD_BG_WIDTH  - UI_PANEL_W) / 2)
#define UI_PANEL_Y    ((LCD_BG_HEIGHT - UI_PANEL_H) / 2)

#define UI_HDR_H      44
#define UI_ROW_H      44
#define UI_ROW_X      (UI_PANEL_X + 10)
#define UI_ROW_W      (UI_PANEL_W - 20)
#define UI_ROW0_Y     (UI_PANEL_Y + UI_HDR_H + 8)

/* A finger is wider than a pixel: rows are 44 px tall, and the chooser's
 * entries -- the first thing anyone touches -- are 62 px. */
#define UI_BTN_W      180
#define UI_BTN_H      52
#define UI_BTN_Y      (UI_PANEL_Y + UI_PANEL_H - UI_BTN_H - 16)
#define UI_BTN_OK_X   (UI_PANEL_X + UI_PANEL_W - UI_BTN_W - 16)
#define UI_BTN_BACK_X (UI_PANEL_X + 16)

/* The chooser gets a taller panel of its own: three entries, each with a
 * title and a line of explanation. */
#define UI_CH_W       540
#define UI_CH_H       332
#define UI_CH_X       ((LCD_BG_WIDTH  - UI_CH_W) / 2)
#define UI_CH_Y       ((LCD_BG_HEIGHT - UI_CH_H) / 2)
#define UI_CH_ROW_H   62
#define UI_CH_ROW_X   (UI_CH_X + 12)
#define UI_CH_ROW_W   (UI_CH_W - 24)
#define UI_CH_ROW0_Y  (UI_CH_Y + UI_HDR_H + 16)

/* ------------------------------------------------------------------ state */

/* Buttons and touches arrive on the display task; the state machine runs on
 * the pipeline task. Only these counters cross between them, and a press is
 * never lost because the consumer subtracts what it has seen rather than
 * clearing to zero. */
static volatile uint32_t evt_next;
static volatile uint32_t evt_select;
static uint32_t seen_next;
static uint32_t seen_select;

/* A tap is a coordinate plus a sequence number, so the consumer can tell a
 * fresh tap from one it has already handled without a lock. */
static volatile uint32_t tap_seq;
static volatile int16_t tap_x, tap_y;
static uint32_t seen_tap;

static int touch_was_down;

static struct {
  ui_state_t state;
  int cursor;
  app_mode_t mode;
  int show_landmarks;
  int show_palmbox;
  uint32_t sweep_start_ms;
  int sweep_pct;
  uint32_t last_active_ms;    /* power saving: when something last happened */
  int asleep;
} u;

/* Reset the UI and start in the mode chooser. */
void ui_init(void)
{
  memset(&u, 0, sizeof(u));
  /* Boot into the chooser, not into a running scheme. Until someone picks,
   * the board sends the host nothing at all -- a simulator or a slide deck
   * already open on the PC cannot be driven by a hand that happened to be in
   * frame at power-on. */
  u.state = UI_STARTUP;
  u.cursor = APP_DEFAULT_MODE;
  u.mode = APP_DEFAULT_MODE;
  u.show_landmarks = 1;
  u.show_palmbox = 0;
  evt_next = evt_select = 0;
  seen_next = seen_select = 0;
  tap_seq = 0;
  seen_tap = 0;
  touch_was_down = 0;
  u.last_active_ms = HAL_GetTick();
  u.asleep = 0;
}

/* Tracked hands count as activity for power saving. */
void ui_note_activity(int hands_tracked)
{
  if (hands_tracked > 0)
    u.last_active_ms = HAL_GetTick();
}

int ui_is_asleep(void)
{
  return u.asleep;
}

/* USER1 pressed. Events are only counted here; the display task handles them. */
void ui_event_next(void)
{
  evt_next++;
}

/* TAMP pressed. */
void ui_event_select(void)
{
  evt_select++;
}

/* A tap at (x, y). */
void ui_event_touch(int x, int y)
{
  tap_x = (int16_t)x;
  tap_y = (int16_t)y;
  tap_seq++;
}

/* Feed the raw touch state; only a new press becomes a tap. */
void ui_touch_sample(int pressed, int x, int y)
{
  /* Press edge only. Without this a finger resting on a row would re-activate
   * it every frame -- thirty times a second. */
  if (pressed && !touch_was_down)
    ui_event_touch(x, y);

  touch_was_down = pressed ? 1 : 0;
}

/* Text of a menu row, including its current setting. */
const char *ui_item_text(int item, const ui_view_t *v)
{
  static char mode_row[40];

  switch (item) {
  case UI_ITEM_CALIBRATE: return "Calibrate for simulator";
  case UI_ITEM_MODE:
    snprintf(mode_row, sizeof(mode_row), "Mode:  %s", app_mode_name(v->mode));

    return mode_row;
  case UI_ITEM_LANDMARKS: return v->show_landmarks ? "Landmarks:  on" : "Landmarks:  off";
  case UI_ITEM_PALMBOX:   return v->show_palmbox   ? "Palm boxes: on" : "Palm boxes: off";
  case UI_ITEM_CLOSE:     return "Resume";
  default:                return "";
  }
}

/* ------------------------------------------------------------ mode change */

/* Adopt an application mode. The HMI control law lives in the hmi state, so
 * keep the two in step, and park the axes either way: switching with a live
 * throttle would hand the next scheme a flying drone. */
static void apply_mode(hmi_state_t *s, app_mode_t mode)
{
  u.mode = mode;
  hmi_set_mode(s, mode == APP_MODE_ACROBATIC ? HMI_MODE_ACROBATIC : HMI_MODE_ASSISTED);
  hmi_rest(s);
}

/* --------------------------------------------------------- the wizard math */

static void center_all(hmi_state_t *s)
{
  /* Step 1: dead centre on every axis, throttle included. The simulator is
   * recording what "neutral" reads as, so throttle must sit mid here even
   * though mid throttle is never the resting value in flight. */
  s->stick[HMI_LEFT].x = 0;
  s->stick[HMI_LEFT].y = 0;
  s->stick[HMI_RIGHT].x = 0;
  s->stick[HMI_RIGHT].y = 0;
}

/* Triangle wave over the sweep: 0 -> +1 -> -1 -> 0. Hits both extremes exactly
 * and returns to centre, which is the order calibration wizards expect. */
static float sweep_value(uint32_t elapsed_ms)
{
  float p = (float)elapsed_ms / (float)UI_SWEEP_MS;

  if (p < 0.25f)
    return 4.0f * p;                /* 0 -> +1 */
  if (p < 0.75f)
    return 2.0f - 4.0f * p;         /* +1 -> -1 */

  return 4.0f * p - 4.0f;           /* -1 -> 0 */
}

/* Carry out the highlighted menu row. */
static void activate_item(hmi_state_t *s)
{
  switch (u.cursor) {
  case UI_ITEM_CALIBRATE:
    /* Only the drone modes have sticks to calibrate. */
    if (!app_mode_is_hmi(u.mode))
      break;
    u.state = UI_CAL_CENTER;
    center_all(s);
    break;
  case UI_ITEM_MODE:
    apply_mode(s, (app_mode_t)((u.mode + 1) % APP_MODE_NB));
    break;
  case UI_ITEM_LANDMARKS:
    u.show_landmarks = !u.show_landmarks;
    break;
  case UI_ITEM_PALMBOX:
    u.show_palmbox = !u.show_palmbox;
    break;
  case UI_ITEM_CLOSE:
  default:
    u.state = UI_HIDDEN;
    hmi_rest(s);
    break;
  }
}

/* ------------------------------------------------------------- hit testing */

static int inside(int x, int y, int rx, int ry, int rw, int rh)
{
  return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

/* Act on whichever row or button a tap landed on. */
static void handle_tap(hmi_state_t *s, int x, int y)
{
  int i;

  if (u.state == UI_STARTUP) {
    for (i = 0; i < APP_MODE_NB; i++) {
      if (inside(x, y, UI_CH_ROW_X, UI_CH_ROW0_Y + i * UI_CH_ROW_H, UI_CH_ROW_W, UI_CH_ROW_H)) {
        u.cursor = i;
        apply_mode(s, (app_mode_t)i);
        u.state = UI_HIDDEN;

        return;
      }
    }

    return;
  }

  if (u.state == UI_HIDDEN) {
    if (inside(x, y, UI_TAB_X, UI_TAB_Y, UI_TAB_W, UI_TAB_H)) {
      u.state = UI_MENU;
      u.cursor = UI_ITEM_MODE;      /* the row people come here for */
      hmi_rest(s);
    }

    return;     /* taps elsewhere fall through: nothing else is live */
  }

  if (u.state == UI_MENU) {
    for (i = 0; i < UI_ITEM_NB; i++) {
      if (inside(x, y, UI_ROW_X, UI_ROW0_Y + i * UI_ROW_H, UI_ROW_W, UI_ROW_H)) {
        u.cursor = i;
        activate_item(s);

        return;
      }
    }

    return;     /* swallow taps on the panel so they do not leak through */
  }

  /* wizard: a big Back and a big Next/Fly */
  if (inside(x, y, UI_BTN_BACK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H)) {
    u.state = UI_MENU;
    hmi_rest(s);

    return;
  }
  if (inside(x, y, UI_BTN_OK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H)) {
    if (u.state == UI_CAL_CENTER) {
      u.state = UI_CAL_SWEEP;
      u.sweep_start_ms = HAL_GetTick();
    } else if (u.state == UI_CAL_DONE) {
      u.state = UI_HIDDEN;
      hmi_rest(s);
    }
  }
}

/* ---------------------------------------------------------- state machine */

int ui_apply(hmi_state_t *s)
{
  uint32_t next_presses = evt_next - seen_next;
  uint32_t select_presses = evt_select - seen_select;
  uint32_t taps = tap_seq - seen_tap;

  seen_next += next_presses;
  seen_select += select_presses;

  /* Any input at all counts as activity. If it arrived while asleep it is a
   * wake-up and nothing else: a tap meant to light the screen back up must
   * not also land on whatever row happens to be under the finger. */
  if (next_presses || select_presses || taps) {
    u.last_active_ms = HAL_GetTick();
    if (u.asleep) {
      u.asleep = 0;
      seen_tap = tap_seq;

      return 1;      /* swallowed; the GUI keeps the controls this frame */
    }
  }

  /* Only the most recent tap matters: taps are edge-triggered and far slower
   * than this loop, so more than one per frame means a bounce. */
  if (taps) {
    int x = tap_x;
    int y = tap_y;

    seen_tap += taps;
    handle_tap(s, x, y);
  }

  while (select_presses--) {
    switch (u.state) {
    case UI_STARTUP:
      apply_mode(s, (app_mode_t)u.cursor);
      u.state = UI_HIDDEN;
      break;
    case UI_HIDDEN:
      u.state = UI_MENU;
      u.cursor = UI_ITEM_MODE;
      hmi_rest(s);
      break;
    case UI_MENU:
      activate_item(s);
      break;
    default:
      /* anywhere in the wizard: back out to the menu */
      u.state = UI_MENU;
      hmi_rest(s);
      break;
    }
  }

  while (next_presses--) {
    switch (u.state) {
    case UI_STARTUP:
      u.cursor = (u.cursor + 1) % APP_MODE_NB;
      break;
    case UI_MENU:
      u.cursor = (u.cursor + 1) % UI_ITEM_NB;
      break;
    case UI_CAL_CENTER:
      u.state = UI_CAL_SWEEP;
      u.sweep_start_ms = HAL_GetTick();
      break;
    case UI_CAL_DONE:
      u.state = UI_HIDDEN;
      hmi_rest(s);
      break;
    case UI_HIDDEN:
    default:
      break;      /* nothing to do while running */
    }
  }

  /* What may time out, and what may not:
   *
   *  - an open panel means somebody is reading it;
   *  - a drone under power means somebody is FLYING it. Assisted mode holds
   *    the throttle when the hands let go -- that is the entire point of it --
   *    so hovering hands-off looks exactly like an idle board. Sleeping there
   *    would park the throttle and drop the aircraft out of the sky.
   *
   * Presenter mode has no persistent output, so only the first rule applies. */
  if (u.state != UI_HIDDEN)
    u.last_active_ms = HAL_GetTick();
  else if (app_mode_is_hmi(u.mode) && !hmi_is_at_rest(s))
    u.last_active_ms = HAL_GetTick();
  else if (!u.asleep && (HAL_GetTick() - u.last_active_ms) > UI_SLEEP_MS)
    u.asleep = 1;

  /* Asleep, the axes are parked and stay parked. Leaving them wherever the
   * last hand put them would hand the host a live throttle for as long as
   * nobody comes back -- a simulated drone would just keep flying. */
  if (u.asleep) {
    hmi_rest(s);

    return 1;
  }

  switch (u.state) {
  case UI_HIDDEN:
    u.sweep_pct = 0;

    return 0;     /* the hands are in control */

  case UI_STARTUP:
  case UI_MENU:
    hmi_rest(s);
    break;

  case UI_CAL_CENTER:
    center_all(s);
    break;

  case UI_CAL_SWEEP: {
    uint32_t elapsed = HAL_GetTick() - u.sweep_start_ms;

    if (elapsed >= UI_SWEEP_MS) {
      u.state = UI_CAL_DONE;
      u.sweep_pct = 100;
      center_all(s);
      break;
    }
    u.sweep_pct = (int)((elapsed * 100u) / UI_SWEEP_MS);
    {
      float v = sweep_value(elapsed);

      /* Every axis together. A simulator only records each axis's minimum and
       * maximum, so moving them in phase is enough and is far easier to
       * follow on screen than a phase-shifted pattern. */
      s->stick[HMI_LEFT].x = v;
      s->stick[HMI_LEFT].y = v;
      s->stick[HMI_RIGHT].x = v;
      s->stick[HMI_RIGHT].y = v;
    }
    break;
  }

  case UI_CAL_DONE:
    center_all(s);
    break;
  }

  return 1;       /* the GUI owns the controls */
}

/* Snapshot of the UI state for drawing. */
void ui_get_view(ui_view_t *out)
{
  out->state = u.state;
  out->cursor = u.cursor;
  out->mode = u.mode;
  out->show_landmarks = u.show_landmarks;
  out->show_palmbox = u.show_palmbox;
  out->sweep_pct = u.sweep_pct;
  out->asleep = u.asleep;
}

/* -------------------------------------------------------------- rendering */

/* Text position that centres a line of the current font in a box of height h.
 * UTIL_LCD has no vertical centring, and a glyph drawn past the bottom of the
 * frame buffer corrupts the neighbouring one (that was a real bug), so this is
 * always computed rather than eyeballed. */
static int text_y(int box_y, int box_h)
{
  return box_y + (box_h - (int)LCD_FONT.Height) / 2;
}

/* Centre a line of text on cx.
 *
 * NOT what UTIL_LCD's alignment enum does: its "centre" mode centres the
 * string on the whole SCREEN and then adds Xpos as an offset --
 *     refcolumn = Xpos + ((screen_chars - len) * font_width) / 2
 * -- so passing a panel's centre pushes the text that far right of the screen
 * centre and straight off the edge, where it wraps onto the following rows.
 * Every heading in this GUI wants centring on its own panel, so the position
 * is worked out here and the text drawn left-aligned. */
static void text_center(int cx, int y, const char *fmt, ...)
{
  char buf[64];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0)
    n = 0;

  UTIL_LCDEx_PrintfAt((uint32_t)(cx - n * (int)LCD_FONT.Width / 2), (uint32_t)y,
                      LEFT_MODE, "%s", buf);
}

static void fill_border(int x, int y, int w, int h, uint32_t fill, uint32_t border)
{
  UTIL_LCD_FillRect(x, y, w, h, fill);
  UTIL_LCD_DrawRect(x, y, w, h, border);
}

/* A button; primary ones are red. */
static void draw_button(int x, int y, int w, int h, const char *label, int primary)
{
  fill_border(x, y, w, h, primary ? UI_RED : UI_WHITE, primary ? UI_RED_DARK : UI_RED);
  UTIL_LCD_SetTextColor(primary ? UI_WHITE : UI_RED);
  UTIL_LCD_SetBackColor(primary ? UI_RED : UI_WHITE);
  text_center(x + w / 2, text_y(y, h), "%s", label);
}

/* The MENU tab, top left. */
static void draw_tab(void)
{
  fill_border(UI_TAB_X, UI_TAB_Y, UI_TAB_W, UI_TAB_H, UI_RED, UI_RED_DARK);
  UTIL_LCD_SetTextColor(UI_WHITE);
  UTIL_LCD_SetBackColor(UI_RED);
  text_center(UI_TAB_X + UI_TAB_W / 2, text_y(UI_TAB_Y, UI_TAB_H), "MENU");
}

/* A panel's red title bar. */
static void draw_header(int x, int y, int w, int h, const char *title)
{
  UTIL_LCD_FillRect(x, y, w, h, UI_RED);
  UTIL_LCD_SetTextColor(UI_WHITE);
  UTIL_LCD_SetBackColor(UI_RED);
  text_center(x + w / 2, text_y(y, h), "%s", title);
}

/* The mode chooser. */
static void draw_startup(const ui_view_t *v)
{
  int i;

  fill_border(UI_CH_X, UI_CH_Y, UI_CH_W, UI_CH_H, UI_WHITE, UI_RED);
  draw_header(UI_CH_X, UI_CH_Y, UI_CH_W, UI_HDR_H, "CHOOSE A MODE");

  for (i = 0; i < APP_MODE_NB; i++) {
    int y = UI_CH_ROW0_Y + i * UI_CH_ROW_H;
    int selected = (i == v->cursor);
    uint32_t ground = selected ? UI_RED_PALE : (i & 1 ? UI_OFFWHITE : UI_WHITE);

    UTIL_LCD_FillRect(UI_CH_ROW_X, y, UI_CH_ROW_W, UI_CH_ROW_H - 4, ground);
    if (selected)
      UTIL_LCD_FillRect(UI_CH_ROW_X, y, 6, UI_CH_ROW_H - 4, UI_RED);
    UTIL_LCD_SetBackColor(ground);

    UTIL_LCD_SetTextColor(selected ? UI_RED : UI_INK);
    UTIL_LCDEx_PrintfAt(UI_CH_ROW_X + 18, y + 6, LEFT_MODE, "%s",
                        app_mode_title((app_mode_t)i));
    UTIL_LCD_SetTextColor(UI_INK_SOFT);
    UTIL_LCDEx_PrintfAt(UI_CH_ROW_X + 18, y + 8 + (int)LCD_FONT.Height, LEFT_MODE, "%s",
                        app_mode_blurb((app_mode_t)i));
  }

  UTIL_LCD_SetTextColor(UI_INK_SOFT);
  UTIL_LCD_SetBackColor(UI_WHITE);
  text_center(UI_CH_X + UI_CH_W / 2, UI_CH_Y + UI_CH_H - 30,
                      "tap, or USER1 = move  TAMP = pick");
}

/* The menu panel. */
static void draw_menu(const ui_view_t *v)
{
  int i;

  fill_border(UI_PANEL_X, UI_PANEL_Y, UI_PANEL_W, UI_PANEL_H, UI_WHITE, UI_RED);
  draw_header(UI_PANEL_X, UI_PANEL_Y, UI_PANEL_W, UI_HDR_H, "MENU");

  for (i = 0; i < UI_ITEM_NB; i++) {
    int y = UI_ROW0_Y + i * UI_ROW_H;
    int selected = (i == v->cursor);
    int greyed = (i == UI_ITEM_CALIBRATE) && !app_mode_is_hmi(v->mode);

    UTIL_LCD_FillRect(UI_ROW_X, y, UI_ROW_W, UI_ROW_H - 2,
                      selected ? UI_RED_PALE : (i & 1 ? UI_OFFWHITE : UI_WHITE));
    if (selected) {
      /* a solid red rail rather than a full red row: the text stays dark on
       * pale, which is far easier to read than white-on-red at this size */
      UTIL_LCD_FillRect(UI_ROW_X, y, 5, UI_ROW_H - 2, UI_RED);
      UTIL_LCD_SetBackColor(UI_RED_PALE);
    } else {
      UTIL_LCD_SetBackColor(i & 1 ? UI_OFFWHITE : UI_WHITE);
    }
    UTIL_LCD_SetTextColor(greyed ? UI_INK_SOFT : (selected ? UI_RED : UI_INK));
    UTIL_LCDEx_PrintfAt(UI_ROW_X + 16, text_y(y, UI_ROW_H - 2), LEFT_MODE, "%s",
                        ui_item_text(i, v));
  }
}

/* The calibration wizard: centre step, then sweep with a progress bar. */
static void draw_wizard(const ui_view_t *v)
{
  int y = UI_PANEL_Y + UI_HDR_H + 18;
  int line_h = (int)LCD_FONT.Height + 6;
  int cx = UI_PANEL_X + UI_PANEL_W / 2;

  fill_border(UI_PANEL_X, UI_PANEL_Y, UI_PANEL_W, UI_PANEL_H, UI_WHITE, UI_RED);

  switch (v->state) {
  case UI_CAL_CENTER:
    draw_header(UI_PANEL_X, UI_PANEL_Y, UI_PANEL_W, UI_HDR_H, "CALIBRATION   1 / 2");
    UTIL_LCD_SetTextColor(UI_INK);
    UTIL_LCD_SetBackColor(UI_WHITE);
    text_center(cx, y, "All sticks are held CENTRED.");
    y += line_h;
    text_center(cx, y, "Let the simulator record");
    y += line_h;
    text_center(cx, y, "neutral, then tap Next.");
    draw_button(UI_BTN_BACK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H, "Back", 0);
    draw_button(UI_BTN_OK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H, "Next", 1);
    break;

  case UI_CAL_SWEEP: {
    int bar_x = UI_PANEL_X + 40;
    int bar_w = UI_PANEL_W - 80;

    draw_header(UI_PANEL_X, UI_PANEL_Y, UI_PANEL_W, UI_HDR_H, "CALIBRATION   2 / 2");
    UTIL_LCD_SetTextColor(UI_INK);
    UTIL_LCD_SetBackColor(UI_WHITE);
    text_center(cx, y, "Sweeping every axis through");
    y += line_h;
    text_center(cx, y, "its full range...");
    y += line_h + 14;
    fill_border(bar_x, y, bar_w, 22, UI_WHITE, UI_RED);
    UTIL_LCD_FillRect(bar_x + 2, y + 2, (uint32_t)((bar_w - 4) * v->sweep_pct / 100), 18, UI_RED);
    y += 34;
    UTIL_LCD_SetTextColor(UI_INK_SOFT);
    UTIL_LCD_SetBackColor(UI_WHITE);
    text_center(cx, y, "%d%%", v->sweep_pct);
    draw_button(UI_BTN_BACK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H, "Cancel", 0);
    break;
  }

  case UI_CAL_DONE:
    draw_header(UI_PANEL_X, UI_PANEL_Y, UI_PANEL_W, UI_HDR_H, "CALIBRATION DONE");
    UTIL_LCD_SetTextColor(UI_INK);
    UTIL_LCD_SetBackColor(UI_WHITE);
    text_center(cx, y, "Sticks are centred. Finish in");
    y += line_h;
    text_center(cx, y, "the simulator, then tap Fly.");
    y += line_h + 6;
    UTIL_LCD_SetTextColor(UI_INK_SOFT);
    text_center(cx, y, "throttle returns to MINIMUM");
    draw_button(UI_BTN_BACK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H, "Menu", 0);
    draw_button(UI_BTN_OK_X, UI_BTN_Y, UI_BTN_W, UI_BTN_H, "Fly", 1);
    break;

  default:
    break;
  }
}

/* Move a label down if it would cover the MENU tab. */
void ui_avoid_chrome(int *x, int *y, int w, int h)
{
  /* Only the tab is permanent chrome; the panels take the whole screen over
   * and the pipeline stops drawing under them anyway. */
  int overlaps_x = (*x < UI_TAB_X + UI_TAB_W) && (*x + w > UI_TAB_X);
  int overlaps_y = (*y < UI_TAB_Y + UI_TAB_H) && (*y + h > UI_TAB_Y);

  if (overlaps_x && overlaps_y)
    *y = UI_TAB_Y + UI_TAB_H + 4;
}

/* Draw a label kept on screen and clear of the tab. */
void ui_draw_label(int x, int y, const char *fmt, ...)
{
  char buf[64];
  va_list ap;
  int fw = (int)LCD_FONT.Width;
  int fh = (int)LCD_FONT.Height;
  int max_chars = LCD_BG_WIDTH / fw;
  int n, w;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0)
    n = 0;
  if (n > (int)sizeof(buf) - 1)
    n = (int)sizeof(buf) - 1;          /* vsnprintf reports the untruncated length */

  /* Truncate rather than wrap: a string wider than the screen has nowhere to
   * go, and wrapping is what produced the artefact in the first place. */
  if (n > max_chars) {
    n = max_chars;
    buf[n] = 0;
  }
  w = n * fw;

  if (x + w > LCD_BG_WIDTH)
    x = LCD_BG_WIDTH - w;
  if (x < 0)
    x = 0;
  if (y + fh > LCD_BG_HEIGHT)
    y = LCD_BG_HEIGHT - fh;
  if (y < 0)
    y = 0;

  ui_avoid_chrome(&x, &y, w, fh);
  if (y + fh > LCD_BG_HEIGHT)          /* avoid_chrome pushes down; re-clamp */
    y = LCD_BG_HEIGHT - fh;

  UTIL_LCDEx_PrintfAt((uint32_t)x, (uint32_t)y, LEFT_MODE, "%s", buf);
}

/* Power saving: the camera is stopped and the background is black, so all
 * that is left is a line saying the board is still awake enough to be woken.
 * Deliberately small and dim -- it may sit there for a long time, and a bright
 * panel would defeat the point. */
static void draw_asleep(void)
{
  int cx = LCD_BG_WIDTH / 2;
  int cy = LCD_BG_HEIGHT / 2;

  UTIL_LCD_SetBackColor(0x00000000);
  UTIL_LCD_SetTextColor(UI_RED);
  text_center(cx, cy - (int)LCD_FONT.Height, "POWER SAVING");
  UTIL_LCD_SetTextColor(UI_INK_SOFT);
  text_center(cx, cy + 4, "touch the screen to wake");
}

/* Draw the UI layer: sleep notice, chooser, menu or wizard, and the tab. */
void ui_draw(const ui_view_t *v)
{
  uint32_t saved_text = UTIL_LCD_GetTextColor();
  uint32_t saved_back = UTIL_LCD_GetBackColor();

  if (v->asleep) {
    draw_asleep();
    UTIL_LCD_SetTextColor(saved_text);
    UTIL_LCD_SetBackColor(saved_back);

    return;
  }

  switch (v->state) {
  case UI_STARTUP: draw_startup(v); break;
  case UI_HIDDEN:  draw_tab();      break;   /* the only chrome while running */
  case UI_MENU:    draw_menu(v);    break;
  default:         draw_wizard(v);  break;
  }

  /* The HUD draws with its own colours and does not set them every time. */
  UTIL_LCD_SetTextColor(saved_text);
  UTIL_LCD_SetBackColor(saved_back);
}
