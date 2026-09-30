/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    usb_hid.h
  * @brief   USB HID device on USB1 (CN18): an ordinary composite
  *          mouse + keyboard + gamepad.
  *
  * No driver, no PC-side software, no pairing. Windows, macOS and Linux all
  * bind their built-in HID drivers, and PowerPoint, Keynote, Impress and
  * Google Slides all respond to the two keys it sends without being told
  * anything.
  *
  * One HID interface carries both devices, separated by report IDs -- the
  * same arrangement every real clicker uses:
  *
  *   report 1  mouse     buttons + relative X/Y   -> presenter pointer
  *   report 2  keyboard  standard 8-byte report   -> Page Down / Page Up
  *   report 3  gamepad   4 signed axes            -> flight sticks
  *
  * All three are declared all the time, in every application mode. A USB
  * device cannot swap its report descriptor without re-enumerating, and the
  * menu can change mode at any moment, so the descriptor is the union and
  * each mode simply sends only the reports it needs. To the host this is one
  * unremarkable composite gadget; nothing is installed for any of it.
  *
  * USBX device stack in standalone mode, serviced by one micro T-Kernel task.
  * The application never talks to USBX directly.
  ******************************************************************************
  */

#ifndef USB_HID_H
#define USB_HID_H

#include <stdint.h>

/* Gamepad payload, report 3. HID Y axes grow "down", so the caller negates
 * pitch and throttle.
 *   x, y   right stick (roll, pitch)
 *   rx, ry left stick  (yaw, throttle)
 * No buttons: the HMI's gestures are axis selectors consumed on the board. */
typedef struct __attribute__((packed)) {
  int8_t x;
  int8_t y;
  int8_t rx;
  int8_t ry;
} usb_hid_gamepad_report_t;

/* HID Usage Tables, keyboard page. What every presenter remote sends, and
 * what every presentation program listens for. */
#define USB_HID_KEY_PAGEDOWN  0x4E
#define USB_HID_KEY_PAGEUP    0x4B

/* Minimum time a key is held down, and the minimum gap before the next
 * keyboard report goes out. The USB interrupt endpoint is polled every
 * millisecond, so without this a press and its release could be 1 ms apart --
 * legal, and handled correctly by any host that reads every report, but
 * needlessly close to the edge for something that must never double-fire or
 * get lost. 24 ms is about one pipeline frame and is comfortably long. */
#ifndef USB_HID_KEY_GAP_MS
#define USB_HID_KEY_GAP_MS 24
#endif

/**
  * @brief  Bring up USB1 as a HID presenter and start the service task.
  *         Call once from the main task after SystemClock_Config().
  * @retval 0 on success.
  */
int usb_hid_init(void);

/** @brief 1 once the host has enumerated and configured the device. */
int usb_hid_is_configured(void);

/**
  * @brief  Move the pointer by a relative step. Non-blocking, callable from
  *         any task. One-deep mailbox, latest wins: if the host is slower
  *         than the pipeline the intermediate steps are dropped, which for a
  *         pointer is right -- the newest position is the only one that
  *         matters.
  * @param  dx, dy  pixels, host-side; positive dy is down, as HID defines it
  * @retval 0 if queued, -1 if the host has not configured the device yet.
  */
int usb_hid_mouse_move(int8_t dx, int8_t dy);

/**
  * @brief  Press and release one key. Non-blocking.
  *
  * Unlike the pointer, keystrokes are queued and never dropped: losing a
  * release would leave the host repeating the key forever. If the queue is
  * full the new keystroke is refused instead, which at worst costs one slide
  * change the user can repeat.
  *
  * @param  keycode  USB_HID_KEY_PAGEDOWN or USB_HID_KEY_PAGEUP
  * @retval 0 if queued, -1 if not configured or the queue is full.
  */
int usb_hid_key_tap(uint8_t keycode);

/**
  * @brief  Queue a gamepad report. Non-blocking, callable from any task.
  *         One-deep mailbox, latest wins: if the host is slower than the
  *         pipeline the intermediate reports are dropped, which for a stick
  *         is right -- only the newest position matters.
  * @retval 0 if queued, -1 if the host has not configured the device yet.
  */
int usb_hid_gamepad_send(const usb_hid_gamepad_report_t *report);

/**
  * @brief  Give the service task a turn. Call once per pipeline iteration.
  *
  * USB interrupts only arrive while transfers are in flight, so with an idle
  * pointer and a queued keystroke there would be nothing to wake the task.
  * This is that wake-up.
  */
void usb_hid_poll(void);

#endif /* USB_HID_H */
