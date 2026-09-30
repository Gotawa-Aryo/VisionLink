/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    usb_hid.c
  * @brief   USB HID presenter remote on USB1 (CN18), USBX standalone under
  *          micro T-Kernel 3.0.
  *
  * The PHY bring-up, the threading model and the port hooks are unchanged from
  * the parent project's gamepad, which in turn follows ST's own uvcl_usbx.c /
  * scrl_usb.c for this silicon. What is new is the device itself: a composite
  * mouse + keyboard behind report IDs instead of a gamepad.
  *
  *   USB1_OTG_HS_IRQHandler : HAL_PCD_IRQHandler(), mask the IRQ, signal
  *   usb task               : run the USBX standalone state machines
  *                            (ux_system_tasks_run) with the IRQ masked,
  *                            push at most one queued report, unmask
  *
  * Everything that touches USBX therefore runs in exactly one context, the
  * usb task, so none of the stack's internal bookkeeping needs locking.
  *
  * Two queues, because the two devices want opposite failure modes:
  *
  *   mouse     one-deep mailbox, newest wins. Dropping an intermediate step
  *             of a pointer is invisible; delivering a stale one is not.
  *   keyboard  a real FIFO. A dropped key release leaves the host repeating
  *             Page Down until something else stops it, so keystrokes are
  *             refused at the door rather than lost in transit.
  ******************************************************************************
  */

#include "usb_hid.h"

#include <assert.h>
#include <string.h>

#include "os_utk.h"
#include "stm32n6xx_hal.h"
#include "ux_api.h"
#include "ux_dcd_stm32.h"
#include "ux_device_class_hid.h"

#ifndef MTK_MANAGED_IRQ_PRIORITY
#define MTK_MANAGED_IRQ_PRIORITY 5
#endif

#define USBX_MEM_SIZE   (24 * 1024)

/* STMicroelectronics' vendor ID -- legitimate on an ST development board. The
 * product ID is one not used by any ST example in this tree; a product that
 * shipped would need its own allocation from the USB-IF. */
#define USB_VID         0x0483
/* A different shape of device from either single-mode build (0x5710 was
 * gamepad-only, 0x572A mouse+keyboard), so it gets its own product ID rather
 * than confusing a host that cached one of those descriptors. */
#define USB_PID         0x572B

#define HID_REPORT_ID_MOUSE     1
#define HID_REPORT_ID_KEYBOARD  2
#define HID_REPORT_ID_GAMEPAD   3

/* Payload sizes, NOT counting the report ID: USBX prepends that itself from
 * ux_device_class_hid_event_report_id. */
#define MOUSE_REPORT_LEN    3   /* buttons, x, y */
#define KEYBOARD_REPORT_LEN 8   /* modifier, reserved, 6 keycodes */
#define GAMEPAD_REPORT_LEN  4   /* x, y, rx, ry */

/* --- descriptors --------------------------------------------------------- */

/* Three top-level collections on one interface, told apart by report ID.
 * This is the ordinary way a clicker, a keyboard with a trackpoint, or any
 * other composite HID gadget presents itself, and needs no driver anywhere.
 *
 * All three are declared in every application mode: the descriptor cannot
 * change without re-enumerating, and the menu can switch mode at any time. */
static const uint8_t hid_report_desc[] = {
  /* ---- report 1: mouse ---- */
  0x05, 0x01,        /* Usage Page (Generic Desktop)         */
  0x09, 0x02,        /* Usage (Mouse)                        */
  0xA1, 0x01,        /* Collection (Application)             */
  0x85, HID_REPORT_ID_MOUSE,
  0x09, 0x01,        /*   Usage (Pointer)                    */
  0xA1, 0x00,        /*   Collection (Physical)              */
  0x05, 0x09,        /*     Usage Page (Button)              */
  0x19, 0x01,        /*     Usage Minimum (Button 1)         */
  0x29, 0x03,        /*     Usage Maximum (Button 3)         */
  0x15, 0x00,        /*     Logical Minimum (0)              */
  0x25, 0x01,        /*     Logical Maximum (1)              */
  0x75, 0x01,        /*     Report Size (1)                  */
  0x95, 0x03,        /*     Report Count (3)                 */
  0x81, 0x02,        /*     Input (Data,Var,Abs)             */
  0x75, 0x05,        /*     Report Size (5)                  */
  0x95, 0x01,        /*     Report Count (1)                 */
  0x81, 0x03,        /*     Input (Cnst,Var,Abs)  padding    */
  0x05, 0x01,        /*     Usage Page (Generic Desktop)     */
  0x09, 0x30,        /*     Usage (X)                        */
  0x09, 0x31,        /*     Usage (Y)                        */
  0x15, 0x81,        /*     Logical Minimum (-127)           */
  0x25, 0x7F,        /*     Logical Maximum (127)            */
  0x75, 0x08,        /*     Report Size (8)                  */
  0x95, 0x02,        /*     Report Count (2)                 */
  0x81, 0x06,        /*     Input (Data,Var,Rel)             */
  0xC0,              /*   End Collection                     */
  0xC0,              /* End Collection                       */

  /* ---- report 2: keyboard ---- */
  0x05, 0x01,        /* Usage Page (Generic Desktop)         */
  0x09, 0x06,        /* Usage (Keyboard)                     */
  0xA1, 0x01,        /* Collection (Application)             */
  0x85, HID_REPORT_ID_KEYBOARD,
  0x05, 0x07,        /*   Usage Page (Keyboard/Keypad)       */
  0x19, 0xE0,        /*   Usage Minimum (Left Control)       */
  0x29, 0xE7,        /*   Usage Maximum (Right GUI)          */
  0x15, 0x00,        /*   Logical Minimum (0)                */
  0x25, 0x01,        /*   Logical Maximum (1)                */
  0x75, 0x01,        /*   Report Size (1)                    */
  0x95, 0x08,        /*   Report Count (8)                   */
  0x81, 0x02,        /*   Input (Data,Var,Abs)  modifiers    */
  0x75, 0x08,        /*   Report Size (8)                    */
  0x95, 0x01,        /*   Report Count (1)                   */
  0x81, 0x03,        /*   Input (Cnst,Var,Abs)  reserved     */
  0x15, 0x00,        /*   Logical Minimum (0)                */
  0x25, 0x65,        /*   Logical Maximum (101)              */
  0x05, 0x07,        /*   Usage Page (Keyboard/Keypad)       */
  0x19, 0x00,        /*   Usage Minimum (0)                  */
  0x29, 0x65,        /*   Usage Maximum (101)                */
  0x75, 0x08,        /*   Report Size (8)                    */
  0x95, 0x06,        /*   Report Count (6)                   */
  0x81, 0x00,        /*   Input (Data,Ary,Abs)  keycodes     */
  0xC0,              /* End Collection                       */

  /* ---- report 3: gamepad ---- */
  0x05, 0x01,        /* Usage Page (Generic Desktop)         */
  0x09, 0x05,        /* Usage (Game Pad)                     */
  0xA1, 0x01,        /* Collection (Application)             */
  0x85, HID_REPORT_ID_GAMEPAD,
  0x09, 0x01,        /*   Usage (Pointer)                    */
  0xA1, 0x00,        /*   Collection (Physical)              */
  0x09, 0x30,        /*     Usage (X)   right stick, roll    */
  0x09, 0x31,        /*     Usage (Y)   right stick, pitch   */
  0x09, 0x33,        /*     Usage (Rx)  left stick, yaw      */
  0x09, 0x34,        /*     Usage (Ry)  left stick, throttle */
  0x15, 0x81,        /*     Logical Minimum (-127)           */
  0x25, 0x7F,        /*     Logical Maximum (127)            */
  0x75, 0x08,        /*     Report Size (8)                  */
  0x95, 0x04,        /*     Report Count (4)                 */
  0x81, 0x02,        /*     Input (Data,Var,Abs)             */
  0xC0,              /*   End Collection                     */
  0xC0,              /* End Collection                       */
};

/* The largest thing on the wire is a keyboard report plus its ID: 9 bytes.
 * 16 leaves room and keeps the packet size a round number. */
#define EP_IN_ADDR   0x81
#define EP_IN_MPS    16

_Static_assert(KEYBOARD_REPORT_LEN + 1 <= EP_IN_MPS,
               "a report must fit in one interrupt packet");
_Static_assert(KEYBOARD_REPORT_LEN + 1 <= UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH,
               "raise UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH in Lib_usb/ux_user.h");

#define DEV_DESC_LEN   18
#define QUAL_DESC_LEN  10
#define CFG_TOTAL_LEN  (9 + 9 + 9 + 7)

/* USBX "device framework": device descriptor [+ qualifier], then the whole
 * configuration. Built at init so the HID descriptor's report length and the
 * endpoint interval can be filled from the constants above. */
static uint8_t fw_hs[DEV_DESC_LEN + QUAL_DESC_LEN + CFG_TOTAL_LEN];
static uint8_t fw_fs[DEV_DESC_LEN + CFG_TOTAL_LEN];

/* Device, configuration and HID descriptors, for full or high speed. */
static int build_framework(uint8_t *p, int is_hs)
{
  const uint8_t *start = p;
  uint16_t rlen = sizeof(hid_report_desc);

  /* device descriptor */
  *p++ = DEV_DESC_LEN; *p++ = 0x01;
  *p++ = 0x00; *p++ = 0x02;               /* bcdUSB 2.00 */
  *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;  /* class defined at interface level */
  *p++ = 0x40;                            /* EP0 max packet 64 */
  *p++ = USB_VID & 0xff; *p++ = USB_VID >> 8;
  *p++ = USB_PID & 0xff; *p++ = USB_PID >> 8;
  *p++ = 0x00; *p++ = 0x01;               /* bcdDevice 1.00 */
  *p++ = 1; *p++ = 2; *p++ = 3;           /* iManufacturer, iProduct, iSerial */
  *p++ = 1;                               /* one configuration */

  if (is_hs) {
    /* device qualifier: what we look like at the other speed */
    *p++ = QUAL_DESC_LEN; *p++ = 0x06;
    *p++ = 0x00; *p++ = 0x02;
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;
    *p++ = 0x40; *p++ = 1; *p++ = 0;
  }

  /* configuration */
  *p++ = 9; *p++ = 0x02;
  *p++ = CFG_TOTAL_LEN & 0xff; *p++ = CFG_TOTAL_LEN >> 8;
  *p++ = 1;                               /* one interface */
  *p++ = 1;                               /* bConfigurationValue */
  *p++ = 0;                               /* iConfiguration */
  *p++ = 0xC0;                            /* self powered (the DK runs off CN6) */
  *p++ = 50;                              /* 100 mA */

  /* interface: HID. Subclass 0 / protocol 0 on purpose -- a boot-protocol
   * keyboard (subclass 1) must put an 8-byte report with no report ID first,
   * which a composite device cannot do. Every modern host uses the report
   * protocol regardless, and a BIOS is not the audience for a slide clicker. */
  *p++ = 9; *p++ = 0x04;
  *p++ = 0; *p++ = 0; *p++ = 1;
  *p++ = 0x03; *p++ = 0x00; *p++ = 0x00;
  *p++ = 0;

  /* HID descriptor */
  *p++ = 9; *p++ = 0x21;
  *p++ = 0x11; *p++ = 0x01;               /* bcdHID 1.11 */
  *p++ = 0x00;                            /* country code */
  *p++ = 1;                               /* one class descriptor follows */
  *p++ = 0x22;                            /* ... a report descriptor */
  *p++ = rlen & 0xff; *p++ = rlen >> 8;

  /* interrupt IN endpoint, polled every 1 ms at either speed */
  *p++ = 7; *p++ = 0x05;
  *p++ = EP_IN_ADDR; *p++ = 0x03;
  *p++ = EP_IN_MPS; *p++ = 0x00;
  *p++ = is_hs ? 4 : 1;                   /* HS: 2^(4-1) microframes = 1 ms */

  return p - start;
}

/* USBX string framework: {langid lo, langid hi, index, length, ascii...} */
static const uint8_t str_framework[] = {
  0x09, 0x04, 0x01, 18, 'S','T','M','i','c','r','o','e','l','e','c','t','r','o','n','i','c','s',
  0x09, 0x04, 0x02, 10, 'V','i','s','i','o','n','L','i','n','k',
  0x09, 0x04, 0x03,  9, 'N','6','P','R','E','S','0','0','1',
};
static const uint8_t lang_framework[] = { 0x09, 0x04 };

/* --- state --------------------------------------------------------------- */

static uint8_t usbx_mem_pool[USBX_MEM_SIZE] __attribute__((aligned(32)));
static PCD_HandleTypeDef hpcd;
static UX_SLAVE_CLASS_HID *hid_instance;
static volatile int is_configured;

static os_task_t usb_task;
static uint8_t usb_task_stack[OS_MINIMAL_STACK_SIZE] __attribute__((aligned(8)));
static os_sem_t usb_sem;

/* mouse: one-deep mailbox, latest wins */
static uint8_t mouse_report[MOUSE_REPORT_LEN];
static volatile int mouse_pending;
static uint8_t gamepad_report[GAMEPAD_REPORT_LEN];
static volatile int gamepad_pending;

/* keyboard: single-producer single-consumer ring. Power of two so the index
 * arithmetic is a mask, and one slot always left empty so head == tail can
 * only ever mean "empty". */
#define KEY_QUEUE_LEN 8
static uint8_t key_queue[KEY_QUEUE_LEN][KEYBOARD_REPORT_LEN];
static volatile uint32_t key_head;   /* written by the producer */
static volatile uint32_t key_tail;   /* written by the usb task */
static uint32_t key_last_ms;

/* --- USBX standalone port hooks ------------------------------------------ */

ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
  ALIGN_TYPE primask = __get_PRIMASK();

  __disable_irq();

  return primask;
}

/* USBX hook: re-enable interrupts if they were on before. */
VOID _ux_utility_interrupt_restore(ALIGN_TYPE primask)
{
  if (!primask)
    __enable_irq();
}

ULONG _ux_utility_time_get(VOID)
{
  return HAL_GetTick();   /* UX_PERIODIC_RATE is 1000: one tick per ms */
}

/* --- HID class callbacks ------------------------------------------------- */

static VOID hid_instance_activate(VOID *instance)
{
  hid_instance = (UX_SLAVE_CLASS_HID *)instance;
  is_configured = 1;
}

/* USBX callback: the host unplugged or deconfigured the device. */
static VOID hid_instance_deactivate(VOID *instance)
{
  (void)instance;
  is_configured = 0;
  hid_instance = UX_NULL;

  /* Unplugged mid-keystroke. Drop whatever is queued: on the next plug-in the
   * host starts from all-keys-up anyway, and delivering a stale release --
   * or worse, a stale press -- into a fresh session would be wrong. */
  key_tail = key_head;
  mouse_pending = 0;
  gamepad_pending = 0;
}

/* Host -> device SET_REPORT. A keyboard's output report is the LED state;
 * there are no LEDs here, so accept and discard. */
static UINT hid_set_report(UX_SLAVE_CLASS_HID *hid, UX_SLAVE_CLASS_HID_EVENT *ev)
{
  (void)hid; (void)ev;
  return UX_SUCCESS;
}

/* GET_REPORT over the control pipe. Hosts ask for this at enumeration; answer
 * with a neutral report of the right shape rather than whatever happens to be
 * in the mailbox, so the pointer cannot be nudged by a control transfer. */
static UINT hid_get_report(UX_SLAVE_CLASS_HID *hid, UX_SLAVE_CLASS_HID_EVENT *ev)
{
  (void)hid;

  if (ev->ux_device_class_hid_event_report_id == HID_REPORT_ID_KEYBOARD) {
    ev->ux_device_class_hid_event_length = KEYBOARD_REPORT_LEN;
    memset(ev->ux_device_class_hid_event_buffer, 0, KEYBOARD_REPORT_LEN);
  } else {
    ev->ux_device_class_hid_event_report_id = HID_REPORT_ID_MOUSE;
    ev->ux_device_class_hid_event_length = MOUSE_REPORT_LEN;
    memset(ev->ux_device_class_hid_event_buffer, 0, MOUSE_REPORT_LEN);
  }

  return UX_SUCCESS;
}

/* --- HAL MSP ------------------------------------------------------------- */

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
  assert(pcd->Instance == USB1_OTG_HS);

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_SLEEP_ENABLE();

  /* VDD33USB monitor + supply, exactly as scrl_usb.c does it */
  HAL_PWREx_EnableVddUSBVMEN();
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_USB33RDY) == 0U)
    ;
  HAL_PWREx_EnableVddUSB();

  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
  __HAL_RCC_USB1_OTG_HS_CLK_SLEEP_ENABLE();

  /* Embedded HS PHY reference: HSE/2 = 24 MHz on the DK (48 MHz crystal).
   * The RCC mux resets to that, so this only pins it; FSEL must agree. */
  USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 4);
  USB1_HS_PHYC->USBPHYC_CR |= (0x2U << 4);      /* FSEL = 24 MHz */

  __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();
  __HAL_RCC_USB1_OTG_HS_PHY_CLK_SLEEP_ENABLE();

  /* Same level as every other peripheral IRQ under this kernel: the ISR
   * signals a semaphore, so it must be maskable by disint(). */
  HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, MTK_MANAGED_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
}

/* USB interrupt: service the controller, then hand over to the usb task. */
void USB1_OTG_HS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);

  /* Hand the rest to the task; it re-enables the line when done. */
  HAL_NVIC_DisableIRQ(USB1_OTG_HS_IRQn);
  (void)os_sem_give(usb_sem);
}

/* --- the service task ---------------------------------------------------- */

static void hid_event_push(uint8_t report_id, const uint8_t *payload, uint32_t len)
{
  UX_SLAVE_CLASS_HID_EVENT ev;

  ev.ux_device_class_hid_event_report_id = report_id;
  ev.ux_device_class_hid_event_report_type = UX_DEVICE_CLASS_HID_REPORT_TYPE_INPUT;
  ev.ux_device_class_hid_event_length = len;
  memcpy(ev.ux_device_class_hid_event_buffer, payload, len);

  /* Queue full means the host is not draining. For the pointer the next
   * report supersedes this one anyway; for a keystroke the caller has
   * already been told the queue depth is finite. Either way, not worth
   * stopping the pipeline over. */
  (void)ux_device_class_hid_event_set(hid_instance, &ev);
}

/* USB task: send pending reports, step USBX, then re-enable the interrupt. */
static void usb_task_fct(void *arg)
{
  (void)arg;

  for (;;) {
    int ret = os_sem_take(usb_sem, OS_BLOCKING);
    assert(ret == OS_OK);

    /* Serialise against the ISR: it must not run HAL_PCD_IRQHandler while
     * the standalone state machines are mid-step. A pending interrupt is
     * held, not lost, and fires the moment the line is re-enabled. */
    HAL_NVIC_DisableIRQ(USB1_OTG_HS_IRQn);

    if (is_configured && hid_instance) {
      uint32_t now = HAL_GetTick();

      /* At most one keyboard report per pass, and never closer together than
       * USB_HID_KEY_GAP_MS, so every press is held long enough to be
       * unambiguous. The queue drains at 1 report per gap, i.e. a keystroke
       * (press + release) takes about 48 ms to go out. */
      if (key_tail != key_head && (now - key_last_ms) >= USB_HID_KEY_GAP_MS) {
        hid_event_push(HID_REPORT_ID_KEYBOARD,
                       key_queue[key_tail % KEY_QUEUE_LEN], KEYBOARD_REPORT_LEN);
        key_last_ms = now;
        __DMB();
        key_tail++;
      }

      if (mouse_pending) {
        mouse_pending = 0;
        __DMB();
        hid_event_push(HID_REPORT_ID_MOUSE, mouse_report, MOUSE_REPORT_LEN);
      }

      if (gamepad_pending) {
        gamepad_pending = 0;
        __DMB();
        hid_event_push(HID_REPORT_ID_GAMEPAD, gamepad_report, GAMEPAD_REPORT_LEN);
      }
    }

    /* Twice, as ST's UVC library does: the first pass can complete a
     * transfer whose completion the second pass then consumes. */
    ux_system_tasks_run();
    ux_system_tasks_run();

    HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
  }
}

/* --- public API ---------------------------------------------------------- */

int usb_hid_is_configured(void)
{
  return is_configured;
}

/* Wake the usb task; with no transfer in flight, no USB interrupt would. */
void usb_hid_poll(void)
{
  if (is_configured)
    (void)os_sem_give(usb_sem);
}

/* Queue a relative pointer move; the newest one wins. */
int usb_hid_mouse_move(int8_t dx, int8_t dy)
{
  if (!is_configured)
    return -1;

  /* Plain store into the mailbox; the task only ever reads it with the flag
   * set, and a torn read costs one frame of a slightly wrong step. */
  mouse_report[0] = 0;               /* no buttons: this is a pointer, not a click */
  mouse_report[1] = (uint8_t)dx;
  mouse_report[2] = (uint8_t)dy;
  __DMB();
  mouse_pending = 1;
  (void)os_sem_give(usb_sem);

  return 0;
}

/* Queue a gamepad report; the newest one wins. */
int usb_hid_gamepad_send(const usb_hid_gamepad_report_t *report)
{
  if (!is_configured)
    return -1;

  /* Same one-deep mailbox as the pointer, for the same reason. */
  gamepad_report[0] = (uint8_t)report->x;
  gamepad_report[1] = (uint8_t)report->y;
  gamepad_report[2] = (uint8_t)report->rx;
  gamepad_report[3] = (uint8_t)report->ry;
  __DMB();
  gamepad_pending = 1;
  (void)os_sem_give(usb_sem);

  return 0;
}

/* Queue a key press and its release together. */
int usb_hid_key_tap(uint8_t keycode)
{
  uint32_t head = key_head;
  uint8_t *press, *release;

  if (!is_configured)
    return -1;

  /* Two slots, or none: queuing a press without room for its release is the
   * one outcome that must never happen. */
  if ((head + 2) - key_tail > KEY_QUEUE_LEN - 1)
    return -1;

  press = key_queue[head % KEY_QUEUE_LEN];
  memset(press, 0, KEYBOARD_REPORT_LEN);
  press[2] = keycode;                /* [0] modifiers, [1] reserved, [2..7] keys */

  release = key_queue[(head + 1) % KEY_QUEUE_LEN];
  memset(release, 0, KEYBOARD_REPORT_LEN);

  __DMB();
  key_head = head + 2;
  (void)os_sem_give(usb_sem);

  return 0;
}

/* Start USBX and the HID class on CN18, and create the usb task. */
int usb_hid_init(void)
{
  UX_SLAVE_CLASS_HID_PARAMETER hid_param;
  RCC_OscInitTypeDef osc = { 0 };
  int fs_len, hs_len;
  int ret;

  /* HSE feeds the PHY. SystemClock_Config() has it on already; this is the
   * same belt-and-braces ST's UVC library uses. */
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  ret = HAL_RCC_OscConfig(&osc);
  if (ret != HAL_OK)
    return -1;

  ret = os_sem_create(&usb_sem, 0, 1);
  if (ret != OS_OK)
    return -1;

  /* PCD: high speed, embedded PHY, FIFO mode (no DMA, so no cache concerns) */
  memset(&hpcd, 0, sizeof(hpcd));
  hpcd.Instance = USB1_OTG_HS;
  hpcd.Init.dev_endpoints = 9;
  hpcd.Init.speed = PCD_SPEED_HIGH;
  hpcd.Init.dma_enable = DISABLE;
  hpcd.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd.Init.Sof_enable = DISABLE;
  hpcd.Init.low_power_enable = DISABLE;
  hpcd.Init.lpm_enable = DISABLE;
  hpcd.Init.vbus_sensing_enable = DISABLE;
  hpcd.Init.use_dedicated_ep1 = DISABLE;
  hpcd.Init.use_external_vbus = DISABLE;
  ret = HAL_PCD_Init(&hpcd);
  if (ret != HAL_OK)
    return -1;
  HAL_PCDEx_SetRxFiFo(&hpcd, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x40);

  /* USBX */
  ret = ux_system_initialize(usbx_mem_pool, USBX_MEM_SIZE, UX_NULL, 0);
  if (ret != UX_SUCCESS)
    return -1;

  hs_len = build_framework(fw_hs, 1);
  fs_len = build_framework(fw_fs, 0);
  assert(hs_len == sizeof(fw_hs) && fs_len == sizeof(fw_fs));

  ret = ux_device_stack_initialize(fw_hs, hs_len, fw_fs, fs_len,
                                   (UCHAR *)str_framework, sizeof(str_framework),
                                   (UCHAR *)lang_framework, sizeof(lang_framework), UX_NULL);
  if (ret != UX_SUCCESS)
    return -1;

  memset(&hid_param, 0, sizeof(hid_param));
  hid_param.ux_slave_class_hid_instance_activate = hid_instance_activate;
  hid_param.ux_slave_class_hid_instance_deactivate = hid_instance_deactivate;
  hid_param.ux_device_class_hid_parameter_report_address = (UCHAR *)hid_report_desc;
  /* UX_TRUE: the reports carry an ID, and USBX prepends it for us from
   * ux_device_class_hid_event_report_id. Without this the mouse and the
   * keyboard would be indistinguishable on the wire. */
  hid_param.ux_device_class_hid_parameter_report_id = UX_TRUE;
  hid_param.ux_device_class_hid_parameter_report_length = sizeof(hid_report_desc);
  hid_param.ux_device_class_hid_parameter_callback = hid_set_report;
  hid_param.ux_device_class_hid_parameter_get_callback = hid_get_report;
  ret = ux_device_stack_class_register(_ux_system_slave_class_hid_name, ux_device_class_hid_entry,
                                       1, 0, &hid_param);
  if (ret != UX_SUCCESS)
    return -1;

  ret = ux_dcd_stm32_initialize((ULONG)USB1_OTG_HS, (ULONG)&hpcd);
  if (ret != UX_SUCCESS)
    return -1;

  /* Above the pipeline tasks: the work is microseconds and the host polls
   * every millisecond. Same slot ST's UVC FreeRTOS variant picks. */
  ret = os_task_create(&usb_task, usb_task_fct, "usb", usb_task_stack, sizeof(usb_task_stack),
                       NULL, OS_PRIORITY(3));
  if (ret != OS_OK)
    return -1;

  return HAL_PCD_Start(&hpcd) == HAL_OK ? 0 : -1;
}
