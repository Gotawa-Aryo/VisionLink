# VisionLink

Hand-gesture control for the **STM32N6570-DK**, running on **micro T-Kernel 3.0**.

A camera watches your hands. Two neural networks on the board's Neural-ART accelerator find and track them, and a small classifier on the Cortex-M55 reads the gesture. The board then appears to a computer as an ordinary USB mouse, keyboard and game controller. The host needs no driver and no extra software.

VisionLink has three control schemes in one firmware image. Pick one on the touchscreen at power-on and switch at any time from the on-screen menu.

| Scheme | Purpose | USB Mode |
|:--|:--|:--|
| Drone HMI, assisted | Flight control, one axis at a time | Gamepad |
| Drone HMI, acrobatic | Flight control, palm position is the stick | Gamepad |
| Presenter remote | Slide changes and an on-screen pointer | Mouse and keyboard |

VisionLink started from ST's `x-cube-n6-ai-hand-landmarks` example. It adds the gesture stage, the control schemes, the touch interface and the USB device, and it replaces FreeRTOS with micro T-Kernel 3.0 from the TRON Forum.

> **Warning: OTP fuses involved. Read this before the first run.** At startup, VisionLink sets two one-time fuse bits (word 124, bits 15 and 16) if they are not already set. The function: `Fuse_Programming()` in `Platform/Src/app_fuseprogramming.c` is inherited unchanged from ST's example. They switch the pins for the external flash and PSRAM to 1.8 V high-speed mode, which the STM32N6570-DK needs. Any board that has run an ST N6 AI example already has them set, and then nothing is written. Fuses cannot be cleared, so on a new or custom board, check them first. **This warning is relevant for a custom board that is not STM32N6570-DK; ignore otherwise.**

## Contents

- [Safety](#safety)
- [Hardware](#hardware)
- [Build](#build)
- [Flash](#flash)
- [Run](#run)
- [Control schemes](#control-schemes)
- [User interface](#user-interface)
- [Host integration](#host-integration)
- [Architecture](#architecture)
- [Build options](#build-options)
- [Repository layout](#repository-layout)
- [Troubleshooting](#troubleshooting)
- [Credits and licenses](#credits-and-licenses)

## Safety

VisionLink is experimental software. It is not certified for, and must not be used in, any safety-critical application.

- **Drone control.** Gesture recognition can misread a hand, lose track of it, or stop without warning. If the board loses power or its cable is pulled, the host keeps acting on the last input it received. Practice in a simulator first. When flying a real aircraft, keep a conventional controller or a kill switch within reach, follow local aviation rules, and never fly over people.
- **Presenter mode** sends real mouse and keyboard input to the host computer.
- **OTP fuses.** See the warning above: the firmware can program one-time fuses on a board that does not already have them set.

The software is provided "AS IS", without warranty of any kind, and the authors are not liable for any damage arising from its use. See sections 7 and 8 of the [Apache License 2.0](LICENSE).

## Hardware

| Item | Requirement |
|:--|:--|
| Board | STM32N6570-DK (MB1939) with the MB1854B camera module |
| Power | USB-C charger on **CN6**, able to supply more than 500 mA |
| Host link | USB cable from **CN18** to the computer. It must carry data; many charging cables do not. |
| Programmer | STM32CubeProgrammer, including the `MX66UW1G45G_STM32N6570-DK` external loader |
| Toolchain | STM32CubeIDE 1.16 or later (tested with version 1.19.0), which supplies Arm GNU 13.3 and GNU Make 4.x |

### Power

**The board will not run VisionLink from a computer's USB port due to a power limits.** With the camera and the accelerator both active, the board draws more current than a PC port supplies. It then runs for a few seconds and dies, which looks exactly like a software crash. Use a phone charger on CN6. A PC port is fine for flashing and for the `APP_DIAG` builds described below.

To tell a power failure from a software fault, look at the LED indicators or measure the rail:

```bash
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG
```

A `Voltage` of `0.00 V` means the supply collapsed. `3.30 V` means the board has
power, so look for the fault in software.

### LEDs

| Pattern | Meaning |
|:--|:--|
| LD3 solid orange, LD4 slowly blinking red | **Normal**, including while VisionLink runs |
| LD3 and LD4 both solid red | The supply has collapsed |

## Build

Application can be built using STM32CubeIDE; tested with version 1.19.0. To do the build process follow these steps:

1. Run STM32CubeIDE, then open **File › Import › General › Existing Projects into Workspace**, and select this folder. The project is called **VisionLink**.
2. Open the `Makefile` and verify the location of `SIGNING_TOOL`. Default configuration in the `Makefile` is `SIGNING_TOOL ?= C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_SigningTool_CLI.exe`.
3. Press **Build** (hammer icon).

The build compiles, links, and then **signs** the image, because the boot ROM refuses to start an unsigned application. It calls `STM32_SigningTool_CLI` from the path in `SIGNING_TOOL`; override that on the command line if STM32CubeProgrammer lives elsewhere.

| File | Usage |
|:--|:--|
| `build/VisionLink.elf` | Debugging |
| `build/VisionLink_sign.bin` | Signed image, to program at `0x70100000` |
| `build/VisionLink_sign.hex` | The same image as Intel HEX, carrying its own address |

## Flash

VisionLink lives in the external octo-SPI flash, alongside a first-stage bootloader and the two network weight blobs. Every file below is Intel HEX and carries its own address, so no start address should be manually typed. The prebuilt version is inside the `Binary` folder.

| File | Address | When to program |
|:--|:--|:--|
| `Binary/ai_fsbl.hex` | `0x70000000` | Once, on a new board |
| `Binary/palm_detector_data.hex` | `0x70380000` | Once, or after regenerating the model |
| `Binary/hand_landmark_data.hex` | `0x70580000` | Once, or after regenerating the model |
| `build/VisionLink_sign.hex` | `0x70100000` | After every build |

Application can be flashed to the board using STM32CubeProgrammer; tested with version 2.20.0. To do the flash process follow these steps:

1. Set **BOOT1 (SW1) toward H** (development mode) and **power cycle** the board.
2. Run STM32CubeProgrammer, then connect to the internal ST-Link in `Hot plug` mode. Programming can be done via STM32N6570-DK's USB connector (CN6).
3. Go to `External loaders` menu and select the external loader: `MX66UW1G45G_STM32N6570-DK`.
4. Go to `Erasing & programming` menu. Check the option `Verify programming` and uncheck the option `Run after programming`. Leave the `Start address` empty; hex files carry their own addresses.
5. Flash all these files one-by-one into the flash memory of STM32N6570-DK: `ai_fsbl.hex`, `palm_detector_data.hex`, `hand_landmark_data.hex`, and `VisionLink_sign.hex` (refer to the table above).

If every operation fails with `failed to erase memory`, the bootloader has left
the flash in octal mode. Only a **power cycle** in development mode clears it; a
reset is not enough.

## Run

After all the images are flashed to the board, follow these steps to run the application:

1. Set **BOOT1 toward L** (boot from flash).
2. Connect the charger to CN6, and CN18 to the computer.
3. Power cycle the board.

The board starts in the mode chooser. It sends the host nothing until you choose a scheme, so a hand in view at power-on cannot disturb a simulator or a presentation that is already open.

## Control schemes

Which hand is which comes from its position in the camera image. By default the image is mirrored, so your left hand appears on the left of the screen. The `CAMERA_MIRRORED` build option reverses this.

### Drone HMI, assisted

The default scheme, and the one meant for real flying. A gesture selects **one axis at a time**, and moving the hand trims that axis from where the gesture began. It works like a clutch rather than a stick.

| Hand | Gesture | Movement | Axis |
|:--|:--|:--|:--|
| Left | Closed | Up and down | Throttle |
| Left | Pointer | Left and right | Yaw |
| Right | Closed | Up and down | Pitch |
| Right | Pointer | Left and right | Roll |

- **Throttle stays where you leave it.** It starts at minimum and changes only while a closed left hand moves it. This is what allows hands-off hovering. The other axes spring back to center on release.
- **Movement counts from where the gesture started**, so closing the hand never jolts an axis. One grip changes an axis by at most its full range, so going from minimum to maximum throttle takes two sweeps. This is deliberate.
- Opening the hand, or taking it out of view, releases the axis.

### Drone HMI, acrobatic

Each hand's offset from the center of its half of the screen drives two axes at once, like a physical stick. It responds quickly but is hard to fly accurately, which is why the assisted scheme exists. Be careful when using this mode.

### Presenter remote

The pointer works like lifting a mouse off the desk. It moves while the fist is held and stays put when the hand opens. A swipe is defined by speed, not distance, so a hand resting in the OK pose cannot drift through a deck. A one-second cooldown, shared by both hands, stops a two-handed gesture from changing two slides.

| Hand | Pose | Action | Result |
|:--|:--|:--|:--|
| Right | Fist | Move the hand | The host's pointer follows |
| Right | OK | Swipe right | Next slide (Page Down) |
| Left | OK | Swipe left | Previous slide (Page Up) |

## User interface

The interface is drawn over the live camera image. Use the touchscreen, or the two board buttons when a hand is in front of the panel.

| Input | Action |
|:--|:--|
| Touch | Tap a row or button |
| USER1 | Move the highlight; "next" in the wizard |
| TAMP | Open the menu; activate the highlighted row; go back |

The red **MENU** tab, top left, opens the menu:

| Row | Effect |
|:--|:--|
| Calibrate for simulator | Runs the calibration wizard (drone schemes only) |
| Mode | Switches control scheme. The axes are parked first, so a switch never hands the next scheme a live throttle. |
| Landmarks | Shows or hides the hand skeleton |
| Palm boxes | Shows or hides the palm detector's boxes |
| Resume | Closes the menu |

**Calibration wizard.** Simulators ask you to center the sticks and then sweep them through their range, which the assisted scheme cannot do by hand. The wizard first holds every axis at center, then sweeps all four through 0 → +1 → −1 → 0 over three seconds. Leaving the wizard by any route returns the throttle to minimum.

**Power saving.** After one minute without input, the board suspends the camera and blanks the screen. A touch or a button press wakes it; that input is used only to wake. The board stays awake while a panel is open, or while any axis is away from rest. In the assisted scheme, a held throttle with no hands in view is normal hands-off flight, and sleeping would drop the aircraft. While asleep the board keeps sending the parked axes, so the host never acts on a stale report.

## Host integration

### USB HID device

On CN18 the board enumerates as one composite HID device named **VisionLink**, with three report types:

| Report ID | Device | Contents |
|:--|:--|:--|
| 1 | Mouse | Buttons and relative X/Y, for the presenter pointer |
| 2 | Keyboard | Page Up and Page Down |
| 3 | Gamepad | Four signed 8-bit axes |

All three are always present, because a USB device cannot change its descriptor without reconnecting, and the menu can change scheme at any moment. Each scheme sends only its own reports.

| Gamepad axis | Meaning |
|:--|:--|
| X | Roll |
| Y | Pitch |
| Rx | Yaw |
| Ry | Throttle |

HID Y axes grow downward, so the firmware negates pitch and throttle. The IDs are VID `0x0483` (STMicroelectronics) and PID `0x572B`, which suit development hardware. A product for sale needs its own IDs from the USB Implementers Forum.

## Architecture

```
camera ─► palm detector ─► hand landmarks ─► gesture ─► control scheme ─► USB
          (NPU)            (NPU, per hand)   (CPU)      (CPU)
```

- **Palm detection** finds new hands. It runs every frame while no hand is tracked, every fourth frame while one tracking slot is free, and not at all while both slots are busy.
- **Hand landmarks** run once per tracked hand and give 21 points. Each slot predicts the next region of interest from them, so a tracked hand is followed without detecting it again.
- **Gesture classification** runs on the CPU in microseconds. The drone schemes use Kazuhito Takahashi's keypoint classifier (Open, Close, Pointer). The presenter uses a 19-class HaGRID classifier trained for VisionLink, and acts on Fist and OK. A majority filter stops one wrong frame from firing an action.

| Task | Priority | Job |
|:--|:--|:--|
| `usb` | Highest | Runs the USB stack; the only code that touches it |
| `isp` | High | Camera image-signal-processor updates |
| `nn` | High | Camera frames, both networks, gestures and the control scheme |
| `dp` | Low | Display, touch and buttons, serial output |
| `idle` | Lowest | Measures idle time for the CPU-load figure |

The USB stack is USBX in standalone mode, stepped by the `usb` task alone, so it needs no locking. The rest of the application hands reports to a one-deep mailbox, where the newest report wins.

### Measured timings

Read from the on-screen figures in presenter mode, on the STM32N6570-DK.

| Hands in view | `pd` (ms) | `hl` (ms) | `gs` (µs) |
|:--|--:|--:|--:|
| None | 25 | 0 | 0 |
| One | 5 | 34 | 85 |
| Two | 0 | 60 | 170 |

- **`pd`** is the palm detector, averaged over frames, with frames where it does not run counting as 0. It runs every frame with no hands (25 ms), every fourth frame with one hand (25 ms / 4, shown as 5), and never with two.
- **`hl`** covers everything done per tracked hand, added up over the hands: the GPU2D crop and rotation, the landmark network, post-processing, next-crop prediction and gesture classification. One hand costs about 34 ms and the second adds about 26 ms. This is more than ST's published landmark inference time, because it includes the steps around the inference.
- **`gs`** is the gesture classifier alone, from the CPU cycle counter. Presenter mode runs the HaGRID classifier at about 85 µs per hand. Drone HMI mode runs the lightweight classifier at about 14 µs per hand. It is under 0.3% of the frame time.

`pd` and `hl` are averaged and shown in whole milliseconds, so treat them as ±1 ms.

## Build options

Set these on the `make` command line, then **clean before building**: object
files do not depend on the options, so a stale build keeps the old setting.

| Option | Default | Effect |
|:--|:--|:--|
| `APP_DEFAULT_MODE` | `ASSISTED` | Scheme highlighted in the chooser: `ASSISTED`, `ACROBATIC` or `PRESENTER` |
| `CAMERA_MIRRORED` | `1` | `1`: your left hand appears on the left. `0`: raw sensor view. |
| `PRESENTER` | `ON` | `OFF` leaves out the presenter scheme and its classifier |
| `APP_DIAG` | `0` | `2`: no inference and no camera. `1`: no inference. Both run from a PC port. `0`: complete pipeline. Needa a phone charger. |
| `UTK_TRACE` | `0` | `1` prints the camera startup markers and a heartbeat from the idle task every 2 s |

These parameters use `#ifndef` guards, so `-D` can override them:

| Parameter | Default | Effect |
|:--|:--|:--|
| `HMI_ASSIST_RANGE_FRAC` | 0.25 | Hand travel for a full-range change (assisted), as a fraction of screen width |
| `HMI_RANGE_FRAC` | 0.18 | The same, for the acrobatic scheme |
| `HMI_DEADZONE` | 0.10 | Movement ignored before an axis responds |
| `HMI_SMOOTH_ALPHA` | 0.5 | Smoothing of the axis output |
| `PD_REDETECT_PERIOD` | 4 | Frames between palm detections while a slot is free |
| `UI_SLEEP_MS` | 60000 | Idle time before power saving |
| `PRES_SWIPE_MIN_FRAC` | 0.15 | Swipe distance, as a fraction of screen width |
| `PRES_SWIPE_WINDOW_MS` | 1000 | Time allowed to cover that distance |
| `PRES_COOLDOWN_MS` | 1000 | Minimum time between slide changes |

## Repository layout

```
VisionLink/
├── App/            the application: what VisionLink does
├── Platform/       what it runs on: startup, micro T-Kernel layer, configuration
├── Model/          the two networks: generated code, sources and anchor table
├── Binary/         images to flash
├── ThirdParty/     third-party code from ST and the TRON Forum
├── mks/            build fragments included by the Makefile
├── Makefile
├── README.md
├── LICENSE                   Apache License 2.0, for VisionLink's own code
├── NOTICE                    what is under which license
└── THIRD_PARTY_LICENSES.md   ST's component license table
```

**`App/`** is the code written for VisionLink, plus ST's camera and LCD text helpers that it builds on.

| Path | Contents |
|:--|:--|
| `App/Src/app.c` | Pipeline, display task, camera and tracker |
| `App/Src/hmi.c`, `App/Inc/hmi.h` | Both drone control laws, axis state, serial protocol |
| `App/Src/presenter.c`, `App/Src/hand_pose.c` | Presenter scheme and pose reduction |
| `App/Src/ui.c`, `App/Inc/ui.h` | Touch interface, chooser, menu, wizard, power saving |
| `App/Src/app_mode.c` | The list of schemes and their wording |
| `App/Src/gesture.c`, `App/Src/gesture_hagrid.c` | The two gesture classifiers |
| `App/Inc/gesture_model_data.h`, `App/Inc/gesture_hagrid_model_data.h` | Classifier weights (generated files) |
| `App/Src/ld.c` | Hand landmark post-processing |
| `App/Src/usb_hid.c` | Composite USB HID device |
| `App/Src/app_cam.c` | Camera pipes: one to the display, one to the palm detector |
| `App/Inc/app_config.h` | Application configuration: screen size, camera orientation, cache use |

**`Platform/`** is everything between the application and the hardware.

| Path | Contents |
|:--|:--|
| `Platform/Src/main.c` | Board bring-up and micro T-Kernel startup |
| `Platform/Src/os_utk.c`, `Platform/Inc/os_utk.h` | The thin micro T-Kernel layer the application uses |
| `Platform/Src/utk_bsp.c` | HAL tick, SysTick restart and CPU-load timer for micro T-Kernel |
| `Platform/Src/nema_hal_utkernel.c` | GPU2D (NemaGFX) glue for micro T-Kernel |
| `Platform/Src/ll_aton_osal_utkernel.c` | Neural-ART runtime glue for micro T-Kernel |
| `Platform/Src/stm32n6xx_it.c` | Interrupt handlers |
| `Platform/Src/app_fuseprogramming.c` | The OTP check described at the top of this file |
| `Platform/Inc/` | HAL, BSP, camera, ISP, NemaGFX and USBX configuration headers |
| `Platform/Gcc/` | Linker script and the C-library glue (console, syscalls, malloc lock) |

**`ThirdParty/`** holds third-party code, kept apart so it can be updated or diffed against upstream.

| Path | Contents |
|:--|:--|
| `ThirdParty/STM32Cube_FW_N6/` | HAL, CMSIS, board support, USBX, LCD utility |
| `ThirdParty/AI_Runtime/` | Neural-ART NPU runtime |
| `ThirdParty/Camera_Middleware/` | Camera middleware, sensor drivers and ISP |
| `ThirdParty/NemaGFX/` | GPU2D graphics library |
| `ThirdParty/screenl/`, `ThirdParty/ipl/` | ST's screen and image-processing libraries |
| `ThirdParty/lib_vision_models_pp/`, `ThirdParty/ai-postprocessing-wrapper/` | Network post-processing |
| `ThirdParty/touchscreen/` | GT911 touchscreen driver, from STM32Cube_FW_N6 V1.2.0 |
| `ThirdParty/mtk3_bsp2/` | micro T-Kernel 3.0 and its STM32Cube board support (TRON Forum) |

`Model/generate-n6-model.sh` regenerates the two networks with ST Edge AI
(`stedgeai`). It rewrites the `.c`, `_ecblobs.h` and `_data.hex` files in
`Model/`, and leaves `pd_anchors.c` alone. Copy the new `_data.hex` files to
`Binary/` before flashing.

## Troubleshooting

| Symptom | Likely cause |
|:--|:--|
| Runs for a few seconds, then dies; both LEDs red | Not enough power. Use a charger on CN6. |
| `failed to erase memory` when flashing | Flash left in octal mode. Power cycle in development mode. |
| `Unable to get core ID` | Normal in boot-from-flash mode, where the boot ROM closes the debug port |
| Host sees no USB device | The CN18 cable carries no data, switch to a cable that can carry data |
| Left and right hands swapped | Change `CAMERA_MIRRORED`, then clean and rebuild |

## Author information
- **Gotawa Aryo Prakoso, Hardware and Embedded Engineer**. Embedded system work: µT-Kernel 3.0 firmware on the STM32N6570-DK, drivers, system integration.
- **Gita Bangun Prakoso, Software and AI Engineer**. AI model work: evaluation of the current model; also exploration, comparison, and retraining of the hand landmark model in Python.

## Credits and licenses

| Component | Source | License |
|:--|:--|:--|
| Palm detection and hand landmark models | Google MediaPipe Hands, via the PINTO model zoo (Katsuya Hyodo), quantized by STMicroelectronics | Apache-2.0 |
| Keypoint gesture classifier | Kazuhito Takahashi, `hand-gesture-recognition-using-mediapipe` | Apache-2.0 |
| HaGRID presenter classifier | Trained for VisionLink on the HaGRID dataset (Kapitanov et al.) | CC BY-SA 4.0 (weights file only) |
| Application base, drivers and middleware | STMicroelectronics `x-cube-n6-ai-hand-landmarks` and STM32Cube_FW_N6 | ST SLA0044 or BSD-3-Clause, as marked per file; see `THIRD_PARTY_LICENSES.md` |
| micro T-Kernel 3.0 and mtk3_bsp2 | TRON Forum | T-License 2.2 |
| USBX | Microsoft, as distributed in STM32Cube_FW_N6 | Microsoft Software License Terms for Azure RTOS |

VisionLink's own code is licensed under the [Apache License 2.0](LICENSE). The repository also contains code from the parties above, and each part keeps its own license; the file header says which applies. See [NOTICE](NOTICE) for the full list and [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) for ST's components. ST's code may only be used with ST microcontrollers (SLA0044).
