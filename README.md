<picture>
  <source media="(prefers-color-scheme: dark)" srcset="Docs/images/iti-logo-dark.png">
  <img src="Docs/images/iti-logo-light.png" alt="ITI - Information Technology Institute" width="260">
</picture>

*Built during the ITI ARM Embedded Systems Track — Summer Training*

# The Living Toy — FOTA Bootloader

### A 16 KB staged-image bootloader that installs firmware updates without ever leaving the device unbootable

![Platform](https://img.shields.io/badge/platform-STM32F401CCU6-blue) ![Footprint](https://img.shields.io/badge/footprint-8.7%20KB%20of%2016%20KB-informational) ![Dev](https://img.shields.io/badge/development-register--level%2C%20no%20vendor%20HAL%2FLL-orange) ![Status](https://img.shields.io/badge/status-shipped-brightgreen) ![License](https://img.shields.io/badge/license-MIT-green)

The bootloader half of The Living Toy's over-the-air updates. The application receives and verifies a new firmware image and parks it in a staging sector; on the next reset this bootloader re-verifies it from scratch, installs it over the running image, checks the result, and boots it. If anything looks wrong at any step, the old application is left alone.

Companion to the application repo: **[The-Living-Toy-STM32](https://github.com/mohamedSamehMS/The-Living-Toy-STM32)**.

| | |
|---|---|
| **Platform** | STM32F401CCU6 (WeAct Black Pill) · ARM Cortex-M4 · 16 MHz HSI (reset default) |
| **Version** | FOTA BOOTLOADER, V1 |
| **Home** | Sector 0, `0x08000000`–`0x08003FFF` (16 KB), enforced by the linker script |
| **Footprint** | 8,664 B of 16,384 B used — 7,720 B free |
| **Development** | Register-level drivers, no vendor HAL/LL calls (RM0368 + PM0214 only) |
| **Training program** | ITI — Information Technology Institute, ARM Embedded Systems track |
| **Author** | Mohamed Sameh |

## Table of Contents

1. [Overview](#1-overview)
2. [Where It Fits](#2-where-it-fits)
3. [The Boot Flow](#3-the-boot-flow)
4. [The Trailer](#4-the-trailer)
5. [Power-Cut Safety](#5-power-cut-safety)
6. [The Status LED](#6-the-status-led)
7. [Deliberate Absences](#7-deliberate-absences)
8. [Drivers](#8-drivers)
9. [Footprint and Build](#9-footprint-and-build)
10. [History](#10-history)
11. [Repository Structure](#11-repository-structure)
12. [Known Limitations](#12-known-limitations)
13. [Related Projects](#13-related-projects)
14. [License](#14-license)

---

## 1. Overview

A firmware update on a microcontroller has one dangerous moment: the instant the old image is erased and the new one is not yet complete. This bootloader exists to make that moment survivable.

Its whole job fits in one sentence: **on reset, if a complete, verified image is waiting in staging, install it into the application region and boot it; otherwise boot the application that is already there.**

It does not download anything and it has no communication code. Receiving firmware over the network is the application's job. By the time this bootloader runs, the image is already sitting in flash.

*(Diagram: [`Docs/images/architecture-diagram.svg`](Docs/images/architecture-diagram.svg))*

---

## 2. Where It Fits

| Region | Sectors | Address | Size | Contents |
|---|---|---|---|---|
| Bootloader | S0 | `0x08000000` | 16 KB | This project |
| Application | S1–S4 | `0x08004000` | 112 KB | The running image (16 + 16 + 16 + 64 KB) |
| Staging | S5 | `0x08020000` | 128 KB | A downloaded image plus its 32-byte trailer |

The update path, end to end:

1. The application's FOTA receiver writes the new image into sector 5, checking every frame's CRC and sequence number.
2. It verifies the whole staged image, then writes the trailer, with the **magic word last** — that word is the "an update is ready" marker.
3. The application resets the MCU.
4. **This bootloader** sees the magic word, re-verifies everything, installs the image, and boots it.

The application is linked to run from `0x08004000` and relocates its vector table there, so the bootloader and the application never overlap.

---

## 3. The Boot Flow

```text
reset
  └─ PC13 as output, LED off
  └─ read sector-5 bounds ──────────────── fail ──► slow-blink halt
  └─ magic word at the staging trailer?
        no  ─────────────────────────────────────────────┐
        yes                                              │
          └─ trailer CRC, version, length                │
          └─ whole-image CRC (re-verified HERE)          │
          └─ staged stack pointer / reset vector sane?   │
                any failure ─► erase sector 5 ───────────┤
          └─ LED on                                      │
          └─ erase S1..S4 → program image → CRC the      │
             installed app ──── any failure ─► fast-blink halt
          └─ erase sector 5  (the commit: clears the magic)
          └─ LED off                                     │
  ◄──────────────────────────────────────────────────────┘
  └─ app stack pointer and reset vector valid? ─ no ─► slow-blink halt
  └─ set VTOR = 0x08004000, load MSP, branch to the app's reset handler
```

Two details worth knowing:

- **Trust nothing, including our own receiver.** The whole staged image is CRC-checked again here, and the staged stack pointer and reset vector are checked *before* the old app is erased. A file that has a valid CRC but is not an application is declined, not installed over a working app.
- **The commit is the last act.** Erasing sector 5 clears the magic word. That happens only after the new app is written and its CRC verified. The marker is cleared by *erasing*, not by programming zero over it: the flash driver's erased-state guard rejects programming a non-erased word, and the F401 has no sub-sector erase.

On a successful install the bootloader jumps straight to the new app. It does not reset first.

---

## 4. The Trailer

The trailer is the last 32 bytes of sector 5 (`0x0803FFE0`–`0x0803FFFF`). The application's receiver writes it, and the layout matches this bootloader's reader exactly.

| Offset | Word | Field | How the bootloader uses it |
|---:|---:|---|---|
| +0 | 0 | Magic `0x464F5441` ("FOTA") | Its presence means an update is pending |
| +4 | 1 | Trailer version = `2` | Must match |
| +8 | 2 | Application version | Read but not used — the "strictly newer" policy lives in the application |
| +12 | 3 | Image length (bytes) | Must be at least 8 and at most 112 KiB |
| +16 | 4 | Whole-image CRC | Checked over the length rounded up to a 4-byte boundary |
| +20 | 5 | Trailer CRC | Covers bytes +4 to +19 (words 1–4) |
| +24 / +28 | 6–7 | Reserved (`0xFFFFFFFF`) | Not checked |

CRC: hardware CRC unit, CRC-32/MPEG-2 style (polynomial `0x04C11DB7`, initial value `0xFFFFFFFF`, no reflection, no final XOR). The self-test constant `0xAE24E09D` is the CRC of the ASCII string `123456789` zero-padded to 12 bytes, because the hardware unit consumes whole 32-bit words; the textbook value for the unpadded 9 bytes is `0x0376E6E7`.

**An image is accepted only if** the trailer checks out, the whole-image CRC matches, the staged stack pointer lies in SRAM (`0x20000000`–`0x20010000`) and is 8-byte aligned, and the staged reset vector lies inside the application region (`0x08004000` up to, not including, `0x08020000`) with the Thumb bit set. An oversized image fails the length check and is never installed.

---

## 5. Power-Cut Safety

The staged image and its magic word stay untouched until the new application has been written and verified. That is what makes every interruption recoverable:

| Power lost… | What the flash holds | What the next boot does |
|---|---|---|
| While the app is downloading to staging | No magic word yet | Boots the old app |
| After the magic word, before any erase | Old app intact, staging intact | Re-commits (the install is idempotent) |
| During the erase or programming of S1–S4 | App region blank or partial; staging and magic intact | Re-commits from staging |
| After programming, before verification | Same | Re-commits |
| During the final sector-5 erase | New app already written and CRC-verified | Boots the new app, or re-commits once more |
| After the sector-5 erase | New app, no magic | Boots the new app |

While S1–S4 are being rewritten the application itself is *not* bootable. That is by design, because the bootloader and the staged copy are both intact, so the device recovers by re-running the install. The source records that the only unrecoverable states need **two independent failures**. Treat that as the design argument recorded in the source, not as a hardware guarantee.

The bootloader also cannot damage itself: the flash driver's writable window is sectors 1–5, so sector 0 is outside it by construction.

---

## 6. The Status LED

The only output is the board LED on PC13 (active low).

| Signal | Meaning | Halts? |
|---|---|---|
| Off | Normal — the happy path never lights it | — |
| Steady on | An update is being installed, roughly 2–4 s; the board is frozen during flash erase (single-bank flash stalls the CPU) | No |
| Fast blink (80 ms on / 80 ms off) | The install failed: erase, program, or post-write CRC. The staged image is still there, so a power cycle retries the install | Until power cycle |
| Slow blink (300 ms on / 300 ms off) | The sector-5 lookup failed, or the application's reset vectors are invalid and no update is pending ("flash me an app") | Forever — recovery is a reflash over SWD |

---

## 7. Deliberate Absences

Each absence is a safety choice recorded in the source:

- **No interrupts and no SysTick.** The NVIC is all-disabled from reset and the bootloader never arms a source, so there is nothing to init and nothing to storm.
- **No UART.** The bootloader is silent by design; SWD is always available for recovery.
- **No `MRCC_Init()`.** It runs on the reset-default 16 MHz HSI, which is exactly what the flash driver's compile-time timeout derivation assumes.
- **No staging erase on the normal path.** The roughly 2.3 s sector erase is paid once per update (and once, self-healing, after a bad staged image), never on an ordinary boot.

---

## 8. Drivers

Register-level MCAL drivers, written from the STM32F401 reference manual (RM0368) and ARM's PM0214. They are the same drivers as in the application repo.

| Driver | Role in the bootloader |
|---|---|
| MRCC | Enables the GPIOC and CRC clocks |
| MGPIO | Configures and drives PC13 |
| MFLS | Sector bounds, sector erase, word programming — writable window sectors 1–5 only |
| MCRC | Hardware CRC-32 for the trailer, the staged image and the installed image |
| MNVIC | Sets the vector table base (VTOR) just before the jump |
| MEXTI, MSPI, MSYSTICK, MTIM, MUSART | **Not used at run time.** Kept so the vector table's interrupt symbols keep their handlers and the image stays byte-for-byte the one that was flashed and tested. Together they account for roughly 2 KB of flash and RAM in the map file |

The driver sources share their history and comments with the application repo, so a few comments still mention application-side hardware.

---

## 9. Footprint and Build

From the linker map (`.text` includes read-only data):

| Section | Size |
|---|---|
| Vector table | 1,012 B |
| `.text` | 7,039 B |
| `.data` (load image in flash) | 568 B |
| `.bss` (RAM) | 332 B |
| **Flash used** | **8,664 B of 16,384 B (7,720 B free)** |

If it ever outgrows 16 KB the **build breaks** instead of the bootloader silently spilling into the application region — the linker's `FLASH` region is exactly 16 KB.

**Build**

1. Import into Eclipse CDT (`File → Import → Existing Projects into Workspace`; `.project` and `.cproject` are committed) and build — Eclipse regenerates the managed makefiles under `Debug/`, which is git-ignored.
2. Compiler: `arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -ffunction-sections -fdata-sections -ffreestanding -std=gnu11 -Wall -Wextra`.
3. Link: `-T mem.ld -T libs.ld -T sections.ld -nostartfiles --gc-sections --specs=nano.specs`; post-build produces the `.hex` and prints the size.
4. Flash the **bootloader** to `0x08000000` over SWD (ST-Link). The application is flashed separately and links at `0x08004000`.

**Recorded results (bench logs, not reproducible from the source alone):** the bootloader has installed 8+ images without ever being reflashed.

---

## 10. History

The drivers carry their own revision history in their headers, from the MRCC clock driver (17 July 2026) through GPIO, SysTick, NVIC, EXTI, SPI and USART in July and August, to the timer and CRC drivers in mid-September 2026. See the application repo for the full chronology.

Bootloader-specific design correction, recorded in `main.c`: the earlier sketch cleared the "update pending" marker by programming zero over it. The flash driver's erased-state guard rejects programming a non-erased word, so the marker is now cleared by erasing sector 5 — which is also why that erase is the last step of the commit.

---

## 11. Repository Structure

```text
the-living-toy-bootloader/
├── include/
│   ├── BIT_MATH.h, STD_TYPES.h, stm32f4xx_hal_conf.h
│   └── MCAL/           # 4-file pattern (config/private/interface) per driver
│       └── CRC/ EXTI/ FLS/ GPIO/ NVIC/ RCC/ SPI/ SYSTICK/ TIM/ USART/
├── src/
│   ├── main.c                       # the whole bootloader
│   ├── MCRC_program.c, MFLS_program.c, MGPIO_program.c, MNVIC_program.c, MRCC_program.c
│   ├── MEXTI_program.c, MSPI_program.c, MSYSTICK_program.c, MTIM_program.c, MUSART_program.c
│   └── MNVIC_interface.h
├── system/             # template: startup, vector table, newlib stubs, CMSIS
├── ldscripts/          # mem.ld (FLASH = 16 KB), libs.ld, sections.ld
├── Docs/images/        # diagrams and logos
└── Debug/              # build output (Eclipse managed makefiles, git-ignored)
```

---

## 12. Known Limitations

- **Integrity, not authenticity.** Images are protected by CRC-32 and sanity checks, not by a signature or encryption. The trust boundary is the application's encrypted link to the update server.
- **Slow blink never recovers on its own.** With no valid application and no staged update, the bootloader halts and needs a reflash over SWD.
- **No application CRC on a normal boot.** Only the application's stack pointer and reset vector are checked before the jump.
- **Feedback is the LED only.** There is no UART or display output; the toy's download progress screen runs in the application, before the reset.
- **The CPU is frozen during an install** (roughly 2–4 s of flash erase and programming on single-bank flash).
- **The final sector-5 erase result is not checked.** If it failed, the next boot simply re-installs the same image — harmless, but wasteful.
- **No downgrade policy here.** The bootloader reads the application version but does not use it; the "strictly newer" rule is enforced by the application before an image is ever staged.
- **Power-cut safety is a design argument,** documented in the source and summarised in §5, not an exhaustive hardware guarantee.

---

## 13. Related Projects

- [The-Living-Toy-STM32](https://github.com/mohamedSamehMS/The-Living-Toy-STM32) — the application: four games, online multiplayer, and the FOTA receiver that stages images for this bootloader
- [GamingHub_BackendServer](https://github.com/mohamedSamehMS/GamingHub_BackendServer) — the cloud referee and update source
- [AVR-Driver-Library](https://github.com/mohamedSamehMS/AVR-Driver-Library) / [Fingerprint-Security-Control-System](https://github.com/mohamedSamehMS/Fingerprint-Security-Control-System) — prior ITI AVR-track driver work and graduation project

## 14. License

MIT — see [LICENSE](LICENSE).

---

*Mohamed Sameh — ARM Embedded Systems Track (Summer Training), Information Technology Institute (ITI)*
