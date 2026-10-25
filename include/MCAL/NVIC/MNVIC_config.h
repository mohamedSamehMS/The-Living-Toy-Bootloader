/*
 * MNVIC_config.h
 *
 *  Configuration of the MNVIC driver.
 *
 *  Section 1 is the ONE user choice (priority grouping). Section 2
 *  derives the group/sub field limits and shifts from it. Section 3
 *  validates.
 *
 *  Reference: ARM PM0214 (Cortex-M4 Devices Generic User Guide)
 *             §4.2 (SCB / AIRCR), §4.3 (NVIC)
 *
 *  Revision history:
 *    Rev 1 — original (empty file; grouping was runtime state)
 *    Rev 2 — grouping becomes a COMPILE-TIME choice; the runtime
 *            static, its silent no-op window, and the re-interpretation
 *            hazard are all eliminated by design (see §1)
 *
 *  Created on: Aug 1, 2026
 *      Author: dell
 */

#ifndef MNVIC_CONFIG_H_
#define MNVIC_CONFIG_H_

/* =====================================================================
 *  1. Priority grouping  (user choice — ONE value, fixed at build)
 *
 *  The F401 implements 4 priority bits per interrupt (16 levels,
 *  0 = highest). PRIGROUP in AIRCR decides how those 4 bits split
 *  between preemption ("group") priority and sub-priority:
 *
 *      PRIGROUP encoding   group : sub    GROUP_MAX : SUB_MAX
 *      3                   16    : 0      15         : 0
 *      4                   8     : 2      7          : 1
 *      5                   4     : 4      3          : 3
 *      6                   2     : 8      1          : 7
 *      7                   1     : 16     0          : 15
 *
 *  Only GROUP priority causes preemption; sub-priority only orders
 *  same-group exceptions that are ALREADY pending (no nesting).
 *
 *  Why 16/0 (the default below) is right for Project 4:
 *    - the roadmap's priority plan is flat: EXTI buttons > USART
 *      (ESP8266) > timers > SysTick — 16 distinct preemption levels
 *      is far more headroom than that plan ever uses;
 *    - sub-priorities only matter when interrupts must NEST inside
 *      their own group — nothing in the super-loop phases needs it.
 *
 *  Why this is compile-time and NOT runtime anymore (Rev 1 history):
 *    - the old runtime static initialized to 0 — not a valid case of
 *      the grouping switch — so every SetGroupPriority call before
 *      SetGroupsConfig silently did NOTHING, and the IRQ kept its
 *      reset priority 0 (the HIGHEST) instead of the intended one;
 *    - changing grouping at runtime silently re-interprets every IPR
 *      byte already written (raw values stay, meaning changes).
 *      Runtime re-grouping is therefore deliberately NOT supported.
 * ===================================================================== */
#define MNVIC_PRIGROUP_16_0     3
#define MNVIC_PRIGROUP_8_2      4
#define MNVIC_PRIGROUP_4_4      5
#define MNVIC_PRIGROUP_2_8      6
#define MNVIC_PRIGROUP_1_16     7

#define MNVIC_PRIORITY_GROUPING             MNVIC_PRIGROUP_16_0

/* =====================================================================
 *  2. Derived values — DO NOT EDIT
 *
 *  The PRIGROUP encoding IS the bit-split parameter, so the field
 *  geometry falls out of it directly:
 *      group bits = 7 - PRIGROUP
 *      sub bits   = PRIGROUP - 3        (they always sum to 4)
 *      group field occupies IPR bits [7 : 8-GROUP_BITS]
 *          -> group shift = 1 + PRIGROUP
 *      sub field always starts at IPR bit 4
 *          -> sub shift = 4
 *
 *  This one formula reproduces all five cases of the old five-case
 *  switch exactly — verified by the _Static_asserts in program.c.
 * ===================================================================== */
#define MNVIC_GROUP_PRIORITY_BITS     (7u  - MNVIC_PRIORITY_GROUPING)
#define MNVIC_SUB_PRIORITY_BITS       (MNVIC_PRIORITY_GROUPING - 3u)

#define MNVIC_GROUP_PRIORITY_MAX      ((1u << MNVIC_GROUP_PRIORITY_BITS) - 1u)
#define MNVIC_SUB_PRIORITY_MAX        ((1u << MNVIC_SUB_PRIORITY_BITS) - 1u)

#define MNVIC_GROUP_PRIORITY_SHIFT    (1u + MNVIC_PRIORITY_GROUPING)   /* = 8 - GROUP_BITS */
#define MNVIC_SUB_PRIORITY_SHIFT      4u

/* =====================================================================
 *  3. Configuration validation — bad values stop the build
 * ===================================================================== */
#if (MNVIC_PRIORITY_GROUPING < 3) || (MNVIC_PRIORITY_GROUPING > 7)
#error "MNVIC: MNVIC_PRIORITY_GROUPING must be one of MNVIC_PRIGROUP_16_0/8_2/4_4/2_8/1_16 (3..7)"
#endif

#endif /* MNVIC_CONFIG_H_ */
