/*
 * MCRC_config.h
 *
 *  Configuration of the MCRC driver.
 *
 *  Section 1: known-answer test constant.
 *
 *  Reference: RM0368 Rev 6, Ch 4 (CRC), §4.4
 *
 *  Revision history:
 *    Rev 1 — first release (FOTA-03 spec)
 *
 *  Created on: Sep 15, 2026
 *      Author: dell
 */

#ifndef MCRC_CONFIG_H_
#define MCRC_CONFIG_H_

/* =====================================================================
 *  1. Known-answer test constant
 *
 *  MCRC_KAT_EXPECTED 0xAE24E09D is the CRC-32/MPEG-2 (poly
 *  0x04C11DB7, init 0xFFFFFFFF, no reflection, no final XOR) of the
 *  ASCII string 123456789 zero-padded to 12 bytes; the hardware unit
 *  consumes whole 32-bit words. The textbook value for the unpadded
 *  9-byte string is 0x0376E6E7.
 * ===================================================================== */
#define MCRC_KAT_EXPECTED  		0xAE24E09DUL
#define MCRC_KAT_STRING         "123456789"
#define MCRC_KAT_LENGTH         9u

#endif /* MCRC_CONFIG_H_ */
