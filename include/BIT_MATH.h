/*
 * BIT_MATH.h
 *
 * Bit-level manipulation macros for direct register access.
 * REG  -> the register / variable being modified
 * BIT  -> bit position (0-based)
 */

#ifndef BIT_MATH_H
#define BIT_MATH_H

/* Set a single bit to 1 */
#define SET_BIT(REG, BIT)        ((REG) |=  (1U << (BIT)))

/* Clear a single bit to 0 */
#define CLR_BIT(REG, BIT)        ((REG) &= ~(1U << (BIT)))

/* Toggle a single bit */
#define TOG_BIT(REG, BIT)        ((REG) ^=  (1U << (BIT)))

/* Read the value of a single bit -> returns 0 or 1 */
#define GET_BIT(REG, BIT)        (((REG) >> (BIT)) & 1U)




/* Write a specific value (0 or 1) into a bit */
#define ASSIGN_BIT(REG, BIT, VALUE) \
    ((REG) = ((REG) & ~(1U << (BIT))) | ((VALUE) << (BIT)))

/* Check helpers -> return TRUE/FALSE style results (0 or 1) */
#define IS_BIT_SET(REG, BIT)     ( GET_BIT(REG, BIT) == 1U )
#define IS_BIT_CLEAR(REG, BIT)   ( GET_BIT(REG, BIT) == 0U )


#endif /* BIT_MATH_H */
