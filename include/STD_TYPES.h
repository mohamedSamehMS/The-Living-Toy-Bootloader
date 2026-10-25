/*
 * STD_TYPES.h
 *
 * Standard portable type definitions for embedded C projects (AVR / ARM).
 * Include this at the top of any driver/module that needs fixed-width types.
 */

#ifndef STD_TYPES_H
#define STD_TYPES_H

/* ---------------- Unsigned integer types ---------------- */
typedef unsigned char          u8;   /* 0 .. 255 */
typedef unsigned short int     u16;  /* 0 .. 65535 */
typedef unsigned long int      u32;  /* 0 .. 4294967295 */
typedef unsigned long long int u64;  /* 0 .. 18446744073709551615 */

/* ---------------- Signed integer types ------------------- */
typedef signed char            s8;   /* -128 .. 127                 */
typedef signed short int       s16;  /* -32768 .. 32767             */
typedef signed long int        s32;  /* -2147483648 .. 2147483647   */
typedef signed long long int   s64;

/* ---------------- Floating point types -------------------- */
typedef float                  f32;
typedef double                 f64;
typedef long double            f128;

/* ---------------- Boolean ---------------------------------- */
typedef u8                     bool_t;

#ifndef FALSE
#define FALSE   (0u)
#endif

#ifndef TRUE
#define TRUE    (1u)
#endif

/* ---------------- NULL pointer ------------------------------ */
#ifndef NULL
#define NULL    ((void *)0)
#endif

/* ---------------- Standard function return type -------------
 * Use this as the return type for driver/API functions instead
 * of int, so error handling is consistent across all modules.
 */
typedef enum {
    STD_TYPES_OK  = 0,
    STD_TYPES_NOK = 1
} Std_ReturnType;
#endif /* STD_TYPES_H */
