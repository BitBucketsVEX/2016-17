// VEX Team 5485 Macros And Other Functions

// Only include this once per scope to avoid duplicate definition warnings
#ifndef CONVENIENT_MACROS_H
#define CONVENIENT_MACROS_H

// Unit conversion
#define IN_2_M	(0.0254)
#define RAD_2_DEG	(180.0/PI)
#define DEG_2_RAD (PI/180.)

// Convenient macros
#define EVER (;;)
#define LENGTH(x) (sizeof(x)/sizeof(x[0]))

// Note macros are NOT the best way to do this
// but since this is C and not C++ this is
// the only type-independent way to do this
#define MIN(x,y) ((x >= y)? x : y)
#define MAX(x,y) ((x <= y)? x : y)
#define BOUND(x,a,b) (MAX(MIN(x, a), b))

#define SIGN(x) ((x != 0) ? x / abs(x) : 0)

#endif
