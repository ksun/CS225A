/***************************************************************
 * PrMathDefn.h
 *
 * This files contains math constants and typedef's.
 *
 ****************************************************************/

/*
 * modification history
 *----------------------
 *
 * 10/26/97: K.C. Chang: created.
 */

#ifndef _PrMathDefn_h
#define _PrMathDefn_h

#include <cmath>

#ifndef M_PI
#define M_PI        3.14159265358979323846  // pi
#endif
#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923  // pi/2
#endif
#ifndef M_PI_4
#define M_PI_4      0.78539816339744830962  // pi/4
#endif
 
#define PR_EPSILON         0.00001
#define PR_COS_THRESHHOLD ( 1.0 - PR_EPSILON )

#define PR_THOUSAND        1000
#define PR_MILLION         ( PR_THOUSAND*PR_THOUSAND )
#define PR_BILLION         ( PR_MILLION*PR_THOUSAND )

#define PR_LB_TO_KG        ( 0.45359237 ) //=( 1.0 / 2.2046226 )
#define PR_INCH_TO_METER   ( 0.0254 )

#define PR_GRAVITY_CONSTANT ( 9.81 )

#define PR_BITS_PER_BYTE  8
#define PR_BYTES_PER_WORD ( sizeof( unsigned int ) )

#endif // _PrMathDefn_h
