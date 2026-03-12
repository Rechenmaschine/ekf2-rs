/**
 * px4_platform_common/defines.h — stub for standalone EKF2 build
 *
 * Provides the macros used by EKF2 and the matrix library when building
 * outside of the full PX4 RTOS/NuttX environment.
 */
#pragma once

#include <cmath>
#include <cstdint>

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
# define M_PI_2 1.57079632679489661923
#endif

#ifndef M_TWOPI
# define M_TWOPI (2.0 * M_PI)
#endif

#ifndef M_DEG_TO_RAD
# define M_DEG_TO_RAD (M_PI / 180.0)
#endif

#ifndef M_RAD_TO_DEG
# define M_RAD_TO_DEG (180.0 / M_PI)
#endif

/* Float versions of math constants (PX4 convention) */
#ifndef M_PI_F
# define M_PI_F         3.14159265358979323846f
#endif
#ifndef M_PI_2_F
# define M_PI_2_F       1.57079632679489661923f
#endif
#ifndef M_TWOPI_F
# define M_TWOPI_F      6.28318530717958647692f
#endif
#ifndef M_DEG_TO_RAD_F
# define M_DEG_TO_RAD_F 0.01745329251994329577f
#endif
#ifndef M_RAD_TO_DEG_F
# define M_RAD_TO_DEG_F 57.29577951308232522583f
#endif
#ifndef M_SQRT2_F
# define M_SQRT2_F      1.41421356237309504880f
#endif
#ifndef M_E_F
# define M_E_F          2.71828182845904523536f
#endif
#ifndef M_LN2_F
# define M_LN2_F        0.69314718055994530942f
#endif

/* PX4 floating-point check macros */
#define PX4_ISNAN(x)      (std::isnan(x))
#define PX4_ISFINITE(x)   (std::isfinite(x))
#define PX4_ISINF(x)      (std::isinf(x))
