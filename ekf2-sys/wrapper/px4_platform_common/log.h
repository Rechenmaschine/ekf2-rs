#pragma once

// Standalone no_std builds do not provide PX4 logging backends.
// Keep the macros available but compile them away.
#define PX4_DEBUG(...) do { } while (0)
#define PX4_INFO(...)  do { } while (0)
#define PX4_WARN(...)  do { } while (0)
#define PX4_ERR(...)   do { } while (0)
