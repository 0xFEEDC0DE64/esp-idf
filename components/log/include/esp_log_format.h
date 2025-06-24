/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @cond */

#define ESP_LOG_STRINGIFY2(x) #x
#define ESP_LOG_STRINGIFY(x) ESP_LOG_STRINGIFY2(x)

// For backward compatibility (these macros are not used in the log v2).
#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " %s:" ESP_LOG_STRINGIFY(__LINE__) " (%" PRIu32 ") %s: " format LOG_RESET_COLOR "\n", (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)
#define _ESP_LOG_DRAM_LOG_FORMAT(letter, format)  DRAM_STR(#letter " %s: " format "\n")
#define LOG_SYSTEM_TIME_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " (%s) %s: " format LOG_RESET_COLOR "\n"
/** @endcond */

#ifdef __cplusplus
}
#endif
