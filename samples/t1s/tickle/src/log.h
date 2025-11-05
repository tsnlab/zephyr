#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

// Log levels
typedef enum {
    TT_LOG_DEBUG = 0,
    TT_LOG_INFO = 1,
    TT_LOG_WARNING = 2,
    TT_LOG_ERROR = 3,
    TT_LOG_NONE = 4
} tt_LogLevel;

// Log configuration
extern tt_LogLevel tt_current_log_level;
extern FILE* tt_log_output;

// Initialize logging system
void tt_log_init(tt_LogLevel level, FILE* output);

// Set log level
void tt_log_set_level(tt_LogLevel level);

// Set log output
void tt_log_set_output(FILE* output);

// Log functions
void tt_log_debug(const char* format, ...);
void tt_log_info(const char* format, ...);
void tt_log_warning(const char* format, ...);
void tt_log_error(const char* format, ...);

// Internal logging function
void tt_log_internal(tt_LogLevel level, const char* level_str, const char* format, va_list args);

// Convenience macros
#define TT_LOG_DEBUG(...) tt_log_debug(__VA_ARGS__)
#define TT_LOG_INFO(...) tt_log_info(__VA_ARGS__)
#define TT_LOG_WARNING(...) tt_log_warning(__VA_ARGS__)
#define TT_LOG_ERROR(...) tt_log_error(__VA_ARGS__)
