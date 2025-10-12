#include <string.h>
#include <time.h>

#include "log.h"

// Default log configuration
tt_LogLevel tt_current_log_level = TT_LOG_INFO;
FILE* tt_log_output = NULL;

// Log level strings
static const char* log_level_strings[] = {
    "DEBUG",
    "INFO",
    "WARNING",
    "ERROR"
};

void tt_log_init(tt_LogLevel level, FILE* output) {
    tt_current_log_level = level;
    tt_log_output = output ? output : stderr;
    
    // Initialize default output if not set
    if (!tt_log_output) {
        tt_log_output = stderr;
    }
}

void tt_log_set_level(tt_LogLevel level) {
    if (level >= TT_LOG_DEBUG && level < TT_LOG_NONE) {
        tt_current_log_level = level;
    }
}

void tt_log_set_output(FILE* output) {
    tt_log_output = output ? output : stderr;
}

void tt_log_internal(tt_LogLevel level, const char* level_str, const char* format, va_list args) {
    if (level < tt_current_log_level) {
        return;
    }

    // Use stderr if tt_log_output is not initialized
    FILE* output = tt_log_output ? tt_log_output : stderr;

    // Get current time
    time_t now;
    struct tm* timeinfo;
    char time_str[26];
    
    time(&now);
    timeinfo = localtime(&now);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);

    // Print timestamp and log level
    fprintf(output, "[%s] [%s] ", time_str, level_str);
    
    // Print the actual message
    vfprintf(output, format, args);
    
    // Ensure newline at the end
    if (format[strlen(format) - 1] != '\n') {
        fprintf(output, "\n");
    }
    
    fflush(output);
}

void tt_log_debug(const char* format, ...) {
    va_list args;
    va_start(args, format);
    tt_log_internal(TT_LOG_DEBUG, log_level_strings[TT_LOG_DEBUG], format, args);
    va_end(args);
}

void tt_log_info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    tt_log_internal(TT_LOG_INFO, log_level_strings[TT_LOG_INFO], format, args);
    va_end(args);
}

void tt_log_warning(const char* format, ...) {
    va_list args;
    va_start(args, format);
    tt_log_internal(TT_LOG_WARNING, log_level_strings[TT_LOG_WARNING], format, args);
    va_end(args);
}

void tt_log_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    tt_log_internal(TT_LOG_ERROR, log_level_strings[TT_LOG_ERROR], format, args);
    va_end(args);
}
