#pragma once

#define tt_SECOND 1000000000ULL
#define tt_MILLISECOND 1000000ULL
#define tt_MICROSECOND 1000ULL

#define tt_NODE_CYCLE tt_MILLISECOND                   // nanosecond
#define tt_NODE_UPDATE_INTERVAL (10 * tt_SECOND)       // nanoseond  TODO: Temporary value for debugging
#define tt_NODE_TX_INTERVAL tt_MILLISECOND             // nanoseond
#define tt_RELIABLE_DEADLINE 0                         // nanosecond, 0 is auto
#define tt_RELIABLE_RETRY 3                            // count
#define tt_CALL_RETRY_INTERVAL (5 * tt_MILLISECOND)    // Default value
#define tt_CALL_RETRY_COUNT 3                          // count
#define tt_SERVER_CACHE_TIMEOUT (100 * tt_MILLISECOND) // (Client server latency) * (CALL_RETRY_COUNT + 1)

#define tt_MAX_ENDPOINT_COUNT 256    // Maximum number of endpoints (data or services)
#define tt_MAX_NAME_LENGTH 32        // Maximum length of endpoint name
#define tt_MAX_STRING_LENGTH 65535   // Maximum length of string
#define tt_MAX_BUFFER_LENGTH 1480    // TX buffer size
#define tt_MAX_SCHEDULER_LENGTH 128  // Scheduling queue
#define tt_MAX_SERVER_CACHE_COUNT 64 // >= # of client

#define _tt_NODE_ADDRESS "0.0.0.0"
#define _tt_NODE_PORT 8282
#define _tt_NODE_BROADCAST "255.255.255.255"

struct _tt_Config {
    char* addr;
    int port;
    char* broadcast;
};

extern struct _tt_Config _tt_CONFIG;
