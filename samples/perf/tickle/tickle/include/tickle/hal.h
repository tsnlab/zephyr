#pragma once

#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Platform detection macros
#if defined(__linux__)
#define TT_PLATFORM_LINUX
#define TT_PLATFORM_NAME "linux"
#elif defined(__ZEPHYR__)
#define TT_PLATFORM_ZEPHYR
#define TT_PLATFORM_NAME "zephyr"
#else
#define TT_PLATFORM_GENERIC
#define TT_PLATFORM_NAME "generic"
#endif

#define _tt_bswap_16(x) bswap_16((x))
#define _tt_bswap_32(x) bswap_32((x))
#define _tt_bswap_64(x) bswap_64((x))
#define _tt_strnlen(s, maxlen) strnlen((s), (maxlen))
#define _tt_strncmp(s1, s2, n) strncmp((s1), (s2), (n))
#define _tt_malloc(size) malloc((size))
#define _tt_memcpy(dest, src, n) memcpy((dest), (src), (n))
#define _tt_memmove(dest, src, n) memmove((dest), (src), (n))
#define _tt_free(ptr) free((ptr))

// Memory alignment macros
#define ALIGN(n) (((n) + 4 - 1) & ~(4 - 1)) // 4 bytes alignment
#define ROUNDUP(n) ALIGN((n) + 4 - 1)       // 4 bytes roundup

#define NATIVE_MAGIC_VALUE (((uint16_t)'T' << 8) | 'K')
#define REVERSE_MAGIC_VALUE (((uint16_t)'K' << 8) | 'T')

enum tickle_error {
    tt_ERROR_NONE = 0,
    tt_CANNOT_CREATE_SOCK = -3,
    tt_CANNOT_SET_REUSEADDR = -4,
    tt_CANNOT_SET_BROADCAST = -5,
    tt_CANNOT_SET_TIMEOUT = -6,
    tt_CANNOT_BIND_SOCKET = -7,
};

struct tt_Node;
struct tt_Header;

// Platform-specific HAL structure inclusion
#if defined(TT_PLATFORM_LINUX)
#include <tickle/hal_linux.h>
#elif defined(TT_PLATFORM_ZEPHYR)
#include <tickle/hal_zephyr.h>
#elif defined(TT_PLATFORM_GENERIC)
#include <tickle/hal_generic.h>
#endif

// Network functions
int32_t tt_get_node_id();
int32_t tt_bind(struct tt_Node* node);
void tt_close(struct tt_Node* node);
int32_t tt_send(struct tt_Node* node, const void* buf, size_t len);
int32_t tt_receive(struct tt_Node* node, void* buf, size_t len, uint32_t* ip, uint16_t* port);
