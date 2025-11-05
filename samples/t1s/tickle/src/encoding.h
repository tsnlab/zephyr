#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <tickle/hal.h>
#include <tickle/tickle.h>

// Endian checking functions
bool tt_is_native_endian(struct tt_Header* header);
bool tt_is_reverse_endian(struct tt_Header* header);

// Hash function
uint32_t tt_hash_id(const char* type, const char* name);

// Basic encoding/decoding utility functions
void* tt_encode_buffer(void* buffer, uint32_t* tail, uint32_t len);
void* tt_decode_buffer(void* buffer, uint32_t* head, uint32_t tail, uint32_t length);

// String encoding/decoding functions
bool tt_encode_string(void* buffer, uint32_t* tail, uint32_t buffer_size, const char* str);
bool tt_decode_string(void* buffer, uint32_t* head, uint32_t tail, uint16_t* str_len, char** str);
