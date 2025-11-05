#include <tickle/hal.h>
#include <tickle/tickle.h>
#include "encoding.h"

// Endian checking functions
bool tt_is_native_endian(struct tt_Header *header)
{
	return header->magic_value == NATIVE_MAGIC_VALUE;
}

bool tt_is_reverse_endian(struct tt_Header *header)
{
	return header->magic_value == REVERSE_MAGIC_VALUE;
}

// Hash function
uint32_t tt_hash_id(const char *type, const char *name)
{
	uint8_t type_len = _tt_strnlen(type, tt_MAX_NAME_LENGTH);
	uint8_t name_len = _tt_strnlen(name, tt_MAX_NAME_LENGTH);

	uint32_t hash = 0;
	((uint8_t *)&hash)[1] = type_len;
	((uint8_t *)&hash)[3] = name_len;

	int count = (int)(type_len / sizeof(uint32_t));
	for (int i = 0; i < count; i++) {
		hash += ((const uint32_t *)type)[i];
	}

	size_t rest = type_len % sizeof(uint32_t);
	if (rest > 0) {
		uint32_t tail = 0;
		size_t offset = count * sizeof(uint32_t);
		_tt_memcpy(&tail, type + offset, rest);
		hash += tail;
	}

	count = (int)(name_len / sizeof(uint32_t));
	for (int i = 0; i < count; i++) {
		hash += ((const uint32_t *)name)[i];
	}

	rest = name_len % sizeof(uint32_t);
	if (rest > 0) {
		uint32_t tail = 0;
		size_t offset = count * sizeof(uint32_t);
		_tt_memcpy(&tail, name + offset, rest);
		hash += tail;
	}

	return hash;
}

// Basic encoding/decoding utility functions
void *tt_encode_buffer(void *buffer, uint32_t *tail, uint32_t len)
{
	uint8_t *buf = (uint8_t *)buffer + *tail;
	*tail += len;
	return buf;
}

void *tt_decode_buffer(void *buffer, uint32_t *head, uint32_t tail, uint32_t length)
{
	if (*head + length > tail) {
		return NULL;
	}

	void *p = (uint8_t *)buffer + *head;
	*head += length;
	return p;
}

// String encoding/decoding functions
bool tt_encode_string(void *buffer, uint32_t *tail, uint32_t buffer_size, const char *str)
{
	size_t str_len = _tt_strnlen(str, tt_MAX_STRING_LENGTH) + 1; // including '\0'

	// str_len always >= 0

	if (*tail + sizeof(uint16_t) + str_len >= buffer_size) {
		return false;
	}

	*(uint16_t *)((uint8_t *)buffer + *tail) = str_len;
	*tail += sizeof(uint16_t);

	_tt_memcpy((uint8_t *)buffer + *tail, str, str_len);
	*tail += str_len;

	return true;
}

bool tt_decode_string(void *buffer, uint32_t *head, uint32_t tail, uint16_t *str_len, char **str)
{
	if (*head + sizeof(uint16_t) > tail) {
		return false;
	}

	*str_len = *(uint16_t *)((uint8_t *)buffer + *head);
	*head += sizeof(uint16_t);

	if (*head + *str_len > tail) {
		return false;
	}

	*str = (char *)((uint8_t *)buffer + *head);
	*head += *str_len;

	return true;
}
