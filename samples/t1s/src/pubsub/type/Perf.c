#include "Perf.h"

#include <tickle/hal.h>
#include <zephyr/sys/byteorder.h>

struct tt_Topic PerfTopic = {
	.name = "PerfTopic",
	.data_size = sizeof(struct PerfData),
	.data_encode_size = (tt_DATA_ENCODE_SIZE)PerfData_encode_size,
	.data_encode = (tt_DATA_ENCODE)PerfData_encode,
	.data_decode = (tt_DATA_DECODE)PerfData_decode,
	.data_free = (tt_DATA_FREE)PerfData_free,
};

int32_t PerfData_encode_size(struct PerfData *data)
{
	return sizeof(uint32_t) + sizeof(uint64_t);
}

int32_t PerfData_encode(struct PerfData *data, uint8_t *payload, const int32_t len)
{
	int32_t encoded = 0;

	if (encoded + sizeof(uint32_t) > len) {
		return -1;
	}

	*(uint32_t *)payload = sys_cpu_to_be32(data->id);

	encoded += sizeof(uint32_t);
	payload += sizeof(uint32_t);

	if (encoded + sizeof(uint64_t) > len) {
		return -1;
	}

	*(uint64_t *)payload = sys_cpu_to_be64(data->timestamp);
	encoded += sizeof(uint64_t);
	payload += sizeof(uint64_t);

	return encoded;
}

int32_t PerfData_decode(struct PerfData *data, const uint8_t *payload, const int32_t len,
			   bool is_native_endian)
{
	int32_t decoded = 0;

	if (decoded + sizeof(uint32_t) > len) {
		return -1;
	}

	data->id = sys_be32_to_cpu(*(uint32_t *)payload);

	decoded += sizeof(uint32_t);
	payload += sizeof(uint32_t);

	if (decoded + sizeof(uint64_t) > len) {
		return -1;
	}

	data->timestamp = sys_be64_to_cpu(*(uint64_t *)payload);

	decoded += sizeof(uint64_t);
	payload += sizeof(uint64_t);

	return decoded;
}

void PerfData_free(struct PerfData *data)
{
	// Do nothing
}
