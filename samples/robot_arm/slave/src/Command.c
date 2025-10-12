#include "Command.h"

#include <tickle/hal.h>
#include <zephyr/sys/byteorder.h>

struct tt_Topic CommandTopic = {
    .name = "CommandTopic",
    .data_size = sizeof(struct CommandData),
    .data_encode_size = (tt_DATA_ENCODE_SIZE)CommandData_encode_size,
    .data_encode = (tt_DATA_ENCODE)CommandData_encode,
    .data_decode = (tt_DATA_DECODE)CommandData_decode,
    .data_free = (tt_DATA_FREE)CommandData_free,
};

int32_t CommandData_encode_size(struct CommandData* data) {
    return sizeof(uint8_t) + sizeof(int32_t);
}

int32_t CommandData_encode(struct CommandData* data, uint8_t* payload, const int32_t len) {
    int32_t encoded = 0;

    if (encoded + sizeof(uint8_t) > len) {
        return -1;
    }

    *(uint8_t*)payload = data->joint;

    encoded += sizeof(uint8_t);
    payload += sizeof(uint8_t);

    if (encoded + sizeof(int32_t) > len) {
        return -1;
    }

    *(int32_t*)payload = sys_cpu_to_be32(data->angle);
    payload += sizeof(int32_t);
    encoded += sizeof(int32_t);

    return encoded;
}

int32_t CommandData_decode(struct CommandData* data, const uint8_t* payload, const int32_t len, bool is_native_endian) {
    int32_t decoded = 0;

    if (decoded + sizeof(uint8_t) > len) {
        return -1;
    }

    data->joint = *(uint8_t*)payload;

    decoded += sizeof(uint8_t);
    payload += sizeof(uint8_t);

    if (decoded + sizeof(int32_t) > len) {
        return -1;
    }

    data->angle = sys_be32_to_cpu(*(int32_t*)payload);

    decoded += sizeof(int32_t);
    payload += sizeof(int32_t);

    return decoded;
}

void CommandData_free(struct CommandData* data) {
    // Do nothing
}
