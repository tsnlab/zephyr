#pragma once

#include <tickle/tickle.h>

struct CommandData {
    uint8_t joint;
    int32_t angle;
};

extern struct tt_Topic CommandTopic;

int32_t CommandData_encode_size(struct CommandData* request);
int32_t CommandData_encode(struct CommandData* request, uint8_t* payload, const int32_t len);
int32_t CommandData_decode(struct CommandData* request, const uint8_t* payload, const int32_t len, bool is_native_endian);
void CommandData_free(struct CommandData* request);
