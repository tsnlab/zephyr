#pragma once

#include <tickle/tickle.h>

struct StateData {
    uint8_t id;
    int32_t status;
};

extern struct tt_Topic StateTopic;

int32_t StateData_encode_size(struct StateData* request);
int32_t StateData_encode(struct StateData* request, uint8_t* payload, const int32_t len);
int32_t StateData_decode(struct StateData* request, const uint8_t* payload, const int32_t len, bool is_native_endian);
void StateData_free(struct StateData* request);
