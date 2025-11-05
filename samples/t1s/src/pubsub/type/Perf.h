#pragma once

#include <tickle/tickle.h>

struct PerfData {
    uint32_t id;
    uint64_t timestamp;
};

extern struct tt_Topic PerfTopic;

int32_t PerfData_encode_size(struct PerfData* request);
int32_t PerfData_encode(struct PerfData* request, uint8_t* payload, const int32_t len);
int32_t PerfData_decode(struct PerfData* request, const uint8_t* payload, const int32_t len, bool is_native_endian);
void PerfData_free(struct PerfData* request);
