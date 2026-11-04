#ifndef DTOE_MSG_PINGPONG_H
#define DTOE_MSG_PINGPONG_H

#include "dtoe_types.h"

#define MSG_PINGPONG_STR_LEN 16

/*
 * Same as idl/pingpong.idl
 * struct pingpong {
 *     char str[16];
 *     unsigned long count;
 *     float angle;
 * };
 */
typedef struct msg_pingpong {
    char str[MSG_PINGPONG_STR_LEN];
    uint32_t count;
    float angle;
} msg_pingpong_t;

int32_t msg_pingpong_serialized_len(const void* sample);
int32_t msg_pingpong_deserialized_len(const void* sample);
int32_t msg_pingpong_serialize(const void* sample, uint8_t* payload, uint16_t len);
int32_t msg_pingpong_deserialize(void* sample, const uint8_t* payload, uint16_t len);

extern const dtoe_typesupport_t msg_pingpong;

#endif /* DTOE_MSG_PINGPONG_H */
