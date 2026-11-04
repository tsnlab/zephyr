#include "typesupport.h"

#include "msg_pingpong.h"

const dtoe_typesupport_t msg_pingpong = {
    .type_name = "pingpong",
    .serialized_len = msg_pingpong_serialized_len,
    .deserialized_len = msg_pingpong_deserialized_len,
    .serialize = msg_pingpong_serialize,
    .deserialize = msg_pingpong_deserialize,
};

DTOE_TYPESUPPORT_FUNC int32_t msg_pingpong_serialized_len(const void* sample) {
    const msg_pingpong_t* pingpong = sample;

    return align_4bytes(sizeof(pingpong->str)
           + sizeof(pingpong->count)
           + sizeof(pingpong->angle));
}

DTOE_TYPESUPPORT_FUNC int32_t msg_pingpong_deserialized_len(const void* sample) {
    return sizeof(msg_pingpong_t);
}

DTOE_TYPESUPPORT_FUNC int32_t msg_pingpong_serialize(const void* sample, uint8_t* payload, uint16_t len) {
    const msg_pingpong_t* pingpong = sample;
    uint8_t* payload_base = payload;

    encode_buffer_with_len(&payload, pingpong->str, MSG_PINGPONG_STR_LEN);
    ENCODE_BUFFER(&payload, pingpong->count);
    ENCODE_BUFFER(&payload, pingpong->angle);
    return payload - payload_base;
}

DTOE_TYPESUPPORT_FUNC int32_t msg_pingpong_deserialize(void* sample, const uint8_t* payload, uint16_t len) {
    msg_pingpong_t* pingpong = sample;
    const uint8_t* payload_base = payload;

    decode_buffer_with_len(&payload, pingpong->str, MSG_PINGPONG_STR_LEN);
    DECODE_BUFFER(&payload, pingpong->count);
    DECODE_BUFFER(&payload, pingpong->angle);
    return payload - payload_base;
}
