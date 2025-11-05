#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <tickle/config.h>
#include <tickle/hal.h>

#define tt_KIND_NONE 0x00
#define tt_KIND_TOPIC 0x01
#define tt_KIND_SERVICE 0x02
#define tt_KIND_SENDER 0x10
#define tt_KIND_RECEIVER 0x20
#define tt_KIND_TOPIC_SUBSCRIBER (tt_KIND_RECEIVER | tt_KIND_TOPIC)
#define tt_KIND_TOPIC_PUBLISHER (tt_KIND_SENDER | tt_KIND_TOPIC)
#define tt_KIND_SERVICE_CLIENT (tt_KIND_RECEIVER | tt_KIND_SERVICE)
#define tt_KIND_SERVICE_SERVER (tt_KIND_SENDER | tt_KIND_SERVICE)

struct tt_Endpoint;
struct tt_UpdateHeader;
struct tt_Node;

// Task Control Block
struct tt_TCB {
    uint64_t time;
    void (*function)(struct tt_Node* node, uint64_t time, void* param);
    void* param;
};

struct tt_Node {
    uint8_t id;
    uint32_t endpoint_count;
    struct tt_Endpoint* endpoints[tt_MAX_ENDPOINT_COUNT];
    uint64_t last_modified; // Last modified timestamp in ns to announce other nodes e.g. server, publisher

    struct tt_UpdateHeader* updates[tt_MAX_ENDPOINT_COUNT];

    uint8_t tx_buffer[tt_MAX_BUFFER_LENGTH * 2];
    uint32_t tx_tail;
    uint32_t tx_size;

    struct tt_TCB scheduler[tt_MAX_SCHEDULER_LENGTH];
    int32_t scheduler_tail;

    struct tt_hal hal;
};

struct tt_Endpoint {
    uint8_t kind;
    uint32_t id; // hash(type + name)
    const char* name;
};

struct tt_Service;
struct tt_Client;
struct tt_Response;
struct tt_SubmessageHeader;

typedef void (*tt_CLIENT_CALLBACK)(struct tt_Client* client, int8_t return_code, struct tt_Response* response);

struct tt_Client { // extends Endpoint
    struct tt_Endpoint super;
    struct tt_Node* node;
    struct tt_Service* service;
    tt_CLIENT_CALLBACK callback;

    // transcation
    uint16_t seq_no;

    // Cache
    struct tt_SubmessageHeader* cache; // Last call cache
    uint64_t cache_time;               // Cache time
    uint32_t latency;                  // Call latency
};

struct tt_Server;
struct tt_Request;

typedef int8_t (*tt_SERVER_CALLBACK)(struct tt_Server* server, struct tt_Request* request,
                                     struct tt_Response* response);

struct tt_Server { // extends Endpoint
    struct tt_Endpoint super;
    struct tt_Node* node;
    struct tt_Service* service;
    tt_SERVER_CALLBACK callback;

    struct tt_SubmessageHeader* cache[tt_MAX_SERVER_CACHE_COUNT];
};

struct tt_Request {};

struct tt_Response {};

typedef int32_t (*tt_REQUEST_ENCODE_SIZE)(struct tt_Request* request);
typedef int32_t (*tt_REQUEST_ENCODE)(struct tt_Request* request, uint8_t* payload, const uint32_t len);
typedef int32_t (*tt_REQUEST_DECODE)(struct tt_Request* request, const uint8_t* payload, const uint32_t len,
                                     bool is_native_endian);
typedef void (*tt_REQUEST_FREE)(struct tt_Request* request);
typedef int32_t (*tt_RESPONSE_ENCODE_SIZE)(struct tt_Response* response);
typedef int32_t (*tt_RESPONSE_ENCODE)(struct tt_Response* response, uint8_t* payload, const uint32_t len);
typedef int32_t (*tt_RESPONSE_DECODE)(struct tt_Response* response, const uint8_t* payload, const uint32_t len,
                                      bool is_native_endian);
typedef void (*tt_RESPONSE_FREE)(struct tt_Response* response);

struct tt_Service {
    const char* name;
    uint32_t request_size;
    uint32_t response_size;
    tt_REQUEST_ENCODE_SIZE request_encode_size;
    tt_REQUEST_ENCODE request_encode;
    tt_REQUEST_DECODE request_decode;
    tt_REQUEST_FREE request_free;
    tt_RESPONSE_ENCODE_SIZE response_encode_size;
    tt_RESPONSE_ENCODE response_encode;
    tt_RESPONSE_DECODE response_decode;
    tt_RESPONSE_FREE response_free;

    // QoS
    uint32_t call_retry_interval; // 0 means auto
    uint32_t call_retry_count;    // 0 means tt_CALL_RETRY_COUNT
};

struct tt_Topic;

struct tt_Data {};

struct tt_Publisher { // extends Endpoint
    struct tt_Endpoint super;
    struct tt_Node* node;
    struct tt_Topic* topic;

    // transcation
    uint16_t seq_no;
};

struct tt_Subscriber;
typedef void (*tt_SUBSCRIBER_CALLBACK)(struct tt_Subscriber* subscriber, uint64_t time, uint16_t seq_no,
                                       struct tt_Data* data);

struct tt_Subscriber { // extends Endpoint
    struct tt_Endpoint super;
    struct tt_Node* node;
    struct tt_Topic* topic;
    tt_SUBSCRIBER_CALLBACK callback;

    // transcation
    uint16_t seq_no;
};

typedef int32_t (*tt_DATA_ENCODE_SIZE)(struct tt_Data* data);
typedef int32_t (*tt_DATA_ENCODE)(struct tt_Data* data, uint8_t* payload, const uint32_t len);
typedef int32_t (*tt_DATA_DECODE)(struct tt_Data* data, const uint8_t* payload, const uint32_t len,
                                  bool is_native_endian);
typedef void (*tt_DATA_FREE)(struct tt_Data* data);

struct tt_Topic {
    const char* name;
    uint32_t data_size;
    tt_DATA_ENCODE_SIZE data_encode_size;
    tt_DATA_ENCODE data_encode;
    tt_DATA_DECODE data_decode;
    tt_DATA_FREE data_free;

    // QoS
    uint16_t history_depth;
    uint32_t deadline_duration;
    uint32_t lifespan_duration;
};

uint32_t tt_hash_id(const char* type, const char* name);
struct tt_Header;
bool tt_is_native_endian(struct tt_Header* header);
bool tt_is_reverse_endian(struct tt_Header* header);
uint64_t tt_get_ns();

/**
 * @return 0 - succeed
 */
int32_t tt_Node_create(struct tt_Node* node);
int32_t tt_Node_create_client(struct tt_Node* node, struct tt_Client* client, struct tt_Service* service,
                              const char* name, tt_CLIENT_CALLBACK callback);
int32_t tt_Node_create_server(struct tt_Node* node, struct tt_Server* server, struct tt_Service* service,
                              const char* name, tt_SERVER_CALLBACK callback);
int32_t tt_Node_create_publisher(struct tt_Node* node, struct tt_Publisher* pub, struct tt_Topic* topic,
                                 const char* name);
int32_t tt_Node_create_subscriber(struct tt_Node* node, struct tt_Subscriber* sub, struct tt_Topic* topic,
                                  const char* name, tt_SUBSCRIBER_CALLBACK callback);
bool tt_Node_schedule(struct tt_Node* node, uint64_t time,
                      void (*function)(struct tt_Node* node, uint64_t time, void* param), void* param);

int32_t tt_Client_call(struct tt_Client* client, struct tt_Request* request);
int32_t tt_Client_destroy(struct tt_Client* client);

int32_t tt_Server_destroy(struct tt_Server* server);

int32_t tt_Publisher_publish(struct tt_Publisher* pub, struct tt_Data* data);
int32_t tt_Publisher_destroy(struct tt_Publisher* pub);

int32_t tt_Subscriber_destroy(struct tt_Subscriber* sub);

int32_t tt_Node_poll(struct tt_Node* node);
int32_t tt_Node_destroy(struct tt_Node* node);

#define tt_VERSION 1
#define tt_PROTOCOL_UPDATE 0
#define tt_PROTOCOL_DATA 1
#define tt_PROTOCOL_ACKNACK 2

struct tt_Header {
    union {
        char magic[2]; // "TK" for big endian, "KT" for little endian
        uint16_t magic_value;
    };
    uint8_t version; // protocol version
    uint8_t source;  // Sender ID
} __attribute__((packed));

#define tt_SUBMESSAGE_ID_ALL 0xff

#define tt_SUBMESSAGE_TYPE_UPDATE 1
#define tt_SUBMESSAGE_TYPE_DATA 2
#define tt_SUBMESSAGE_TYPE_ACKNACK 3
#define tt_SUBMESSAGE_TYPE_CALLREQUEST 4
#define tt_SUBMESSAGE_TYPE_CALLRESPONSE 5

struct tt_SubmessageHeader {
    uint8_t type;     // 0 for Node update, 2 for Data, 3 for AckNack
    uint8_t receiver; // Receiver ID
    uint16_t length;  // Body length in 4 bytes including header
} __attribute__((packed));

struct tt_UpdateHeader {
    uint64_t last_modified;
    uint8_t entity_count;
    /* Dynamically allocated
    struct tt_UpdateEntity entities[];
    */
} __attribute__((packed));

struct tt_UpdateEntity {
    uint32_t id; // endpoint id: hash(type + name)
    uint8_t kind;
    /* Dynamically allocated
    uint16_t type_len;
    char type[];
    uint16_t name_len;
    char name[];
    */
} __attribute__((packed));

struct tt_DataHeader {
    uint32_t id; // endpoint id
    uint32_t seq_no;
    uint64_t timestamp;
    // type + name
    // CDR
} __attribute__((packed));

struct tt_AckNackHeader {
    uint32_t seq_no;
    uint64_t bitmap;
    // type + name
} __attribute__((packed));

struct tt_CallRequestHeader {
    uint32_t id;     // endpoint id
    uint16_t seq_no; // sequence number
    uint8_t retry;   // retry count from client side
    // CDR
} __attribute__((packed));

struct tt_CallResponseHeader {
    uint32_t id;        // endpoint id
    uint16_t seq_no;    // sequence number
    uint8_t retry;      // retry count from server side
    int8_t return_code; // return code
    // type + name
    // CDR
} __attribute__((packed));
