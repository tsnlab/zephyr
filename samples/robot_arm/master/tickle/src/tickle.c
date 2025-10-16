#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tickle/config.h>
#include <tickle/hal.h>
#include <tickle/tickle.h>

#include "consts.h"
#include "encoding.h"
#include "log.h"

#define UNUSED(x) (void)(x)

static struct tt_SubmessageHeader* start_encode(struct tt_Node* node, uint8_t type, uint8_t receiver) {
    if (node->tx_tail + sizeof(struct tt_SubmessageHeader) >= node->tx_size) {
        TT_LOG_ERROR("Lack of tx buffer");
        return NULL;
    }

    struct tt_SubmessageHeader* submessage_header = (struct tt_SubmessageHeader*)(node->tx_buffer + node->tx_tail);
    node->tx_tail += sizeof(struct tt_SubmessageHeader);

    submessage_header->type = type;
    submessage_header->receiver = receiver;
    submessage_header->length = 0;

    return submessage_header;
}

static void* encode(struct tt_Node* node, uint32_t len) {
    if (node->tx_tail + len >= node->tx_size) {
        TT_LOG_ERROR("Lack of tx buffer");
        return NULL;
    }

    return tt_encode_buffer(node->tx_buffer, &node->tx_tail, len);
}

static bool encode_string(struct tt_Node* node, const char* str) {
    if (!tt_encode_string(node->tx_buffer, &node->tx_tail, node->tx_size, str)) {
        TT_LOG_ERROR("Lack of tx buffer");
        return false;
    }
    return true;
}

static void rollback(struct tt_Node* node, struct tt_SubmessageHeader* submessage_header) {
    node->tx_tail = (uintptr_t)submessage_header - (uintptr_t)node->tx_buffer;
}

static bool flush_tx(struct tt_Node* node, uint32_t len) {
    // Check at least 1 submessage is contained
    if (len < sizeof(struct tt_Header) + sizeof(struct tt_SubmessageHeader)) {
        return true; // Nothing to flush
    }

    // Not possible
    if (len > tt_MAX_BUFFER_LENGTH) {
        TT_LOG_ERROR("Logic ERROR!!!!");
        return false;
    }

    struct tt_Header* header = (struct tt_Header*)node->tx_buffer;
    header->magic_value = NATIVE_MAGIC_VALUE;
    header->version = tt_VERSION;
    header->source = node->id;

    if (tt_send(node, node->tx_buffer, len) < 0) {
        perror("Cannot send packet");
        return false;
    }

    _tt_memmove(node->tx_buffer + sizeof(struct tt_Header), node->tx_buffer + len, node->tx_tail - len);
    node->tx_tail = sizeof(struct tt_Header) + (node->tx_tail - len);

    return true;
}

static bool end_encode(struct tt_Node* node, struct tt_SubmessageHeader* submessage_header, bool is_flush) {
    // Set submessage header length
    size_t length = (uintptr_t)node->tx_buffer + node->tx_tail - (uintptr_t)submessage_header;
    size_t roundup = ROUNDUP(length) - length;

    if (node->tx_tail + roundup <= tt_MAX_BUFFER_LENGTH) { // tail in below the buffer
        submessage_header->length = length + roundup;
        node->tx_tail += roundup;
    } else if (node->tx_tail <= tt_MAX_BUFFER_LENGTH &&
               node->tx_tail + roundup > tt_MAX_BUFFER_LENGTH) { // tail exceeds buffer if roundup
        submessage_header->length = length;
        is_flush = true;
    } else { // tail exceeds buffer
        submessage_header->length = length + roundup;
        node->tx_tail += roundup;
        is_flush = true;
    }

    // Case 1: Immediate flush when is_flush is true
    // case 2: Flush when tx_tail exceeds tt_MAX_BUFFER_LENGTH
    if (is_flush) {
        uint32_t len;
        /* TODO: Is this really necessary? */
        if (true || node->tx_tail > tt_MAX_BUFFER_LENGTH) {
            len = node->tx_tail;
        } else {
            len = node->tx_tail - length;
        }

        if (!flush_tx(node, len)) {
            return false;
        }
    } else {
        ; // Case 3: Don't flush
    }

    return true;
}

static void* decode(struct tt_Node* node, uint8_t* buffer, uint32_t* head, uint32_t tail, uint32_t length) {
    UNUSED(node);
    return tt_decode_buffer(buffer, head, tail, length);
}

static bool decode_string(struct tt_Node* node, uint8_t* buffer, uint32_t* head, uint32_t tail, uint16_t* str_len,
                          char** str) {
    UNUSED(node);

    if (!tt_decode_string(buffer, head, tail, str_len, str)) {
        TT_LOG_ERROR("Too big string length: %d + %d > %d", *head, *str_len, tail);
        return false;
    }

    return true;
}

static struct tt_Endpoint* find_endpoint(struct tt_Node* node, uint8_t kind, uint32_t id) {
    for (int i = 0; i < node->endpoint_count; i++) {
        struct tt_Endpoint* endpoint = node->endpoints[i];
        if (endpoint->kind == kind && endpoint->id == id) {
            return endpoint;
        }
    }

    return NULL;
}

bool tt_Node_schedule(struct tt_Node* node, uint64_t time,
                      void (*function)(struct tt_Node* node, uint64_t time, void* param), void* param) {
    if (node->scheduler_tail + 1 >= tt_MAX_SCHEDULER_LENGTH) {
        return false;
    }

    struct tt_TCB* tcb = &node->scheduler[node->scheduler_tail];
    for (int32_t i = 0; i < node->scheduler_tail; i++) {
        if (node->scheduler[i].time > time) {
            _tt_memmove(&node->scheduler[i + 1], &node->scheduler[i],
                        sizeof(struct tt_TCB) * (node->scheduler_tail - i));
            tcb = &node->scheduler[i];
            break;
        }
    }

    tcb->time = time;
    tcb->function = function;
    tcb->param = param;

    node->scheduler_tail++;

    return true;
}

static struct tt_TCB* peek_scheduler(struct tt_Node* node) {
    if (node->scheduler_tail > 0) {
        return &node->scheduler[0];
    }

    return NULL;
}

static void pop_scheduler(struct tt_Node* node) {
    _tt_memmove(&node->scheduler[0], &node->scheduler[1], sizeof(struct tt_TCB) * (node->scheduler_tail - 1));
    node->scheduler_tail--;
}

static void node_update(struct tt_Node* node, uint64_t time, void* param);
static void node_flush(struct tt_Node* node, uint64_t time, void* param);

int32_t tt_Node_create(struct tt_Node* node) {
    tt_log_init(TT_LOG_DEBUG, stdout);
    node->id = 0;
    node->endpoint_count = 0;

    for (int i = 0; i < tt_MAX_ENDPOINT_COUNT; i++) {
        node->endpoints[i] = NULL;
    }

    node->last_modified = 0;

    for (int i = 0; i < tt_MAX_ENDPOINT_COUNT; i++) {
        node->updates[i] = NULL;
    }

    memset(node->tx_buffer, 0, (long)tt_MAX_BUFFER_LENGTH * 2);
    node->tx_tail = sizeof(struct tt_Header);
    node->tx_size = tt_MAX_BUFFER_LENGTH * 2;

    memset(node->scheduler, 0, sizeof(struct tt_TCB) * tt_MAX_SCHEDULER_LENGTH);
    node->scheduler_tail = 0;

    node->id = tt_get_node_id();

    if (node->id == 0) {
        TT_LOG_ERROR("Cannot get node id from IP address");
        return -2;
    }

    if (tt_bind(node) != 0) {
        TT_LOG_ERROR("Cannot bind");
        return -3;
    }

    TT_LOG_INFO("Node open at %d", _tt_CONFIG.port);

    // Register TCBs to scheduler
    uint64_t basetime = tt_get_ns();
    uint64_t rem = basetime % tt_NODE_CYCLE;
    basetime = basetime - rem + tt_NODE_CYCLE;

    if (!tt_Node_schedule(node, basetime, node_update, NULL)) {
        TT_LOG_ERROR("Cannot schedule node_update");
        return -1;
    }

    if (!tt_Node_schedule(node, basetime + tt_NODE_TX_INTERVAL, node_flush, NULL)) {
        TT_LOG_ERROR("Cannot schedule node_flush");
        return -1;
    }

    return 0;
}

int32_t tt_Node_create_client(struct tt_Node* node, struct tt_Client* client, struct tt_Service* service,
                              const char* name, tt_CLIENT_CALLBACK callback) {
    // TODO: Check dup
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)client;
    endpoint->kind = tt_KIND_SERVICE_CLIENT;
    endpoint->id = tt_hash_id(service->name, name);
    endpoint->name = name;
    client->node = node;
    client->service = service;
    client->callback = callback;
    client->seq_no = 0;
    client->cache = NULL;
    client->cache_time = 0;
    client->latency = 0;

    node->endpoints[node->endpoint_count++] = endpoint;

    return 0;
}

int32_t tt_Node_create_server(struct tt_Node* node, struct tt_Server* server, struct tt_Service* service,
                              const char* name, tt_SERVER_CALLBACK callback) {
    // TODO: Check dup
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)server;
    endpoint->kind = tt_KIND_SERVICE_SERVER;
    endpoint->id = tt_hash_id(service->name, name);
    endpoint->name = name;
    server->node = node;
    server->service = service;
    server->callback = callback;

    for (int i = 0; i < tt_MAX_SERVER_CACHE_COUNT; i++) {
        server->cache[i] = NULL;
    }

    node->endpoints[node->endpoint_count++] = endpoint;
    node->last_modified = tt_get_ns();

    return 0;
}

int32_t tt_Node_create_publisher(struct tt_Node* node, struct tt_Publisher* pub, struct tt_Topic* topic,
                                 const char* name) {
    // TODO: Check dup
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)pub;
    endpoint->kind = tt_KIND_TOPIC_PUBLISHER;
    endpoint->id = tt_hash_id(topic->name, name);
    endpoint->name = name;
    pub->node = node;
    pub->topic = topic;
    pub->seq_no = 0;

    node->endpoints[node->endpoint_count++] = endpoint;
    node->last_modified = tt_get_ns();

    return 0;
}

int32_t tt_Node_create_subscriber(struct tt_Node* node, struct tt_Subscriber* sub, struct tt_Topic* topic,
                                  const char* name, tt_SUBSCRIBER_CALLBACK callback) {
    // TODO: Check dup
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)sub;
    endpoint->kind = tt_KIND_TOPIC_SUBSCRIBER;
    endpoint->id = tt_hash_id(topic->name, name);
    endpoint->name = name;
    sub->node = node;
    sub->topic = topic;
    sub->callback = callback;

    node->endpoints[node->endpoint_count++] = endpoint;

    return 0;
}

static void call_retry(struct tt_Node* node, uint64_t time, void* param) {
    UNUSED(time);

    struct tt_Client* client = param;

    struct tt_SubmessageHeader* submessage_header = client->cache;
    if (submessage_header == NULL) {
        // Already respond.
        return;
    }

    struct tt_CallRequestHeader* callrequest_header =
        (struct tt_CallRequestHeader*)((void*)submessage_header + sizeof(struct tt_SubmessageHeader));

    if (++callrequest_header->retry > client->service->call_retry_count) {
        client->callback(client, 0, NULL); // No response from server

        _tt_free(client->cache);
        client->cache = NULL;
    } else {
        void* buf = encode(node, submessage_header->length);
        if (buf == NULL) {
            return;
        }

        _tt_memcpy(buf, submessage_header, submessage_header->length);

        end_encode(node, buf, false);

        uint32_t retry_interval;
        if (client->service->call_retry_interval == 0) {
            retry_interval = client->latency == 0 ? tt_CALL_RETRY_INTERVAL : client->latency;
            retry_interval = retry_interval + (retry_interval >> 1); // latency * 1.5
        } else {
            retry_interval = client->service->call_retry_interval;
        }

        if (!tt_Node_schedule(node, tt_get_ns() + retry_interval, call_retry, client)) {
            TT_LOG_ERROR("Cannot schedule call_retry");
        }
    }
}

int32_t tt_Client_call(struct tt_Client* client, struct tt_Request* request) {
    if (client->cache != NULL) {
        return -3; // waiting response
    }

    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)client;
    struct tt_Node* node = client->node;

    // Header and SubmessageHeader
    struct tt_SubmessageHeader* submessage_header =
        start_encode(node, tt_SUBMESSAGE_TYPE_CALLREQUEST, tt_SUBMESSAGE_ID_ALL);
    if (submessage_header == NULL) {
        rollback(node, submessage_header);
        return -1;
    }

    // CallRequestHeader
    struct tt_CallRequestHeader* callrequest_header = encode(node, sizeof(struct tt_CallRequestHeader));
    if (callrequest_header == NULL) {
        rollback(node, submessage_header);
        return -1;
    }

    callrequest_header->id = endpoint->id;
    callrequest_header->seq_no = client->seq_no;
    callrequest_header->retry = 0;

    // CallRequestBody
    int32_t cdr_len = client->service->request_encode_size(request);
    void* cdr = encode(node, cdr_len);
    if (cdr == NULL) {
        rollback(node, submessage_header);
        return -1;
    }

    int32_t encoded_len = client->service->request_encode(request, cdr, cdr_len);
    if (encoded_len < 0) {
        rollback(node, submessage_header);
        return -2;
    }

    // Make cache
    size_t length = ROUNDUP((uintptr_t)node->tx_buffer + node->tx_tail - (uintptr_t)submessage_header);
    struct tt_SubmessageHeader* cache = _tt_malloc(length);
    if (cache == NULL) {
        TT_LOG_ERROR("Out of memory");
        rollback(node, submessage_header);
        return -3;
    }

    _tt_memcpy(cache, submessage_header, length);
    cache->length = length;

    // Flush tx
    if (!end_encode(node, submessage_header, false)) {
        _tt_free(cache);
        rollback(node, submessage_header);
        return -3;
    }

    client->cache = cache;
    client->cache_time = tt_get_ns();
    client->seq_no++;

    uint32_t retry_interval;
    if (client->service->call_retry_interval == 0) {
        retry_interval = client->latency == 0 ? tt_CALL_RETRY_INTERVAL : client->latency;
        retry_interval = retry_interval + (retry_interval >> 1); // latency * 1.5
    } else {
        retry_interval = client->service->call_retry_interval;
    }

    if (!tt_Node_schedule(node, tt_get_ns() + retry_interval, call_retry, client)) {
        TT_LOG_ERROR("Cannot schedule call_retry");
        return -1;
    }

    return 0;
}

int32_t tt_Client_destroy(struct tt_Client* client) {
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)client;

    for (int i = 0; i < client->node->endpoint_count; i++) {
        if (client->node->endpoints[i] == endpoint) {
            client->node->endpoints[i] = NULL;
            client->node->last_modified = tt_get_ns();
            return 0;
        }
    }

    return -1;
}

int32_t tt_Server_destroy(struct tt_Server* server) {
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)server;

    for (int i = 0; i < server->node->endpoint_count; i++) {
        if (server->node->endpoints[i] == endpoint) {
            server->node->endpoints[i] = NULL;
            server->node->last_modified = tt_get_ns();
            return 0;
        }
    }

    return -1;
}

int32_t tt_Publisher_publish(struct tt_Publisher* pub, struct tt_Data* data) {
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)pub;
    struct tt_Node* node = pub->node;

    // Header and SubmessageHeader
    struct tt_SubmessageHeader* submessage_header = start_encode(node, tt_SUBMESSAGE_TYPE_DATA, tt_SUBMESSAGE_ID_ALL);
    if (submessage_header == NULL) {
        rollback(node, submessage_header);
        return -1;
    }

    // CallRequestHeader
    struct tt_DataHeader* data_header = encode(node, sizeof(struct tt_DataHeader));
    if (data_header == NULL) {
        rollback(node, submessage_header);
        return -1;
    }

    data_header->id = endpoint->id;
    data_header->seq_no = pub->seq_no + 1;
    data_header->timestamp = tt_get_ns();

    // DataBody
    int32_t cdr_len = pub->topic->data_encode_size(data);
    void* cdr = encode(node, cdr_len);
    if (cdr == NULL) {
        rollback(node, submessage_header);
        return -1;
    }

    int32_t encoded_len = pub->topic->data_encode(data, cdr, cdr_len);
    if (encoded_len < 0) {
        rollback(node, submessage_header);
        return -2;
    }

    if (!end_encode(node, submessage_header, true)) {
        rollback(node, submessage_header);
        return -3;
    }

    pub->seq_no++;

    return 0;
}

int32_t tt_Publisher_destroy(struct tt_Publisher* pub) {
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)pub;

    for (int i = 0; i < pub->node->endpoint_count; i++) {
        if (pub->node->endpoints[i] == endpoint) {
            pub->node->endpoints[i] = NULL;
            pub->node->last_modified = tt_get_ns();
            return 0;
        }
    }

    return -1;
}

int32_t tt_Subscriber_destroy(struct tt_Subscriber* sub) {
    struct tt_Endpoint* endpoint = (struct tt_Endpoint*)sub;

    for (int i = 0; i < sub->node->endpoint_count; i++) {
        if (sub->node->endpoints[i] == endpoint) {
            sub->node->endpoints[i] = NULL;
            sub->node->last_modified = tt_get_ns();
            return 0;
        }
    }

    return -1;
}

static void node_update(struct tt_Node* node, uint64_t time, void* param) {
    UNUSED(param);

    // Header and SubmessageHeader
    struct tt_SubmessageHeader* submessage_header = start_encode(node, tt_SUBMESSAGE_TYPE_UPDATE, tt_SUBMESSAGE_ID_ALL);
    if (submessage_header == NULL) {
        TT_LOG_ERROR("Lack of tx_buffer");
        goto done;
    }

    struct tt_UpdateHeader* update_header = encode(node, sizeof(struct tt_UpdateHeader));
    if (update_header == NULL) {
        TT_LOG_ERROR("Lack of tx_buffer");
        rollback(node, submessage_header);
        goto done;
    }

    update_header->last_modified = node->last_modified;
    update_header->entity_count = 0;

    uint8_t entity_count = 0;
    for (int i = 0; i < node->endpoint_count; i++) {
        struct tt_Endpoint* endpoint = node->endpoints[i];

        const char* type;

        switch (endpoint->kind) {
        case tt_KIND_TOPIC_PUBLISHER:
            type = ((struct tt_Publisher*)endpoint)->topic->name;
            break;
        case tt_KIND_TOPIC_SUBSCRIBER:
        case tt_KIND_SERVICE_CLIENT:
            continue;
        case tt_KIND_SERVICE_SERVER:
            type = ((struct tt_Server*)endpoint)->service->name;
            break;
        default:
            rollback(node, submessage_header);
            TT_LOG_ERROR("Illegal endpoint kind: %d", endpoint->kind);
            goto done;
        }

        struct tt_UpdateEntity* update_entity = encode(node, sizeof(struct tt_UpdateEntity));
        if (update_entity == NULL) {
            TT_LOG_ERROR("Lack of tx_buffer");
            rollback(node, submessage_header);
            goto done;
        }

        update_entity->id = endpoint->id;
        update_entity->kind = endpoint->kind;

        if (!encode_string(node, type)) {
            TT_LOG_ERROR("Lack of tx_buffer");
            rollback(node, submessage_header);
            goto done;
        }

        if (!encode_string(node, endpoint->name)) {
            TT_LOG_ERROR("Lack of tx_buffer");
            rollback(node, submessage_header);
            goto done;
        }

        if (++entity_count == UINT8_MAX) {
            TT_LOG_WARNING("Maximum update entity: endpoint index: %d, endpoint count: %d", i, node->endpoint_count);
            break;
        }
    }

    update_header->entity_count = entity_count;

    if (!end_encode(node, submessage_header, false)) {
        TT_LOG_ERROR("Lack of tx_buffer");
        rollback(node, submessage_header);
        goto done;
    }

done:
    if (!tt_Node_schedule(node, time + tt_NODE_UPDATE_INTERVAL, node_update, NULL)) {
        TT_LOG_ERROR("Cannot schedule node_update");
    }
}

static void node_flush(struct tt_Node* node, uint64_t time, void* param) {
    UNUSED(param);

    if (!flush_tx(node, node->tx_tail)) {
        TT_LOG_ERROR("Cannot flush tx_buffer");
    }

    if (!tt_Node_schedule(node, time + tt_NODE_TX_INTERVAL, node_flush, NULL)) {
        TT_LOG_ERROR("Cannot schedule node_flush");
    }
}

static bool process_update(struct tt_Node* node, struct tt_Header* header, uint8_t* buffer, uint32_t head,
                           uint32_t tail) {
    uint32_t length = tail - head;

    struct tt_UpdateHeader* update_header = decode(node, buffer, &head, tail, sizeof(struct tt_UpdateHeader));
    if (update_header == NULL) {
        TT_LOG_ERROR("Illegal UpdateHeader");
        return false;
    }

    TT_LOG_DEBUG("  update->last_modified = %lu", update_header->last_modified);
    TT_LOG_DEBUG("  update->entity_count = %u", update_header->entity_count);

    if (node->updates[header->source] != NULL &&
        node->updates[header->source]->last_modified == update_header->last_modified) {
        return true;
    }

    struct tt_UpdateHeader* new_update = _tt_malloc(length);
    if (new_update == NULL) {
        TT_LOG_ERROR("Out of memory!");
        return false;
    }

    _tt_memcpy(new_update, update_header, length);

    // Update
    /*
    struct tt_UpdateHeader* old_update = node->updates[header->source];
    void* old_p = (void*)old_update + sizeof(struct tt_UpdateHeader);
    struct tt_UpdateEntity* old_entity = old_p;
    int old_entity_count = old_update->entity_count;
    int old_entity_id = -1;

    void* new_p = (void*)new_update + sizeof(struct tt_UpdateHeader);
    struct tt_UpdateEntity* new_entity = new_p;
    int new_entity_count = new_update->entity_count;
    int new_entity_id = -1;

    while (true) {

    }
    */

    int entity_count = update_header->entity_count;

    for (int i = 0; i < entity_count && head + sizeof(struct tt_UpdateEntity) + 2 * sizeof(uint16_t) < tail; i++) {
        struct tt_UpdateEntity* update_entity = decode(node, buffer, &head, tail, sizeof(struct tt_UpdateEntity));

        TT_LOG_DEBUG("  update_entity->id = %08x", update_entity->id);
        TT_LOG_DEBUG("  update_entity->kind = %d", update_entity->kind);

        uint16_t type_len = 0;
        char* type = NULL;
        if (!decode_string(node, buffer, &head, tail, &type_len, &type)) {
            TT_LOG_ERROR("Cannot decode type");
            return false;
        }

        TT_LOG_DEBUG("  update_entity->type: (%d)\"%s\"", type_len, type);

        uint16_t name_len = 0;
        char* name = NULL;
        if (!decode_string(node, buffer, &head, tail, &name_len, &name)) {
            TT_LOG_ERROR("Cannot decode name");
            return false;
        }

        TT_LOG_DEBUG("  update_entity->name: (%d)\"%s\"", name_len, name);
    }

    return true;
}

static bool process_data(struct tt_Node* node, struct tt_Header* header, uint8_t* buffer, uint32_t head,
                         uint32_t tail) {
    uint32_t length = tail - head;

    struct tt_DataHeader* data_header = decode(node, buffer, &head, tail, sizeof(struct tt_DataHeader));
    if (data_header == NULL) {
        TT_LOG_ERROR("  Illegal DataHeader");
        return false;
    }

    TT_LOG_DEBUG("  Data");
    TT_LOG_DEBUG("  id: %08x", data_header->id);
    TT_LOG_DEBUG("  timestamp: %ld", data_header->timestamp);
    TT_LOG_DEBUG("  seq_no: %d", data_header->seq_no);

    struct tt_Endpoint* endpoint = find_endpoint(node, tt_KIND_TOPIC_SUBSCRIBER, data_header->id);
    if (endpoint != NULL) {
        // Callback
        struct tt_Subscriber* sub = (struct tt_Subscriber*)endpoint;
        struct tt_Topic* topic = sub->topic;

        uint8_t data[topic->data_size];
        int32_t decoded =
            topic->data_decode((struct tt_Data*)data, buffer + head, tail - head, tt_is_native_endian(header));
        sub->callback(sub, data_header->timestamp, data_header->seq_no, (struct tt_Data*)data);

        topic->data_free((struct tt_Data*)data);

        return true;
    }

    return true;
}

static struct tt_SubmessageHeader* get_server_cache(struct tt_Server* server, uint8_t receiver, uint16_t seq_no) {
    for (int i = 0; i < tt_MAX_SERVER_CACHE_COUNT; i++) {
        if (server->cache[i] != NULL) {
            struct tt_SubmessageHeader* submessage_header = server->cache[i];
            struct tt_CallResponseHeader* callresponse_header =
                (struct tt_CallResponseHeader*)((void*)submessage_header + sizeof(struct tt_SubmessageHeader));

            if (submessage_header->receiver == receiver && callresponse_header->seq_no == seq_no) {
                return submessage_header;
            }
        }
    }

    return NULL;
}

struct server_cache_clean_config {
    struct tt_Server* server;
    struct tt_SubmessageHeader* cache;
};

static void server_cache_clean(struct tt_Node* node, uint64_t time, void* param) {
    UNUSED(node);
    UNUSED(time);

    struct server_cache_clean_config* clean = param;

    for (int i = 0; i < tt_MAX_SERVER_CACHE_COUNT; i++) {
        if (clean->server->cache[i] == clean->cache) {
            clean->server->cache[i] = NULL;
            _tt_free(clean->cache);
            break;
        }
    }

    _tt_free(clean);
}

static bool set_server_cache(struct tt_Server* server, struct tt_SubmessageHeader* submessage_header,
                             uint8_t receiver) {
    size_t length = ROUNDUP((uintptr_t)server->node->tx_buffer + server->node->tx_tail - (uintptr_t)submessage_header);

    struct tt_SubmessageHeader* cache = _tt_malloc(length);
    if (cache == NULL) {
        TT_LOG_ERROR("Out of memory: %ld", length);
        return false;
    }

    _tt_memcpy(cache, submessage_header, length);
    cache->length = length;

    for (int i = 0; i < tt_MAX_SERVER_CACHE_COUNT; i++) {
        if (server->cache[i] != NULL) {
            struct tt_SubmessageHeader* submessage_header = server->cache[i];
            struct tt_CallResponseHeader* callresponse_header =
                (struct tt_CallResponseHeader*)((void*)submessage_header + sizeof(struct tt_SubmessageHeader));

            if (submessage_header->receiver == receiver) {
                _tt_free(server->cache[i]);
                server->cache[i] = NULL;
            }
        }

        if (cache != NULL && server->cache[i] == NULL) {
            struct tt_CallResponseHeader* callresponse_header =
                (struct tt_CallResponseHeader*)((void*)cache + sizeof(struct tt_SubmessageHeader));
            server->cache[i] = cache;

            struct server_cache_clean_config* clean = _tt_malloc(sizeof(struct server_cache_clean_config));
            if (clean == NULL) {
                TT_LOG_ERROR("Out of memory");
                return false;
            }

            clean->server = server;
            clean->cache = cache;

            if (!tt_Node_schedule(server->node, tt_get_ns() + tt_SERVER_CACHE_TIMEOUT, server_cache_clean, clean)) {
                TT_LOG_ERROR("Cannot schedule server_cache_clean");
                return false;
            }

            cache = NULL;
        }
    }

    if (cache != NULL) {
        _tt_free(cache);
        return false;
    }

    return true;
}

static bool process_callrequest(struct tt_Node* node, struct tt_Header* header, uint8_t* buffer, uint32_t head,
                                uint32_t tail) {
    UNUSED(node);

    uint32_t length = tail - head;

    struct tt_CallRequestHeader* callrequest_header =
        decode(node, buffer, &head, tail, sizeof(struct tt_CallRequestHeader));
    if (callrequest_header == NULL) {
        TT_LOG_ERROR("Illegal CallRequestHeader");
        return false;
    }

    TT_LOG_DEBUG("CallRequest");
    TT_LOG_DEBUG("  id: %08x", callrequest_header->id);
    TT_LOG_DEBUG("  seq_no: %d", callrequest_header->seq_no);
    TT_LOG_DEBUG("  retry: %d", callrequest_header->retry);

    struct tt_Endpoint* endpoint = find_endpoint(node, tt_KIND_SERVICE_SERVER, callrequest_header->id);
    if (endpoint == NULL) {
        return true;
    }

    struct tt_Server* server = (struct tt_Server*)endpoint;
    struct tt_Service* service = server->service;

    // Check cache
    struct tt_SubmessageHeader* submessage_header =
        get_server_cache(server, header->source, callrequest_header->seq_no);
    if (submessage_header != NULL) {
        struct tt_CallResponseHeader* callresponse_header =
            (void*)submessage_header + sizeof(struct tt_SubmessageHeader);
        callresponse_header->retry++;

        void* buf = encode(node, submessage_header->length);

        if (buf == NULL) {
            TT_LOG_ERROR("Cannot retry response");
            return false;
        }

        _tt_memcpy(buf, submessage_header, submessage_header->length);

        submessage_header = buf;
    } else {
        // Callback
        uint8_t request[service->request_size];
        uint32_t decoded = service->request_decode((struct tt_Request*)request, buffer + head, tail - head,
                                                   tt_is_native_endian(header));

        uint8_t response[service->response_size];
        int8_t return_code = server->callback(server, (struct tt_Request*)request, (struct tt_Response*)response);

        service->request_free((struct tt_Request*)request);

        TT_LOG_DEBUG("return_code: %d", return_code);

        // Send response
        // Header and SubmessageHeader
        submessage_header = start_encode(node, tt_SUBMESSAGE_TYPE_CALLRESPONSE, header->source);
        if (submessage_header == NULL) {
            return false;
        }

        // CallResponseHeader
        struct tt_CallResponseHeader* call_response_header = encode(node, sizeof(struct tt_CallResponseHeader));
        if (call_response_header == NULL) {
            rollback(node, submessage_header);
            return false;
        }

        call_response_header->id = endpoint->id;
        call_response_header->seq_no = callrequest_header->seq_no;
        call_response_header->retry = 0;
        call_response_header->return_code = return_code;

        // CallRequestBody
        if (return_code == 0) {
            int32_t cdr_len = service->response_encode_size((struct tt_Response*)response);
            void* cdr = encode(node, cdr_len);
            if (cdr == NULL) {
                rollback(node, submessage_header);
                return false;
            }

            int32_t encoded_len = service->response_encode((struct tt_Response*)response, cdr, cdr_len);
            service->response_free((struct tt_Response*)response);

            if (encoded_len < 0) {
                rollback(node, submessage_header);
                return false;
            }
        }

        // Cache submessage header before flush
        if (!set_server_cache(server, submessage_header, header->source)) {
            rollback(node, submessage_header);
            TT_LOG_ERROR("Cannot cache callresponse: Out of cache");
            return false;
        }
    }

    // Flush
    if (!end_encode(node, submessage_header, false)) {
        rollback(node, submessage_header);
        return false;
    }
    return true;
}

static bool process_callresponse(struct tt_Node* node, struct tt_Header* header, uint8_t* buffer, uint32_t head,
                                 uint32_t tail) {
    struct tt_CallResponseHeader* callresponse_header =
        decode(node, buffer, &head, tail, sizeof(struct tt_CallResponseHeader));
    if (callresponse_header == NULL) {
        TT_LOG_ERROR("Illegal CallResponseHeader");
        return false;
    }

    TT_LOG_DEBUG("CallResponse");
    TT_LOG_DEBUG("  id: %08x", callresponse_header->id);
    TT_LOG_DEBUG("  seq_no: %d", callresponse_header->seq_no);
    TT_LOG_DEBUG("  retry: %d", callresponse_header->retry);
    TT_LOG_DEBUG("  return_code: %d", callresponse_header->return_code);

    struct tt_Endpoint* endpoint = find_endpoint(node, tt_KIND_SERVICE_CLIENT, callresponse_header->id);
    if (endpoint == NULL) {
        return true;
    }

    struct tt_Client* client = (struct tt_Client*)endpoint;
    struct tt_Service* service = client->service;
    uint32_t latency = client->cache_time - tt_get_ns();

    struct tt_Response* response = NULL;
    uint8_t response_buffer[service->response_size];

    if (callresponse_header->return_code == 0) {
        int32_t decoded = service->response_decode((struct tt_Response*)response_buffer, buffer + head, tail - head,
                                                   tt_is_native_endian(header));

        if (decoded < 0) {
            TT_LOG_ERROR("Cannot decode response: %d", decoded);
            return false;
        }

        response = (struct tt_Response*)response_buffer;
    }

    _tt_free(client->cache);
    client->cache = NULL;

    if (client->latency == 0) {
        client->latency = latency;
    } else {
        // Latency moving average
        // client->latency * 0.875 + latency * 0.125
        client->latency = (client->latency - (client->latency >> 3)) + (latency >> 3);
    }

    client->callback(client, callresponse_header->return_code, response);

    if (response != NULL) {
        service->response_free(response);
    }

    return true;
}

static bool process_packet(struct tt_Node* node, uint8_t* buffer, uint32_t head, uint32_t tail) {
    // Decode header
    struct tt_Header* header = decode(node, buffer, &head, tail, sizeof(struct tt_Header));
    if (header == NULL) {
        TT_LOG_ERROR("RX buffer underflow");
        return false;
    }

    bool is_native_endian = false;
    if (tt_is_native_endian(header)) {
        is_native_endian = true;
    } else if (tt_is_reverse_endian(header)) {
        is_native_endian = false;
    } else {
        TT_LOG_ERROR("Illegal magic: 0x%04x", header->magic_value);
        return false;
    }

    TT_LOG_DEBUG("magic: 0x%04x (%c%c)", header->magic_value, header->magic[0], header->magic[1]);

    // Accept higher version while ignoring ignoring reserved field. But not lower version.
    if (header->version < tt_VERSION) {
        TT_LOG_ERROR("Illegal version: %d < %d", header->version, tt_VERSION);
        return false;
    }

    // Self sent message
    if (header->source == node->id) {
        TT_LOG_DEBUG("Self sent packet");
        return true;
    }
    TT_LOG_DEBUG("header->source: %d", header->source);

    // Parse submessage
    while (true) {
        // Decode submessage header
        struct tt_SubmessageHeader* submessage_header =
            decode(node, buffer, &head, tail, sizeof(struct tt_SubmessageHeader));
        if (submessage_header == NULL) {
            TT_LOG_DEBUG("End of submessage: %d", tail - head);
            break;
        }

        TT_LOG_DEBUG("submessage->type: %d", submessage_header->type);
        TT_LOG_DEBUG("submessage->receiver: %d", submessage_header->receiver);
        TT_LOG_DEBUG("submessage->length: %d / %ld", submessage_header->length,
                     tail - head + sizeof(struct tt_SubmessageHeader));

        // Decode submessage body
        if (submessage_header->length < sizeof(struct tt_SubmessageHeader) ||
            submessage_header->length > tail - head + sizeof(struct tt_SubmessageHeader)) {
            TT_LOG_ERROR("Illegal submessage length: %d < %ld || %d > %ld", submessage_header->length,
                         sizeof(struct tt_SubmessageHeader), submessage_header->length,
                         tail - head + sizeof(struct tt_SubmessageHeader));
            return false;
        }

        if (submessage_header->receiver == tt_SUBMESSAGE_ID_ALL || submessage_header->receiver == node->id) {
            switch (submessage_header->type) {
            case tt_SUBMESSAGE_TYPE_UPDATE:
                if (!process_update(node, header, buffer, head,
                                    head + submessage_header->length - sizeof(struct tt_SubmessageHeader))) {
                    TT_LOG_ERROR("ERROR on update");
                }
                break;
            case tt_SUBMESSAGE_TYPE_DATA:
                if (!process_data(node, header, buffer, head,
                                  head + submessage_header->length - sizeof(struct tt_SubmessageHeader))) {
                    TT_LOG_ERROR("ERROR on data");
                }
                break;
            case tt_SUBMESSAGE_TYPE_ACKNACK:
                TT_LOG_ERROR("Not supported submessage type: %02x", submessage_header->type);
                return false;
            case tt_SUBMESSAGE_TYPE_CALLREQUEST:
                if (!process_callrequest(node, header, buffer, head,
                                         head + submessage_header->length - sizeof(struct tt_SubmessageHeader))) {
                    TT_LOG_ERROR("ERROR on call request");
                }
                break;
            case tt_SUBMESSAGE_TYPE_CALLRESPONSE:
                if (!process_callresponse(node, header, buffer, head,
                                          head + submessage_header->length - sizeof(struct tt_SubmessageHeader))) {
                    TT_LOG_ERROR("ERROR on call response");
                }
                break;
            default:
                TT_LOG_ERROR("Illegal submessage type: %d, len: %02x", submessage_header->type, tail - head);
                return false;
            }
        }

        head += submessage_header->length - sizeof(struct tt_SubmessageHeader);
    }

    return true;
}

int32_t tt_Node_poll(struct tt_Node* node) {
    uint8_t buffer[tt_MAX_BUFFER_LENGTH];

    while (1) {
        uint32_t ip = 0;
        uint16_t port = 0;
        int32_t len = tt_receive(node, buffer, tt_MAX_BUFFER_LENGTH, &ip, &port);

        if (len == -1) {      // Timeout
            ;                 // Do nothing
        } else if (len < 0) { // I/O error
            printk("Cannot receive data");
            break;
        } else {
            TT_LOG_DEBUG("Process packet from addr: %d.%d.%d.%d:%d len: %d", (ip >> 24) & 0xff, (ip >> 16) & 0xff,
                         (ip >> BITS_IN_1BYTE) & MASK_8BIT, (ip >> 0) & MASK_8BIT, port, len);

            if (!process_packet(node, buffer, 0, len)) {
                TT_LOG_ERROR("Cannot process packet");
            }
        }

        // Scheduled task
        uint64_t time = tt_get_ns();
        struct tt_TCB* tcb = peek_scheduler(node);

        if (tcb != NULL && tcb->time <= time) {
            tcb->function(node, time, tcb->param);
            pop_scheduler(node);
        }
        k_yield();
    }

    return 0;
}

int32_t tt_Node_destroy(struct tt_Node* node) {
    uint64_t time = tt_get_ns();

    for (int i = 0; i < node->endpoint_count; i++) {
        struct tt_Endpoint* endpoint = node->endpoints[i];
        node->endpoints[i] = NULL;

        if (endpoint != NULL && (endpoint->kind & tt_KIND_SENDER) == tt_KIND_SENDER) {
            node->last_modified = time;
        }
    }

    node_update(node, time, NULL);

    tt_close(node);

    return 0;
}
