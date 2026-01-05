#include "control_msg.h"
#include "lan8651.h"

#define MIN_ETH_PACKET_SIZE (64)

int send_control_msg_request(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
        const uint8_t target_mac_addr[ETH_ALEN], uint8_t slave_id, uint8_t command)
{
    uint8_t send_buff[MIN_ETH_PACKET_SIZE] = { '\0', };

    struct ethhdr *eth = (struct ethhdr *)send_buff;
    memcpy(eth->h_dest, target_mac_addr, ETH_ALEN);
    memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
    eth->h_proto = sys_cpu_to_be16(CONTROL_MSG_PROTO_TYPE);

    struct control_request_msg *request_msg = (struct control_request_msg *)(eth + 1);
    request_msg->header.msg_type = CONTROL_MSG_TYPE_REQUEST;
    request_msg->header.slave_id = slave_id;
    request_msg->command = command;

    return send_packet(spi, (uint8_t *)&send_buff, MIN_ETH_PACKET_SIZE);
}

int send_control_msg_response(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
        const uint8_t target_mac_addr[ETH_ALEN], uint8_t slave_id, uint8_t response)
{
    uint8_t send_buff[MIN_ETH_PACKET_SIZE] = { '\0', };

    struct ethhdr *eth = (struct ethhdr *)send_buff;
    memcpy(eth->h_dest, target_mac_addr, ETH_ALEN);
    memcpy(eth->h_source, my_mac_addr, ETH_ALEN);
    eth->h_proto = sys_cpu_to_be16(CONTROL_MSG_PROTO_TYPE);

    struct control_response_msg *response_msg = (struct control_response_msg *)(eth + 1);
    response_msg->header.msg_type = CONTROL_MSG_TYPE_RESPONSE;
    response_msg->header.slave_id = slave_id;
    response_msg->response = response;

    return send_packet(spi, (uint8_t *)&send_buff, MIN_ETH_PACKET_SIZE);
}