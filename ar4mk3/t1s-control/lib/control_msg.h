#ifndef CONTROL_MSG_H
#define CONTROL_MSG_H

#include <stdint.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/spi.h>

#include "eth.h"

#define CONTROL_MSG_PROTO_TYPE 0xBEEF

enum CONTROL_MSG_TYPE {
	CONTROL_MSG_TYPE_REQUEST = 0x00,
	CONTROL_MSG_TYPE_RESPONSE,
	CONTROL_MSG_TYPE_INVALID,
};

enum CONTROL_MSG_COMMAND {
    CONTROL_MSG_COMMAND_CALIBRATE = 0x00,
	CONTROL_MSG_COMMAND_MOVE_JOINT_1_LEFT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_1_RIGHT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_2_LEFT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_2_RIGHT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_3_LEFT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_3_RIGHT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_4_LEFT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_4_RIGHT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_5_LEFT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_5_RIGHT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_6_LEFT,
	CONTROL_MSG_COMMAND_MOVE_JOINT_6_RIGHT,
	CONTROL_MSG_COMMAND_INVALID,
};

enum CONTROL_MSG_RESPONSE {
	CONTROL_MSG_RESPONSE_SUCCESS = 0x00,
	CONTROL_MSG_RESPONSE_LIMIT_BUTTON_PRESSED,
	CONTROL_MSG_RESPONSE_INVALID,
};

struct control_msg_header {
    uint8_t msg_type;
	uint8_t slave_id;
} __attribute__((packed));

struct control_request_msg {
	struct control_msg_header header;
    uint8_t command;
} __attribute__((packed));

struct control_response_msg {
	struct control_msg_header header;
	uint8_t response;
} __attribute__((packed));

int send_control_msg_request(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
			 const uint8_t target_mac_addr[ETH_ALEN], uint8_t slave_id, uint8_t command);
int send_control_msg_response(const struct spi_dt_spec *spi, const uint8_t my_mac_addr[ETH_ALEN],
			 const uint8_t target_mac_addr[ETH_ALEN], uint8_t slave_id, uint8_t response);

#endif /* CONTROL_MSG_H */