/*
 * Copyright (c) 2025 TSN Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include "../lib/lan8651.h"
#include "../lib/eth.h"
#include "../lib/control_msg.h"
 
 
/*#########################################################################################
#                                    Define and Macro                                     #
##########################################################################################*/
#define CHECK_INIT_UART(ret) \
    if (ret < 0) { \
        return -1; \
    }

#define SPI_NODE        DT_ALIAS(spi0)



/*#########################################################################################
#                                    Enum and Struct                                      #
##########################################################################################*/
/* Nothing  */



/*#########################################################################################
#                                    Global variables                                      #
##########################################################################################*/

/* Initialize UART1 device for controlling global_robot_arm */
static const struct device *uart1 = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Global 10Base-T1S device */
static struct spi_dt_spec t1s_spi_dev;

/* My MAC address */
static const uint8_t my_mac_addr[ETH_ALEN] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, };
/* Target MAC address */
static const uint8_t target_mac_addr[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, };



/*#########################################################################################
#                                    Prototype                                            #
##########################################################################################*/
static int initialize(void);
static int init_uart(void);
static void init_t1s(uint32_t frequency, spi_operation_t operation, enum PLCA_MODE mode);




/*#########################################################################################
#                                    Main                                                 #
##########################################################################################*/
int main(void)
{
    char uart_char = '\0';

    uint8_t recv_buff[MAX_PACKET_SIZE] = { '\0', };
    uint16_t length = 0;
    struct ethhdr *eth;
    struct control_response_msg *control_resp_msg;

    uint8_t slave_id = 1;
    uint8_t joint_num = 0;

    if (initialize() < 0) {
        return 0;
    }

    while (1) {
        if (uart_poll_in(uart1, &uart_char) < 0) {
            continue;
        }

        /* Send Control Request Message to Slave Node - Ping*/
        {
            switch (uart_char) {
                
                /* Calibration Commands */
                case '0':
                    joint_num = 0xff;
                    printk("Send to Calibration Message\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_CALIBRATE);
                    break;

                /* Joint 1 Commands */
                case '1':
                    joint_num = 1;
                    printk("Joint 1 Direction Left\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_1_LEFT);
                    break;
                case '2':
                    joint_num = 1;  
                    printk("Joint 1 Direction Right\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_1_RIGHT);
                    break;

                /* Joint 2 Commands */
                case 'q':
                    joint_num = 2;
                    printk("Joint 2 Direction Left\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_2_LEFT);
                    break;
                case 'w':
                    joint_num = 2;
                    printk("Joint 2 Direction Right\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_2_RIGHT);
                    break;

                /* Joint 3 Commands */
                case 'a':
                    joint_num = 3;
                    printk("Joint 3 Direction Left\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_3_LEFT);
                    break;
                case 's':
                    joint_num = 3;
                    printk("Joint 3 Direction Right\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_3_RIGHT);
                    break;

                /* Joint 4 Commands */
                case 'z':
                    joint_num = 4;
                    printk("Joint 4 Direction Left\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_4_LEFT);
                    break;
                case 'x':
                    joint_num = 4;
                    printk("Joint 4 Direction Right\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_4_RIGHT);
                    break;

                /* Joint 5 Commands */
                case '3':
                    joint_num = 5;
                    printk("Joint 5 Direction Left\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_5_LEFT);
                    break;
                case '4':
                    joint_num = 5;
                    printk("Joint 5 Direction Right\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_5_RIGHT);
                    break;

                /* Joint 6 Commands */
                case 'e':
                    joint_num = 6;
                    printk("Joint 6 Direction Left\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_6_LEFT);
                    break;
                case 'r':
                    joint_num = 6;
                    printk("Joint 6 Direction Right\n");
                    send_control_msg_request(&t1s_spi_dev, my_mac_addr, target_mac_addr, slave_id, 
                        CONTROL_MSG_COMMAND_MOVE_JOINT_6_RIGHT);
                    break;

                /* Default Commands */
                case '\0':
                    continue;
                default:
                    printk("Invalid input: %c. Commands: 1, 2, q, w, a, s, z, x, 3, 4, e, r, 0\n", uart_char);
                    break;
            }
        }

        /* Receive Control Response Message from Slave Node - Pong*/
        if (uart_char == '0' || uart_char == '1' || uart_char == '2' || uart_char == 'q' || uart_char == 'w' || 
            uart_char == 'a' || uart_char == 's' || uart_char == 'z' || uart_char == 'x' || uart_char == '3' || 
            uart_char == '4' || uart_char == 'e' || uart_char == 'r') 
        {
            while (length == 0) {
                if (receive_packet(&t1s_spi_dev, recv_buff, &length) < 0) {
                    printk("Failed to receive control message\n");
                    return -1;
                }
            }

            /* Parse Control Response Message */
            eth = (struct ethhdr *)recv_buff;
            control_resp_msg = (struct control_response_msg *)(eth + 1);

            /* Check Protocol Type */
            if (eth->h_proto != sys_cpu_to_be16(CONTROL_MSG_PROTO_TYPE)) {
                continue;
            }

            /* Parse Control Response Message */
            if (control_resp_msg->header.msg_type == CONTROL_MSG_TYPE_RESPONSE) {
                switch (control_resp_msg->response) {
                    case CONTROL_MSG_RESPONSE_SUCCESS:
                        printk("Joint %d Control Success\n", joint_num);
                        break;
                    case CONTROL_MSG_RESPONSE_LIMIT_BUTTON_PRESSED:
                        printk("Joint %d Limit Button Pressed\n", joint_num);
                        break;
                    default:
                        printk("Invalid control message response.\n");
                        break;
                }
            }
            else if (control_resp_msg->header.msg_type == CONTROL_MSG_TYPE_INVALID) {
                printk("Master node doesn't support the control request message.\n");
            }
            else {
                printk("Invalid control message type.\n");
            }
        }

        /* Clear */
        memset(recv_buff, '\0', MAX_PACKET_SIZE);
        length = 0;
        uart_char = '\0';
    }

    return 0;
}





/*#########################################################################################
#                                    Function                                             #
##########################################################################################*/
int initialize(void)
{
    printk("Do Initialization\n");

    CHECK_INIT_UART(init_uart());

    printk("UART1 initialized\n");

    /* Initialize 10Base-T1S Module (lan8651) */
    uint32_t frequency = 25000000;
    spi_operation_t operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;
    enum PLCA_MODE mode = PLCA_MODE_COORDINATOR; /* PLCA_MODE_COORDINATOR or PLCA_MODE_FOLLOWER */
    init_t1s(frequency, operation, mode);
    
    printk("10Base-T1S initialized\n");

    printk("Initialization Done\n");

    return 0;
}

int init_uart(void)
{
    if (!device_is_ready(uart1)) {
		printk("Error: UART %s is not ready\n", uart1->name);
        return -1;
    }

    return 0;
}

void init_t1s(uint32_t frequency, spi_operation_t operation, enum PLCA_MODE mode)
{
    t1s_spi_dev.bus = DEVICE_DT_GET(SPI_NODE);
    t1s_spi_dev.config.frequency = frequency;
    t1s_spi_dev.config.operation = operation;

    set_register(&t1s_spi_dev, mode);
}