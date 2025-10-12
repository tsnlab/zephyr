/*
 * Copyright (c) 2025 TSN Lab
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include "../lib/lan8651.h"
#include "../lib/eth.h"
#include "../lib/control_msg.h"






/*#########################################################################################
#                                    Define and Macro                                     #
##########################################################################################*/
#define CHECK_INIT_GPIO(ret) \
    if (ret < 0) { \
        return -1; \
    }

#define LOW     0
#define HIGH    1

#define PULSE_COUNT 20  /* one step is 20 pulses */

#define JOINTS_NODE     DT_ALIAS(joints0)
#define BUTTONS_NODE    DT_ALIAS(buttons0)
#define SPI_NODE        DT_ALIAS(spi0)

#define SLAVE_ID 1





/*#########################################################################################
#                                    Enum and Struct                                      #
##########################################################################################*/
enum JOINT_DIRECTION {
    LEFT,
    RIGHT,
};

enum JOINT_NUM {
    JOINT_1,
    JOINT_2,
    JOINT_3,
    JOINT_4,
    JOINT_5,
    JOINT_6,
};

struct joint {
    struct gpio_dt_spec pulse;
    struct gpio_dt_spec direction;
    struct gpio_dt_spec limit_button;
};

enum ROBOT_ARM_STATE {
    NOT_READY,
    CALIBRATING,
    READY,
    RUNNING,
};

struct robot_arm {
    struct joint joint1;
    struct joint joint2;
    struct joint joint3;
    struct joint joint4;
    struct joint joint5;
    struct joint joint6;

    enum ROBOT_ARM_STATE state;
};





/*#########################################################################################
#                                    Global variables                                      #
##########################################################################################*/

/* Initialize joint GPIO specifications from device tree for global_robot_arm */
static const struct gpio_dt_spec joint1_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 0);
static const struct gpio_dt_spec joint1_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 1);
static const struct gpio_dt_spec joint1_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 0);
static const struct gpio_dt_spec joint2_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 2);
static const struct gpio_dt_spec joint2_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 3);
static const struct gpio_dt_spec joint2_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 1);
static const struct gpio_dt_spec joint3_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 4);
static const struct gpio_dt_spec joint3_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 5);
static const struct gpio_dt_spec joint3_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 2);
static const struct gpio_dt_spec joint4_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 6);
static const struct gpio_dt_spec joint4_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 7);
static const struct gpio_dt_spec joint4_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 3);
static const struct gpio_dt_spec joint5_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 8);
static const struct gpio_dt_spec joint5_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 9);
static const struct gpio_dt_spec joint5_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 4);
static const struct gpio_dt_spec joint6_pulse = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 10);
static const struct gpio_dt_spec joint6_direction = GPIO_DT_SPEC_GET_BY_IDX(JOINTS_NODE, gpios, 11);
static const struct gpio_dt_spec joint6_limit_button = GPIO_DT_SPEC_GET_BY_IDX(BUTTONS_NODE, gpios, 5);

/* Global joint structures */
static struct joint joint1;
static struct joint joint2;
static struct joint joint3;
static struct joint joint4;
static struct joint joint5;
static struct joint joint6;

/* Global robot arm structure */
static struct robot_arm global_robot_arm;

/* Global 10Base-T1S device */
static struct spi_dt_spec t1s_spi_dev;

/* My MAC address */
static const uint8_t my_mac_addr[ETH_ALEN] = { 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, };
/* Target MAC address */
static const uint8_t target_mac_addr[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, };


/*#########################################################################################
#                                    Prototype                                            #
##########################################################################################*/
static int initialize(void);
static int init_gpio(const struct gpio_dt_spec* gpio, gpio_flags_t flags);
static void init_t1s(uint32_t frequency, spi_operation_t operation, enum PLCA_MODE mode);

static void do_calibration(void);

static bool move_joint(enum JOINT_NUM joint_num, enum JOINT_DIRECTION direction);

static bool move_joint_1_one_step(enum JOINT_DIRECTION direction);
static bool move_joint_2_one_step(enum JOINT_DIRECTION direction);
static bool move_joint_3_one_step(enum JOINT_DIRECTION direction);
static bool move_joint_4_one_step(enum JOINT_DIRECTION direction);
static bool move_joint_5_one_step(enum JOINT_DIRECTION direction);
static bool move_joint_6_one_step(enum JOINT_DIRECTION direction);

static void common_move_joint(enum JOINT_NUM joint_num);
static bool check_joint_limit_button(enum JOINT_NUM joint_num);





/*#########################################################################################
#                                    Main                                                 #
##########################################################################################*/
int main(void)
{
    uint8_t recv_buff[MAX_PACKET_SIZE] = { '\0', };
    uint16_t length = 0;

    struct ethhdr *eth;
    struct control_request_msg *control_req_msg;

    bool move_joint_ret = false;

    if (initialize() < 0) {
        return 0;
    }

    while (1) {
        while (length == 0) {
            if (receive_packet(&t1s_spi_dev, recv_buff, &length) < 0) {
                printk("Failed to receive control message\n");
                return -1;
            }
        }

        eth = (struct ethhdr *)recv_buff;
        control_req_msg = (struct control_request_msg *)(eth + 1);

        if (eth->h_proto != sys_cpu_to_be16(CONTROL_MSG_PROTO_TYPE)) {
            continue;
        }

        switch (control_req_msg->header.msg_type) {
            case CONTROL_MSG_TYPE_REQUEST: {
                switch (control_req_msg->command) {

                    /* Calibration Commands */
                    case CONTROL_MSG_COMMAND_CALIBRATE:
                        do_calibration();
                        move_joint_ret = true;
                        break;

                    /* Joint 1 Commands */
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_1_LEFT:
                        move_joint_ret = move_joint(JOINT_1, LEFT);
                        break;
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_1_RIGHT:
                        move_joint_ret = move_joint(JOINT_1, RIGHT);
                        break;

                    /* Joint 2 Commands */
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_2_LEFT:
                        move_joint_ret = move_joint(JOINT_2, LEFT);
                        break;
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_2_RIGHT:
                        move_joint_ret = move_joint(JOINT_2, RIGHT);
                        break;

                    /* Joint 3 Commands */
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_3_LEFT:
                        move_joint_ret = move_joint(JOINT_3, LEFT);
                        break;
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_3_RIGHT:
                        move_joint_ret = move_joint(JOINT_3, RIGHT);
                        break;

                    /* Joint 4 Commands */
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_4_LEFT:
                        move_joint_ret = move_joint(JOINT_4, LEFT);
                        break;
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_4_RIGHT:
                        move_joint_ret = move_joint(JOINT_4, RIGHT);
                        break;

                    /* Joint 5 Commands */
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_5_LEFT:
                        move_joint_ret = move_joint(JOINT_5, LEFT);
                        break;
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_5_RIGHT:
                        move_joint_ret = move_joint(JOINT_5, RIGHT);
                        break;

                    /* Joint 6 Commands */
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_6_LEFT:
                        move_joint_ret = move_joint(JOINT_6, LEFT);
                        break;
                    case CONTROL_MSG_COMMAND_MOVE_JOINT_6_RIGHT:
                        move_joint_ret = move_joint(JOINT_6, RIGHT);
                        break;

                    default:
                        printk("Invalid control command.\n");
                        move_joint_ret = false;
                        break;
                }

                send_control_msg_response(&t1s_spi_dev, my_mac_addr, target_mac_addr, SLAVE_ID, 
                    move_joint_ret ? CONTROL_MSG_RESPONSE_SUCCESS : CONTROL_MSG_RESPONSE_LIMIT_BUTTON_PRESSED);
                break;
            }
            case CONTROL_MSG_TYPE_RESPONSE:
                printk("Slave node doesn't support the control response message.\n");
                break;
            default:
                printk("Invalid control message type.\n");
                break;
        }

        /* Clear the received buffer */
        memset(recv_buff, '\0', MAX_PACKET_SIZE);
        length = 0;
    }

    return 0;
}











/*#########################################################################################
#                                    Function                                             #
##########################################################################################*/
int initialize(void)
{
    printk("Do Initialization\n");

    /* Initialize all joint GPIOs as outputs */
    CHECK_INIT_GPIO(init_gpio(&joint1_pulse, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint1_direction, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint2_pulse, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint2_direction, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint3_pulse, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint3_direction, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint4_pulse, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint4_direction, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint5_pulse, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint5_direction, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint6_pulse, GPIO_OUTPUT));
    CHECK_INIT_GPIO(init_gpio(&joint6_direction, GPIO_OUTPUT));

    /* Initialize limit button GPIOs as inputs */
    CHECK_INIT_GPIO(init_gpio(&joint1_limit_button, GPIO_INPUT));
    CHECK_INIT_GPIO(init_gpio(&joint2_limit_button, GPIO_INPUT));
    CHECK_INIT_GPIO(init_gpio(&joint3_limit_button, GPIO_INPUT));
    CHECK_INIT_GPIO(init_gpio(&joint4_limit_button, GPIO_INPUT));
    CHECK_INIT_GPIO(init_gpio(&joint5_limit_button, GPIO_INPUT));
    CHECK_INIT_GPIO(init_gpio(&joint6_limit_button, GPIO_INPUT));

    /* Initialize joint and robot_arm structures */
    joint1 = (struct joint){
        .pulse = joint1_pulse,
        .direction = joint1_direction,
        .limit_button = joint1_limit_button,
    };
    joint2 = (struct joint){
        .pulse = joint2_pulse,
        .direction = joint2_direction,
        .limit_button = joint2_limit_button,
    };
    joint3 = (struct joint){
        .pulse = joint3_pulse,
        .direction = joint3_direction,
        .limit_button = joint3_limit_button,
    };
    joint4 = (struct joint){
        .pulse = joint4_pulse,
        .direction = joint4_direction,
        .limit_button = joint4_limit_button,
    };
    joint5 = (struct joint){
        .pulse = joint5_pulse,
        .direction = joint5_direction,
        .limit_button = joint5_limit_button,
    };
    joint6 = (struct joint){
        .pulse = joint6_pulse,
        .direction = joint6_direction,
        .limit_button = joint6_limit_button,
    };
    global_robot_arm = (struct robot_arm){
        .joint1 = joint1,
        .joint2 = joint2,
        .joint3 = joint3,
        .joint4 = joint4,
        .joint5 = joint5,
        .joint6 = joint6,
        .state = NOT_READY,
    };

    printk("Robot Arm initialized\n");

    /* Initialize 10Base-T1S Module (lan8651) */
    uint32_t frequency = 25000000;
    spi_operation_t operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(32) | SPI_HOLD_ON_CS;
    enum PLCA_MODE mode = PLCA_MODE_FOLLOWER; /* PLCA_MODE_COORDINATOR or PLCA_MODE_FOLLOWER */
    init_t1s(frequency, operation, mode);

    printk("10Base-T1S initialized\n");

    printk("Initialization Done\n");

    return 0;
}

int init_gpio(const struct gpio_dt_spec* gpio, gpio_flags_t flags)
{
    int ret;

    if (!gpio_is_ready_dt(gpio)) {
        printk("Error: GPIO %s is not ready\n",
               gpio->port->name);
        return -1;
    }

    ret = gpio_pin_configure_dt(gpio, flags);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, gpio->port->name, gpio->pin);
        return -1;
    } else {
        printk("Set up GPIO at %s pin %d\n", gpio->port->name, gpio->pin);
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

void do_calibration(void)
{
    printk("Do Calibration\n");

    global_robot_arm.state = CALIBRATING;

    printk("1. Calibrate Joint 1\n");
    while(1) {
        move_joint(JOINT_1, LEFT);
        if (check_joint_limit_button(JOINT_1)) {
            break;
        }
    }

    printk("2. Calibrate Joint 2\n");
    while(1) {
        move_joint(JOINT_2, LEFT);
        if (check_joint_limit_button(JOINT_2)) {
            break;
        }
    }

    printk("3. Calibrate Joint 3\n");
    while(1) {
        move_joint(JOINT_3, RIGHT);
        if (check_joint_limit_button(JOINT_3)) {
            break;
        }
    }

    printk("4. Move Joint 3 to Left N degree\n");
    for(int i = 0; i < 50; i++) {
        move_joint(JOINT_3, LEFT);
    }

    printk("5. Calibrate Joint 4\n");
    while(1) {
        move_joint(JOINT_4, LEFT);
        if (check_joint_limit_button(JOINT_4)) {
            break;
        }
    }

    global_robot_arm.state = READY;

    printk("Calibration Done\n");
}

bool move_joint(enum JOINT_NUM joint_num, enum JOINT_DIRECTION direction)
{
    bool ret = false;
    
    global_robot_arm.state = RUNNING;
    switch (joint_num) {
        case JOINT_1:
            ret = move_joint_1_one_step(direction);
            break;
        case JOINT_2:
            ret = move_joint_2_one_step(direction);
            break;
        case JOINT_3:
            ret = move_joint_3_one_step(direction);
            break;
        case JOINT_4:
            ret = move_joint_4_one_step(direction);
            break;
        case JOINT_5:
            ret = move_joint_5_one_step(direction);
            break;
        case JOINT_6:
            ret = move_joint_6_one_step(direction);
            break;
        default:
            printk("Invalid joint number: %d\n", joint_num);
            break;
    }
    global_robot_arm.state = READY;

    return ret;
}

bool move_joint_1_one_step(enum JOINT_DIRECTION direction)
{
    switch (direction) {
        case LEFT: {
            if (check_joint_limit_button(JOINT_1)) {
                printk("Joint 1 Limit Button Pressed\n");
                return false;
            }
            gpio_pin_set_dt(&global_robot_arm.joint1.direction, LOW);
            break;
        }
        case RIGHT: {
            gpio_pin_set_dt(&global_robot_arm.joint1.direction, HIGH);
            break;
        }
    }

    common_move_joint(JOINT_1);
    return true;
}

bool move_joint_2_one_step(enum JOINT_DIRECTION direction)
{
    switch (direction) {
        case LEFT: {
            if (check_joint_limit_button(JOINT_2)) {
                printk("Joint 2 Limit Button Pressed\n");
                return false;
            }
            gpio_pin_set_dt(&global_robot_arm.joint2.direction, LOW);
            break;
        }
        case RIGHT: {
            gpio_pin_set_dt(&global_robot_arm.joint2.direction, HIGH);
            break;
        }
    }

    common_move_joint(JOINT_2);
    return true;
}

bool move_joint_3_one_step(enum JOINT_DIRECTION direction)
{
    switch (direction) {
        case LEFT: {
            gpio_pin_set_dt(&global_robot_arm.joint3.direction, LOW);
            break;
        }
        case RIGHT: {
            if (check_joint_limit_button(JOINT_3)) {
                printk("Joint 3 Limit Button Pressed\n");
                return false;
            }
            gpio_pin_set_dt(&global_robot_arm.joint3.direction, HIGH);
            break;
        }
    }

    common_move_joint(JOINT_3);
    return true;
}

bool move_joint_4_one_step(enum JOINT_DIRECTION direction)
{
    switch (direction) {
        case LEFT: {
            if (check_joint_limit_button(JOINT_4)) {
                printk("Joint 4 Limit Button Pressed\n");
                return false;
            }
            gpio_pin_set_dt(&global_robot_arm.joint4.direction, LOW);
            break;
        }
        case RIGHT: {
            gpio_pin_set_dt(&global_robot_arm.joint4.direction, HIGH);
            break;
        }
    }

    common_move_joint(JOINT_4);
    return true;
}

bool move_joint_5_one_step(enum JOINT_DIRECTION direction)
{
    printk("Not supported\n");
    return true;

    switch (direction) {
        case LEFT: {
            if (check_joint_limit_button(JOINT_5)) {
                printk("Joint 5 Limit Button Pressed\n");
                return false;
            }
            gpio_pin_set_dt(&global_robot_arm.joint5.direction, LOW);
            break;
        }
        case RIGHT: {
            gpio_pin_set_dt(&global_robot_arm.joint5.direction, HIGH);
            break;
        }
    }

    common_move_joint(JOINT_5);
    return true;
}

bool move_joint_6_one_step(enum JOINT_DIRECTION direction)
{
    printk("Not supported\n");
    return true;

    switch (direction) {
        case LEFT: {
            gpio_pin_set_dt(&global_robot_arm.joint6.direction, LOW);
            break;
        }
        case RIGHT: {
            gpio_pin_set_dt(&global_robot_arm.joint6.direction, HIGH);
            break;
        }
    }

    common_move_joint(JOINT_6);
    return true;
}

void common_move_joint(enum JOINT_NUM joint_num)
{

    /* NOTE:
     *   - The first delay is the minimum delay required to generate pulses for the stepper motor.
     *   - The second delay controls the speed of the stepper motor. (This value differs for each joint.)
     *   - (ToDo) For the second delay, an API to calculate the appropriate speed should be implemented.
     */

    switch (joint_num) {
        case JOINT_1: {
            for(int i = 0; i < PULSE_COUNT * 2; i++) {
                gpio_pin_set_dt(&global_robot_arm.joint1.pulse, LOW);
                k_busy_wait(100); // first delay
                gpio_pin_set_dt(&global_robot_arm.joint1.pulse, HIGH);
                k_busy_wait(250); // second delay
            }
            break;
        }
        case JOINT_2: {
            for(int i = 0; i < PULSE_COUNT; i++) {
                gpio_pin_set_dt(&global_robot_arm.joint2.pulse, LOW);
                k_busy_wait(100); // first delay
                gpio_pin_set_dt(&global_robot_arm.joint2.pulse, HIGH);
                k_busy_wait(500); // second delay
            }
            break;
        }
        case JOINT_3: {
            for(int i = 0; i < PULSE_COUNT * 2; i++) {
                gpio_pin_set_dt(&global_robot_arm.joint3.pulse, LOW);
                k_busy_wait(100); // first delay
                gpio_pin_set_dt(&global_robot_arm.joint3.pulse, HIGH);
                k_busy_wait(250); // second delay
            }
            break;
        }
        case JOINT_4: {
            for(int i = 0; i < PULSE_COUNT; i++) {
                gpio_pin_set_dt(&global_robot_arm.joint4.pulse, LOW);
                k_busy_wait(100); // first delay
                gpio_pin_set_dt(&global_robot_arm.joint4.pulse, HIGH);
                k_busy_wait(500); // second delay
            }
            break;
        }
        case JOINT_5: {
            for(int i = 0; i < PULSE_COUNT; i++) {
                gpio_pin_set_dt(&global_robot_arm.joint5.pulse, LOW);
                k_busy_wait(100); // first delay
                gpio_pin_set_dt(&global_robot_arm.joint5.pulse, HIGH);
                k_busy_wait(500); // second delay
            }
            break;
        }
        case JOINT_6: {
            for(int i = 0; i < PULSE_COUNT; i++) {
                gpio_pin_set_dt(&global_robot_arm.joint6.pulse, LOW);
                k_busy_wait(100); // first delay
                gpio_pin_set_dt(&global_robot_arm.joint6.pulse, HIGH);
                k_busy_wait(500); // second delay	
            }
            break;
        }
    }
}

bool check_joint_limit_button(enum JOINT_NUM joint_num)
{
    switch (joint_num) {
        case JOINT_1:
            return gpio_pin_get_dt(&global_robot_arm.joint1.limit_button) == HIGH;
        case JOINT_2:
            return gpio_pin_get_dt(&global_robot_arm.joint2.limit_button) == HIGH;
        case JOINT_3:
            return gpio_pin_get_dt(&global_robot_arm.joint3.limit_button) == HIGH;
        case JOINT_4:
            return gpio_pin_get_dt(&global_robot_arm.joint4.limit_button) == HIGH;
        case JOINT_5:
            return gpio_pin_get_dt(&global_robot_arm.joint5.limit_button) == HIGH;
        case JOINT_6:
            return gpio_pin_get_dt(&global_robot_arm.joint6.limit_button) == HIGH;
        default:
            printk("Invalid joint number: %d\n", joint_num);
            return false;
    }
}