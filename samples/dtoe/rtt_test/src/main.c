/*
 * RTT test, replaces examples/hello_dtoe/main.c
 *
 * FastDDS app writes pings on "rttPingTopic" and measures RTT with pongs on "rttPongTopic".
 * DTOE writes a pong for every ping as soon as it takes it, without logging.
 * Counterpart: test/performance/rtt_test/fastdds
 *
 * The agent polling interval is a board setting (engine.polling_interval_us in board.yaml), one firmware for all.
 *
 * ping: str = "ping", count = 1..ping count
 * pong: str = "pong", count copied from the ping, angle = agent polling interval of this board (us)
 */
#include <pico/types.h>
#include <stdio.h>
#include <string.h>
#include <pico/time.h>

#include "dtoe_host.h"
#include "msg_pingpong.h"


int main(void) {
    dtoe_handle_t handle;
    dtoe_board_config_t board;
    dtoe_retcode_t ret;
    float polling_interval_us;

    // Initialize DTOE and DDS entities
    ret = dtoe_init(&handle);
    if (ret != RET_OK) {
        printf("failed to initialize DTOE: %d\n", ret);
        return 1;
    }
    dtoe_get_config(&handle, &board);
    polling_interval_us = (float)board.polling_interval_us;
    ret = dtoe_writer_create(&handle, "rttPongTopic", &msg_pingpong, RELIABILITY_BEST_EFFORT);
    if (ret != RET_OK) {
        printf("failed to create writer: %d\n", ret);
        return 1;
    }
    ret = dtoe_reader_create(&handle, "rttPingTopic", &msg_pingpong, RELIABILITY_BEST_EFFORT);
    if (ret != RET_OK) {
        printf("failed to create reader: %d\n", ret);
        return 1;
    }
    printf("\n[test] rtt test: agent polling interval %lu us, replying pongs\n",
           (unsigned long)board.polling_interval_us);

    // no sleep and no log in this loop, so that only the agent polling interval differs between firmwares.
    // dtoe_take() waits for the agent to process the command, so this loop is paced by the agent.
    while (1) {
        msg_pingpong_t ping;
        msg_pingpong_t pong;
        uint16_t received_len;

        if (dtoe_take(&handle, &ping, sizeof(ping), &received_len) != RET_OK) {
            continue;
        }
        memset(&pong, 0, sizeof(pong));
        strncpy(pong.str, "pong", MSG_PINGPONG_STR_LEN - 1);
        pong.count = ping.count;
        pong.angle = polling_interval_us;
        dtoe_write(&handle, &pong, sizeof(pong));
    }
}
