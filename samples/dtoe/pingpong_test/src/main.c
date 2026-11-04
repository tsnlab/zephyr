/*
 * Pingpong test, replaces examples/hello_dtoe/main.c
 *
 * FastDDS app writes pings on "pingTopic", DTOE writes a pong on "pongTopic" for every ping it takes.
 * Counterpart: test/performance/pingpong_test/fastdds
 *
 * ping: str = "ping", count = 1..total, angle = total ping count of the run
 * pong: str = "pong", count and angle copied from the ping
 *
 * A ping with count 0 is a probe: FastDDS app writes probes until one comes back, to check DTOE side is ready.
 * DTOE replies to probes but does not count them.
 *
 * A run ends when the last ping (count == total) is taken, or when no ping is taken for RUN_IDLE_TIMEOUT_US.
 * Then DTOE prints how many pings were lost, out of order or duplicated, and waits for the next run.
 */
#include <pico/types.h>
#include <stdio.h>
#include <string.h>
#include <pico/time.h>

#include "dtoe_host.h"
#include "msg_pingpong.h"


#define TAKE_INTERVAL_US (100)
// ping count of a readiness probe, see test/performance/pingpong_test/fastdds
#define PROBE_COUNT (0)
#define RUN_IDLE_TIMEOUT_US (10 * USEC_PER_SEC)
// counts above this are not checked for duplication
#define MAX_TRACKED_COUNT (65536)

typedef struct ping_stats {
    bool active;
    uint32_t total;         // total ping count of the run, from ping angle
    uint32_t received;
    uint32_t unique;
    uint32_t out_of_order;  // count smaller than the largest count taken before
    uint32_t duplicated;
    uint32_t max_count;
    uint32_t pong_written;
    uint32_t pong_failed;
    absolute_time_t last_ping_time;
} ping_stats_t;

static uint8_t seen_bitmap[MAX_TRACKED_COUNT / 8];

static void start_run(ping_stats_t* stats) {
    *stats = (ping_stats_t) {
        .active = true,
    };
    memset(seen_bitmap, 0, sizeof(seen_bitmap));
    printf("[test] run started\n");
}

static void record_ping(ping_stats_t* stats, const msg_pingpong_t* ping) {
    uint32_t count = ping->count;

    ++stats->received;
    stats->total = (uint32_t)ping->angle;
    if (count < stats->max_count) {
        ++stats->out_of_order;
    } else {
        stats->max_count = count;
    }
    if (count < MAX_TRACKED_COUNT) {
        uint8_t mask = 1 << (count % 8);

        if (seen_bitmap[count / 8] & mask) {
            ++stats->duplicated;
            return;
        }
        seen_bitmap[count / 8] |= mask;
    }
    ++stats->unique;
}

static void finish_run(ping_stats_t* stats, const char* reason) {
    // if total is unknown, expect up to the largest count taken
    uint32_t expected = stats->total > 0 ? stats->total : stats->max_count;
    uint32_t lost = expected > stats->unique ? expected - stats->unique : 0;

    printf("\n[test] ===== pingpong result (DTOE side, ping) =====\n");
    printf("[test] run ended by\t= %s\n", reason);
    printf("[test] expected\t\t= %lu\n", (unsigned long)expected);
    printf("[test] received\t\t= %lu\n", (unsigned long)stats->received);
    printf("[test] lost\t\t= %lu\n", (unsigned long)lost);
    printf("[test] out of order\t= %lu\n", (unsigned long)stats->out_of_order);
    printf("[test] duplicated\t= %lu\n", (unsigned long)stats->duplicated);
    printf("[test] pong written\t= %lu (write failed %lu)\n\n", (unsigned long)stats->pong_written,
           (unsigned long)stats->pong_failed);
    stats->active = false;
}

int main(void) {
    dtoe_handle_t handle;
    dtoe_retcode_t ret;
    ping_stats_t stats = {
        .active = false,
    };

    // Initialize DTOE and DDS entities
    ret = dtoe_init(&handle);
    if (ret != RET_OK) {
        printf("failed to initialize DTOE: %d\n", ret);
        return 1;
    }
    ret = dtoe_writer_create(&handle, "pongTopic", &msg_pingpong, RELIABILITY_BEST_EFFORT);
    if (ret != RET_OK) {
        printf("failed to create writer: %d\n", ret);
        return 1;
    }
    ret = dtoe_reader_create(&handle, "pingTopic", &msg_pingpong, RELIABILITY_BEST_EFFORT);
    if (ret != RET_OK) {
        printf("failed to create reader: %d\n", ret);
        return 1;
    }
    printf("\n[test] pingpong: waiting for pings\n");

    while (1) {
        msg_pingpong_t ping;
        msg_pingpong_t pong;
        uint16_t received_len;

        ret = dtoe_take(&handle, &ping, sizeof(ping), &received_len);
        if (ret == RET_OK) {
            // reply regardless of stats, pong carries the ping count back
            memset(&pong, 0, sizeof(pong));
            strncpy(pong.str, "pong", MSG_PINGPONG_STR_LEN - 1);
            pong.count = ping.count;
            pong.angle = ping.angle;

            if (ping.count == PROBE_COUNT) {
                // probe: reply only, not a part of the run
                dtoe_write(&handle, &pong, sizeof(pong));
                continue;
            }
            if (!stats.active) {
                start_run(&stats);
            }
            record_ping(&stats, &ping);
            stats.last_ping_time = get_absolute_time();

            if (dtoe_write(&handle, &pong, sizeof(pong)) == RET_OK) {
                ++stats.pong_written;
            } else {
                // e.g. no matched remote reader
                ++stats.pong_failed;
            }
            if (stats.total > 0 && ping.count >= stats.total) {
                finish_run(&stats, "last ping");
            }
            continue;
        } else if (ret != RET_NO_DATA && ret != RET_PRECONDITION_NOT_MET) {
            printf("[test] take failed: %d\n", ret);
        }
        if (stats.active && absolute_time_diff_us(stats.last_ping_time, get_absolute_time()) > RUN_IDLE_TIMEOUT_US) {
            finish_run(&stats, "idle timeout");
        }
        sleep_us(TAKE_INTERVAL_US);
    }
}
