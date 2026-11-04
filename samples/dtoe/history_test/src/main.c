/*
 * History (KEEP_LAST) test of DTOE reader, replaces examples/hello_dtoe/main.c
 *
 * DTOE reader history keeps the last DTOE_SAMPLE_COUNT samples. FastDDS app writes 1, 2 and 3 samples
 * on "historyDataTopic" in turn, and DTOE takes them in a take round.
 * Counterpart: test/functional/history_test/fastdds
 *
 * Every TAKE_ROUND_INTERVAL_US, DTOE takes until RET_NO_DATA and writes one report on "historyReportTopic":
 *   str = counts of the taken samples in taken order, e.g. "22,23" (empty if nothing taken)
 *   count = number of taken samples
 *   angle = round number
 * FastDDS app writes the samples of a case right after a report, so that they are taken in the next round.
 */
#include <pico/types.h>
#include <stdio.h>
#include <string.h>
#include <pico/time.h>

#include "dtoe_host.h"
#include "msg_pingpong.h"


#define TAKE_ROUND_INTERVAL_US (2 * USEC_PER_SEC)
// more than any case writes, so that a round always ends with RET_NO_DATA
#define MAX_TAKE_PER_ROUND (8)

int main(void) {
    dtoe_handle_t handle;
    dtoe_retcode_t ret;
    uint32_t round = 0;

    // Initialize DTOE and DDS entities
    ret = dtoe_init(&handle);
    if (ret != RET_OK) {
        printf("failed to initialize DTOE: %d\n", ret);
        return 1;
    }
    ret = dtoe_writer_create(&handle, "historyReportTopic", &msg_pingpong, RELIABILITY_RELIABLE);
    if (ret != RET_OK) {
        printf("failed to create writer: %d\n", ret);
        return 1;
    }
    ret = dtoe_reader_create(&handle, "historyDataTopic", &msg_pingpong, RELIABILITY_RELIABLE);
    if (ret != RET_OK) {
        printf("failed to create reader: %d\n", ret);
        return 1;
    }
    printf("\n[test] history test: take round every %llu ms, reader history depth %d\n",
           (unsigned long long)(TAKE_ROUND_INTERVAL_US / USEC_PER_MSEC), DTOE_SAMPLE_COUNT);

    absolute_time_t next_round = make_timeout_time_us(TAKE_ROUND_INTERVAL_US);
    while (1) {
        msg_pingpong_t report;
        uint32_t taken = 0;
        size_t used = 0;

        sleep_until(next_round);
        next_round = delayed_by_us(next_round, TAKE_ROUND_INTERVAL_US);
        ++round;

        memset(&report, 0, sizeof(report));
        for (int32_t i = 0; i < MAX_TAKE_PER_ROUND; ++i) {
            msg_pingpong_t sample;
            uint16_t received_len;

            ret = dtoe_take(&handle, &sample, sizeof(sample), &received_len);
            if (ret != RET_OK) {
                // RET_NO_DATA: history is empty, RET_PRECONDITION_NOT_MET: no matched remote writer
                break;
            }
            if (taken == 0) {
                printf("[test] round %lu\n", (unsigned long)round);
            }
            printf("\ttake %lu: count=%lu\n", (unsigned long)taken + 1, (unsigned long)sample.count);
            // list of taken counts, e.g. "21,22,23", truncated if it does not fit
            used += snprintf(report.str + used, used < MSG_PINGPONG_STR_LEN ? MSG_PINGPONG_STR_LEN - used : 0,
                             taken == 0 ? "%lu" : ",%lu", (unsigned long)sample.count);
            ++taken;
        }
        if (taken > 0) {
            printf("\ttake %lu: %s\n", (unsigned long)taken + 1, ret == RET_NO_DATA ? "no data" : "stopped");
        }

        report.count = taken;
        report.angle = (float)round;
        ret = dtoe_write(&handle, &report, sizeof(report));
        if (ret != RET_OK && ret != RET_PRECONDITION_NOT_MET) {
            // RET_PRECONDITION_NOT_MET: FastDDS app is not running yet
            printf("[test] round %lu: report write failed: %d\n", (unsigned long)round, ret);
        }
    }
}
