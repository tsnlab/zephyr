/*
 * Data transfer test, replaces examples/hello_dtoe/main.c
 *
 * phase 1: DTOE -> FastDDS, write 10 msg_joint_state samples on "joint_state_topic"
 * phase 2: FastDDS -> DTOE, take 10 msg_joint_setpoint samples on "joint_setpoint_topic"
 *
 * Every sample has different values, generated from its index. The receiver generates the same values
 * and compares them with the received sample. Counterpart: test/functional/data_transfer_test/fastdds
 */
#include <pico/types.h>
#include <stdio.h>
#include <pico/time.h>

#include "dtoe_host.h"
#include "msg_joint_state.h"
#include "msg_joint_setpoint.h"


#define SAMPLE_COUNT (10)
#define WRITE_INTERVAL_US (100 * USEC_PER_MSEC)
#define ACK_TIMEOUT_US (5 * USEC_PER_SEC)
#define RECEIVE_TIMEOUT_US (60 * USEC_PER_SEC)
#define TAKE_INTERVAL_US (USEC_PER_MSEC)

/*
 * Expected values, must be the same as test/functional/data_transfer_test/fastdds/src/data_transfer_test.cpp
 * All float values are exactly representable, so received values are compared with ==.
 */
static msg_joint_state_t make_joint_state(uint32_t index) {
    return (msg_joint_state_t) {
        .position = 1.5f * index + 0.25f,
        .velocity = -2.0f * index - 0.5f,
        .effort = 100.0f + 0.125f * index,
        .status = 0x10 + index,
        .flags = 0xA0 ^ index,
        .error_code = 0xFF - index,
    };
}

static msg_joint_setpoint_t make_joint_setpoint(uint32_t index) {
    return (msg_joint_setpoint_t) {
        .target_position = 0.75f * index - 3.0f,
        .target_velocity = 1000.0f - 8.5f * index,
        .control_mode = 1 + index,
        .flags = 0x5A ^ index,
    };
}

static bool compare_joint_setpoint(const msg_joint_setpoint_t* expected, const msg_joint_setpoint_t* received) {
    return expected->target_position == received->target_position
           && expected->target_velocity == received->target_velocity
           && expected->control_mode == received->control_mode
           && expected->flags == received->flags;
}

static void print_joint_state(const char* label, const msg_joint_state_t* state) {
    printf("\t%s: position=%f, velocity=%f, effort=%f, status=0x%02hhx, flags=0x%02hhx, error_code=0x%02hhx\n",
           label, state->position, state->velocity, state->effort, state->status, state->flags, state->error_code);
}

static void print_joint_setpoint(const char* label, const msg_joint_setpoint_t* setpoint) {
    printf("\t%s: target_position=%f, target_velocity=%f, control_mode=0x%02hhx, flags=0x%02hhx\n", label,
           setpoint->target_position, setpoint->target_velocity, setpoint->control_mode, setpoint->flags);
}

// return true if acked by all remote readers before timeout
static bool wait_acked_by_all(dtoe_handle_t* handle) {
    absolute_time_t timeout = make_timeout_time_us(ACK_TIMEOUT_US);
    bool acked = false;

    while (get_absolute_time() < timeout) {
        if (dtoe_is_acked_by_all(handle, &acked) == RET_OK && acked) {
            return true;
        }
        sleep_us(TAKE_INTERVAL_US);
    }
    return false;
}

// phase 1: return number of samples written
static uint32_t run_phase_1(dtoe_handle_t* handle, uint32_t* acked_count) {
    uint32_t written = 0;
    dtoe_retcode_t ret;

    printf("\n[test] phase 1: DTOE -> FastDDS, waiting for remote reader\n");
    *acked_count = 0;
    while (written < SAMPLE_COUNT) {
        msg_joint_state_t state = make_joint_state(written);

        ret = dtoe_write(handle, &state, sizeof(state));
        if (ret == RET_PRECONDITION_NOT_MET) {
            // remote data reader does not exist yet, retry the same sample
            sleep_us(WRITE_INTERVAL_US);
            continue;
        } else if (ret != RET_OK) {
            printf("[test] sample %lu: write failed: %d\n", (unsigned long)written, ret);
            break;
        }
        if (wait_acked_by_all(handle)) {
            ++*acked_count;
            printf("[test] sample %lu\n", (unsigned long)written);
        } else {
            printf("[test] sample %lu: ACK timeout\n", (unsigned long)written);
        }
        print_joint_state("sent", &state);
        ++written;
        sleep_us(WRITE_INTERVAL_US);
    }
    return written;
}

// phase 2: return number of samples received
static uint32_t run_phase_2(dtoe_handle_t* handle, uint32_t* mismatch_count) {
    absolute_time_t timeout = make_timeout_time_us(RECEIVE_TIMEOUT_US);
    uint32_t received = 0;

    printf("\n[test] phase 2: FastDDS -> DTOE, waiting for samples\n");
    *mismatch_count = 0;
    while (received < SAMPLE_COUNT && get_absolute_time() < timeout) {
        msg_joint_setpoint_t setpoint;
        msg_joint_setpoint_t expected = make_joint_setpoint(received);
        uint16_t received_len;
        dtoe_retcode_t ret;

        ret = dtoe_take(handle, &setpoint, sizeof(setpoint), &received_len);
        if (ret == RET_PRECONDITION_NOT_MET || ret == RET_NO_DATA) {
            // no matched remote writer, or no data yet
            sleep_us(TAKE_INTERVAL_US);
            continue;
        } else if (ret != RET_OK) {
            printf("[test] take failed: %d\n", ret);
            sleep_us(TAKE_INTERVAL_US);
            continue;
        }
        // received_len is not compared: it is the number of CDR bytes deserialized (10),
        // not sizeof(msg_joint_setpoint_t) (12 with padding)
        if (compare_joint_setpoint(&expected, &setpoint)) {
            printf("[test] sample %lu\n", (unsigned long)received);
            print_joint_setpoint("received", &setpoint);
        } else {
            ++*mismatch_count;
            printf("[test] sample %lu: MISMATCH (len=%hu)\n", (unsigned long)received, received_len);
            print_joint_setpoint("received", &setpoint);
        }
        ++received;
    }
    if (received < SAMPLE_COUNT) {
        printf("[test] receive timeout\n");
    }
    return received;
}

int main(void) {
    dtoe_handle_t handle;
    dtoe_retcode_t ret;
    uint32_t written;
    uint32_t acked;
    uint32_t received;
    uint32_t mismatched;

    // Initialize DTOE and DDS entities
    ret = dtoe_init(&handle);
    if (ret != RET_OK) {
        printf("failed to initialize DTOE: %d\n", ret);
        return 1;
    }
    ret = dtoe_writer_create(&handle, "joint_state_topic", &msg_joint_state, RELIABILITY_RELIABLE);
    if (ret != RET_OK) {
        printf("failed to create writer: %d\n", ret);
        return 1;
    }
    ret = dtoe_reader_create(&handle, "joint_setpoint_topic", &msg_joint_setpoint, RELIABILITY_RELIABLE);
    if (ret != RET_OK) {
        printf("failed to create reader: %d\n", ret);
        return 1;
    }

    written = run_phase_1(&handle, &acked);
    received = run_phase_2(&handle, &mismatched);

    printf("\n[test] ===== data transfer test result =====\n");
    printf("[test] phase 1: written %lu/%d, acked %lu/%d\n", (unsigned long)written, SAMPLE_COUNT,
           (unsigned long)acked, SAMPLE_COUNT);
    printf("[test] phase 2: received %lu/%d, mismatched %lu\n", (unsigned long)received, SAMPLE_COUNT,
           (unsigned long)mismatched);
    printf("[test] DTOE side %s (phase 1 integrity is checked by FastDDS app)\n\n",
           (acked == SAMPLE_COUNT && received == SAMPLE_COUNT && mismatched == 0) ? "PASS" : "FAIL");

    // keep DTOE running so that FastDDS app can finish
    while (1) {
        sleep_ms(1000);
    }
}
