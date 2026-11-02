#include <stdio.h>

#include <zephyr/kernel.h>

#include "dtoe_host.h"
#include "msg_joint_state.h"
#include "msg_joint_setpoint.h"

#define LOOP_INTERVAL_US (2 * USEC_PER_MSEC)
#define WAIT_ACK_INTERVAL_US (USEC_PER_MSEC)

static void print_joint_state(msg_joint_state_t* state) {
    printf("writing state\n");
    printf("\tposition=\t%f\n\tvelocity=\t%f\n", state->position, state->velocity);
    printf("\teffort=\t\t%f\n\tstatus=\t\t0x%02hhx\n", state->effort, state->status);
    printf("\tflags=\t\t0x%02hhx\n\terror_code=\t0x%02hhx\n\n", state->flags, state->error_code);
}

static void print_joint_setpoint(msg_joint_setpoint_t* setpoint) {
    printf("received setpoint\n");
    printf("\ttarget_position=%f\n\ttarget_velocity=%f\n", setpoint->target_position, setpoint->target_velocity);
    printf("\tcontrol_mode=\t0x%02hhx\n\tflags=\t\t0x%02hhx\n\n", setpoint->control_mode, setpoint->flags);
}

// pico-sdk time functions used by the original, on Zephyr
typedef k_timepoint_t absolute_time_t;
#define make_timeout_time_us(us) sys_timepoint_calc(K_USEC(us))
#define sleep_until(t)           k_sleep(sys_timepoint_timeout(t))

int main(void) {
    dtoe_handle_t handle;
    dtoe_retcode_t ret;

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
        printf("failed to create writer: %d\n", ret);
        return 1;
    }

    float count = 1.0f;

    // Main loop
    while (1) {
        absolute_time_t next_loop_time = make_timeout_time_us(LOOP_INTERVAL_US);
        msg_joint_state_t state;
        msg_joint_setpoint_t setpoint;
        uint16_t received_len;
        bool out_acked;

        // set data to send
        state = (msg_joint_state_t) {
            .position = count,
            .velocity = 1 / count,
            .effort = count - 1 / count,
            .status = count + 1 / count,
            .flags = 0x11,
            .error_code = 0x00,
        };
        // send data
        ret = dtoe_write(&handle, &state, sizeof(state));
        if (ret == RET_PRECONDITION_NOT_MET) {
            // remote data reader does not exist
            sleep_until(next_loop_time);
            continue;
        } else if (ret != RET_OK) {
            printf("failed to write: %d\n", ret);
            sleep_until(next_loop_time);
            continue;
        }
        // wait acked by all until timeout
        /*
        absolute_time_t wait_ack_until = make_timeout_time_us(WAIT_ACK_INTERVAL_US);
        while (true) {
            ret = dtoe_is_acked_by_all(&handle, &out_acked);
            if (out_acked) {
                break;
            }
            if (get_absolute_time() > wait_ack_until) {
                printf("ACK timeout\n");
                break;
            }
        }
        */
        // receive data
        ret = dtoe_take(&handle, &setpoint, sizeof(setpoint), &received_len);
        if (ret == RET_OK) {
            print_joint_setpoint(&setpoint);
        } else if (ret == RET_PRECONDITION_NOT_MET) {
            // ignore if there's no matched remote data writer
        } else if (ret == RET_NO_DATA) {
            // no data available
        } else {
            printf("receive error: %d\n\n", ret);
        }
        count += 1.0f;
        sleep_until(next_loop_time);
    }

    dtoe_writer_delete(&handle);
    dtoe_reader_delete(&handle);
    dtoe_shutdown(&handle);
}
