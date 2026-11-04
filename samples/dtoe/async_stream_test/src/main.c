#include <stdio.h>

#include "dtoe_host.h"
#include "dtoe_monitor.h"
#include "msg_joint_state.h"
#include "msg_joint_setpoint.h"

#include "pico/stdlib.h"
#include "pico/time.h"

// Async streaming test, handoff doc 07 section 3.1. The two flows never wait for
// each other: the board sends state on its own fixed period and drains whatever
// setpoints arrived, the server sends setpoints on its own period.
//
// Demo types are kept (doc 07 3.1 option 2).
//   setpoint.target_velocity = server send seq (0-based)
//   setpoint.flags           = run ID of the server, 1..255
//   state.position           = board state seq (0-based)
//   state.velocity           = highest setpoint seq taken in this run + 1, 0 = none yet
//   state.effort             = setpoints taken in this run
//   state.flags              = run ID the two counters above belong to, 0 = none yet
// The server times setpoint N from its send to the first state of its run carrying >= N+1,
// on its own clock. The loop is write -> take -> wait, like the motor app, so a
// setpoint shows up in the state written on the next period.
//
// A float holds integers exactly up to 2^24 = 16,777,216, which bounds the length of a run:
// state.position at a 0.5 ms state period reaches it after about 2.3 h (4.6 h at 1 ms).
//
// Core 1 load, retransmits and losses come from the DTOE monitoring counters (include/dtoe_monitor.h), which
// exist only when DTOE is built with -DDTOE_MONITOR=ON. Without them, [stat] says "monitor off".

#ifndef STATE_PERIOD_US
#define STATE_PERIOD_US (2000)
#endif
#define REPORT_US        (5000 * 1000)

void isr_hardfault(void)
{
    puts(get_core_num() == 0 ? "\n[FATAL] CORE0 HardFault" : "\n[FATAL] CORE1 HardFault");
    stdio_flush();
    while (1) {
        tight_loop_contents();
    }
}

static msg_joint_setpoint_t setpoints[DTOE_SAMPLE_COUNT];

int main(void) {
    dtoe_handle_t handle;
    dtoe_retcode_t ret;
    msg_joint_state_t state = {0};
    uint16_t received_count;

    ret = dtoe_init(&handle);
    if (ret != RET_OK) {
        printf("failed to initialize DTOE: %d\n", ret);
        return 1;
    }
    ret = dtoe_writer_create(&handle, "joint_state_topic", &msg_joint_state, RELIABILITY_BEST_EFFORT);
    if (ret != RET_OK) {
        printf("failed to create writer: %d\n", ret);
        return 1;
    }
#ifdef SP_RELIABLE
    ret = dtoe_reader_create(&handle, "joint_setpoint_topic", &msg_joint_setpoint, RELIABILITY_RELIABLE);
#else
    ret = dtoe_reader_create(&handle, "joint_setpoint_topic", &msg_joint_setpoint, RELIABILITY_BEST_EFFORT);
#endif
    if (ret != RET_OK) {
        printf("failed to create reader: %d\n", ret);
        return 1;
    }

    printf("\n[async] state period %u us, setpoint %s, built %s %s\n", STATE_PERIOD_US,
#ifdef SP_RELIABLE
           "RELIABLE",
#else
           "BEST_EFFORT",
#endif
           __DATE__, __TIME__);

    uint32_t st_seq = 0;          // next state seq, advances only on RET_OK
    uint8_t  run    = 0;          // run ID of the setpoints counted below, 0 = none yet
    uint32_t sp_ack = 0;          // highest setpoint seq taken in this run + 1
    uint32_t sp_rx  = 0;          // setpoints taken in this run

    uint64_t mon_t0 = time_us_64();
    uint64_t mon_busy_us = 0;
    uint32_t mon_loops = 0, mon_sent0 = 0, mon_late = 0, mon_late_max = 0, mon_wr_err = 0;
#ifdef DTOE_MONITOR
    // 32 bits each, written by core 1 only: one load reads a whole value, differences wrap correctly
    uint32_t mon_c1_prev = g_mon_core1_busy_us;
    uint32_t mon_resent_prev = g_mon_resent, mon_gap_prev = g_mon_gap, mon_rx_lost_prev = g_mon_rx_lost;
#endif

    uint64_t next = time_us_64();
    while (1) {
        uint64_t loop_t0 = time_us_64();

        // exact up to 2^24: about 2.3 h at a 0.5 ms state period, 4.6 h at 1 ms
        state.position = (float)st_seq;
        state.velocity = (float)sp_ack;
        state.effort   = (float)sp_rx;
        state.flags    = run;
        if (dtoe_write(&handle, &state, sizeof(state)) == RET_OK)
            st_seq++;
        else
            mon_wr_err++;

        dtoe_take_many(&handle, setpoints, sizeof(setpoints[0]), DTOE_SAMPLE_COUNT, &received_count);
        for (uint16_t i = 0; i < received_count; i++) {
            // a setpoint of another run starts a new run. a pause of the stream, even a long one on
            // RELIABLE, keeps the counters
            if (setpoints[i].flags != run) {
                run = setpoints[i].flags;
                sp_ack = sp_rx = 0;
            }
            uint32_t seq = (uint32_t)setpoints[i].target_velocity;
            if (seq + 1 > sp_ack)
                sp_ack = seq + 1;
            sp_rx++;
        }

        mon_busy_us += time_us_64() - loop_t0;
        mon_loops++;

        uint64_t now = time_us_64();
        if (now - mon_t0 >= REPORT_US) {
            uint64_t win = now - mon_t0;
            uint32_t core0 = (uint32_t)(mon_busy_us * 1000 / win);
            // This line delays the next state write by the USB CDC write, so the gap the server sees around it
            // can show up in st_int_max.
            printf("[stat] core0=%lu.%lu%%", (unsigned long)(core0 / 10), (unsigned long)(core0 % 10));
#ifdef DTOE_MONITOR
            uint32_t core1 = (uint32_t)((uint64_t)(uint32_t)(g_mon_core1_busy_us - mon_c1_prev) * 1000 / win);
            printf(" core1=%lu.%lu%%", (unsigned long)(core1 / 10), (unsigned long)(core1 % 10));
#endif
            printf(" | loop=%luus n=%lu | tx=%luHz wr_err=%lu | late=%lu late_max=%luus | run=%u sp_rx=%lu sp_ack=%lu",
                   (unsigned long)(win / mon_loops), (unsigned long)mon_loops,
                   (unsigned long)((uint64_t)(st_seq - mon_sent0) * 1000000ULL / win),
                   (unsigned long)mon_wr_err, (unsigned long)mon_late, (unsigned long)mon_late_max,
                   (unsigned)run, (unsigned long)sp_rx, (unsigned long)sp_ack);
#ifdef DTOE_MONITOR
            // resent, gap: state writer (always 0 here, it is BEST_EFFORT). rx_lost: setpoint reader
            printf(" | resent=%lu gap=%lu rx_lost=%lu\n", (unsigned long)(uint32_t)(g_mon_resent - mon_resent_prev),
                   (unsigned long)(uint32_t)(g_mon_gap - mon_gap_prev),
                   (unsigned long)(uint32_t)(g_mon_rx_lost - mon_rx_lost_prev));
#else
            printf(" | monitor off\n");
#endif
            // reopen after printing: a blocked CDC write must not count against the next window
            mon_t0 = time_us_64();
            mon_busy_us = 0;
            mon_loops = mon_late = mon_late_max = mon_wr_err = 0;
            mon_sent0 = st_seq;
#ifdef DTOE_MONITOR
            mon_c1_prev = g_mon_core1_busy_us;
            mon_resent_prev = g_mon_resent;
            mon_gap_prev = g_mon_gap;
            mon_rx_lost_prev = g_mon_rx_lost;
#endif
            next = mon_t0;   // the report took the slot; restart the schedule here
        }

        // fixed period from the schedule, not from the end of the work
        next += STATE_PERIOD_US;
        now = time_us_64();
        if (now >= next) {
            uint32_t late = (uint32_t)(now - next);
            mon_late++;
            if (late > mon_late_max)
                mon_late_max = late;
            next = now;
        } else {
            busy_wait_until(from_us_since_boot(next));
        }
    }
}
