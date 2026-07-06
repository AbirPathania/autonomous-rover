/*
 * control_loop.c - hard-real-time motor control + watchdog (STM32 reference).
 *
 * This is the safety-critical code Linux cannot be trusted to run with
 * millisecond determinism. It runs from a hardware timer interrupt at a fixed
 * rate (CONTROL_HZ). It:
 *   1. Enforces an independent WATCHDOG: if no fresh VEL_CMD has arrived within
 *      WATCHDOG_MS, it forces the guaranteed safe state (motors to zero) and
 *      raises FAULT_WATCHDOG. This is the last line of defence if the Jetson,
 *      the safety node, or the serial/CAN link dies.
 *   2. Runs a per-wheel PID velocity loop toward the commanded setpoints.
 *   3. Detects over-current and stall and latches the corresponding fault.
 *
 * Mirror of rover_hw_bridge/rover_hw_bridge/mcu_sim_node.py (the SITL emulator).
 */
#include "link_protocol.h"

#define CONTROL_HZ     1000u      /* 1 kHz control loop */
#define WATCHDOG_MS    200u
#define WHEEL_SEP      0.60f
#define WHEEL_RADIUS   0.15f
#define OVERCURRENT_A  30.0f

/* Provided by the board layer (encoders, motor drivers, timebase). */
extern float   hw_wheel_speed(int side);        /* measured rad/s            */
extern void    hw_set_motor(int side, float u);  /* apply control effort      */
extern float   hw_bus_current(void);             /* amps                      */
extern uint32_t hw_millis(void);                 /* monotonic ms              */

static volatile float    g_cmd_vx = 0.0f, g_cmd_wz = 0.0f;
static volatile uint32_t g_last_cmd_ms = 0;
static volatile uint8_t  g_fault = FAULT_NONE;

/* Called from the UART/CAN RX path when a VEL_CMD is decoded. */
void control_on_command(float vx, float wz)
{
    g_cmd_vx = vx;
    g_cmd_wz = wz;
    g_last_cmd_ms = hw_millis();
}

uint8_t control_fault_flags(void) { return g_fault; }

static float pid_step(int side, float setpoint)
{
    static float integ[2] = {0.0f, 0.0f};
    static float prev[2]  = {0.0f, 0.0f};
    const float kp = 0.8f, ki = 0.05f, kd = 0.01f;
    float meas = hw_wheel_speed(side);
    float err  = setpoint - meas;
    integ[side] += err;
    float deriv = err - prev[side];
    prev[side]  = err;
    return kp * err + ki * integ[side] + kd * deriv;
}

/* Timer ISR entry point at CONTROL_HZ. */
void control_loop_isr(void)
{
    float vx, wz;

    /* 1) Watchdog: stale command -> guaranteed safe state. */
    if ((hw_millis() - g_last_cmd_ms) > WATCHDOG_MS) {
        vx = 0.0f; wz = 0.0f;
        g_fault |= FAULT_WATCHDOG;
    } else {
        g_fault &= (uint8_t)~FAULT_WATCHDOG;
        vx = g_cmd_vx; wz = g_cmd_wz;
    }

    /* 2) Convert body twist -> per-side wheel setpoints (rad/s). */
    float sp_left  = (vx - wz * WHEEL_SEP * 0.5f) / WHEEL_RADIUS;
    float sp_right = (vx + wz * WHEEL_SEP * 0.5f) / WHEEL_RADIUS;

    /* 3) Fault gating: on any latched fault, command zero. */
    if (g_fault != FAULT_NONE) { sp_left = 0.0f; sp_right = 0.0f; }

    hw_set_motor(0, pid_step(0, sp_left));
    hw_set_motor(1, pid_step(1, sp_right));

    /* 4) Over-current / stall detection. */
    if (hw_bus_current() > OVERCURRENT_A)
        g_fault |= FAULT_OVERCURRENT;
    if ((sp_left != 0.0f || sp_right != 0.0f) &&
        hw_wheel_speed(0) < 0.05f && hw_wheel_speed(1) < 0.05f)
        g_fault |= FAULT_STALL;
}
