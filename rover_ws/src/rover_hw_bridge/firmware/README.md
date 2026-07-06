# STM32 / ODrive firmware (rover low-level board)

This directory holds the **reference firmware** for the microcontroller that runs
the hard-real-time motor loops and safety watchdog. Linux + ROS 2 on the Jetson
cannot guarantee millisecond timing, so this code owns the time-critical control.

The `mcu_sim_node` in `rover_hw_bridge` is a software-in-the-loop (SITL) emulator
of exactly this logic, so the whole stack can be developed and tested with no
board present. When hardware arrives, flash this firmware and point the Jetson
bridge at the real serial/CAN device instead of the `/mcu/rx` and `/mcu/tx` SITL
topics.

## Files
- `link_protocol.h` - the framed Jetson<->MCU protocol, byte-identical to the
  Python `link_protocol.py`. Shared framing means the same messages cross the SITL
  topic pair or a real UART/CAN bus unchanged.
- `control_loop.c` - the 1 kHz timer-ISR control loop: independent watchdog
  (safe-state on stale command / dead link), per-wheel PID, and over-current /
  stall fault latching. This is the safety-critical core.

## Bring-up outline (custom STM32 board)
1. Configure a hardware timer for a 1 kHz interrupt calling `control_loop_isr()`.
2. Wire UART or CAN RX to decode frames (`link_parse_byte`) and call
   `control_on_command(vx, wz)` on each `VEL_CMD`.
3. Periodically send `MSG_HEARTBEAT` (10 Hz) and `MSG_FEEDBACK` (50 Hz) frames.
4. Implement the `hw_*` board hooks (encoders, motor drivers, current sense,
   `hw_millis`).

## Using an ODrive instead of a custom STM32
An off-the-shelf **ODrive** can replace the custom board for the motor loops:
- Run each side in `AXIS_STATE_CLOSED_LOOP_CONTROL`, `CONTROL_MODE_VELOCITY_CONTROL`.
- Set `config.enable_watchdog = True` and `config.watchdog_timeout` (e.g. 0.2 s);
  the Jetson bridge must call `axis.watchdog_feed()` each command cycle, giving the
  same "dead link -> motors stop" guarantee as `FAULT_WATCHDOG` here.
- The Jetson converts the body twist to per-axis `input_vel` (rad/s) using the same
  `WHEEL_SEP` / `WHEEL_RADIUS` kinematics as `control_loop.c`.

## Safety invariant
On **any** fault (watchdog, over-current, stall, e-stop) or loss of the command
link, the motors are commanded to zero. Recovery requires a fresh, valid command
stream (and, for a latched E-STOP, an explicit reset from the Jetson).
