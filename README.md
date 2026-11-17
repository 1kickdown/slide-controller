# slide-controller

[![CI](https://github.com/1kickdown/slide-controller/actions/workflows/ci.yml/badge.svg)](https://github.com/1kickdown/slide-controller/actions/workflows/ci.yml)

Position controller and command interface for a motor-driven linear slide (lift, arm, gantry axis) in embedded-style C99, with a small simulation, tests and CI.

This is a learning project in simulation. It is not production code, it has not been run on a microcontroller, and it makes no claim of MISRA or functional-safety compliance.

## Modules

| File | Purpose |
|---|---|
| `src/slide.c` | control loop: homing, motion profile, PID, stall detection, fault handling |
| `src/proto.c` | byte-stream command protocol: framing, CRC-8, resynchronising parser, encoder |
| `src/crc8.c` | CRC-8 (polynomial 0x07) |
| `src/ringbuf.c` | fixed-size ring buffer for received bytes |
| `src/app.c` | firmware-style glue: `app_rx_byte()` for the UART interrupt, `app_tick()` once per control period |
| `sim/` | host-only simulation: DC-motor-like plant, friction, hard stops, quantised encoder |

Embedded-style constraints: no `malloc`, no globals, no I/O in the library, fixed-size state, single-precision floats, fixed control period (50 Hz by default), encoder input as a raw 32-bit count with any starting value.

## Controller

`slide_step()` is a pure function of (state, config, input) that returns a motor duty in [-1, 1]:

- **Homing:** drives towards the bottom switch with a fixed duty and zeroes the position when the switch closes. Repeatability is limited by the control period (about 1 mm at 60 mm/s and 50 Hz). A homing timeout leads to `FAULT`.
- **Motion profile:** trapezoidal, with velocity and acceleration limits. The target is clamped to the software limits and the set-point never crosses it.
- **Control:** PID on position, damping on velocity error, velocity and acceleration feed-forward, integrator with anti-windup.
- **Stall detection:** large duty without movement for longer than a threshold leads to `FAULT`.
- **Faults:** invalid configuration, NaN target, homing timeout, stall. `FAULT` outputs zero and is latched until the controller is disabled; the next enable starts with homing.

## Command protocol

Frame: `0xA5 | LEN | CMD | payload | CRC8(LEN, CMD, payload)`, payload up to 6 bytes.

| Command | Code | Payload |
|---|---|---|
| `CMD_ENABLE` | `0x01` | 1 byte, 0 = off |
| `CMD_SET_TARGET` | `0x02` | int16 little endian, 0.1 mm |
| `CMD_STATUS_REQ` | `0x03` | none |
| `CMD_STATUS` (reply) | `0x83` | mode, homed, position int16 (0.1 mm), duty int8 (%) |

The parser is a small state machine that skips garbage until the next start byte, rejects invalid lengths and bad CRCs and counts both. Frames with a wrong payload size are ignored. Receive overflows of the ring buffer are counted.

## Build and run

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/slide_sim                    # summary of all scenarios
./build/slide_sim two_moves          # CSV trace of one scenario
```

Compiler flags: `-std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.

## Scenarios (default configuration)

| Scenario | What happens | Result |
|---|---|---|
| `homing_and_move` | start at an unknown position, home, move to 300 mm | settles within 1 mm, overshoot about 1.5 mm, peak speed 317 mm/s (limit 300) |
| `two_moves` | 300 mm, then back to 100 mm | settles within 1 mm of the new target |
| `obstacle_stall` | hard obstacle at 200 mm on the way up | `FAULT` after about 3.9 s, output goes to zero |
| `switch_stuck` | bottom switch never closes | `FAULT` when the 6 s homing timeout expires |

## Tests

26 tests in two programs (`test_slide`, `test_app`): duty bounds, profile limits, target clamping, integrator limit, fault latching, invalid input, CRC-8 check value, ring buffer wrap-around and overflow, parser resynchronisation, bad CRC and length handling, 200 000 pseudo-random bytes through the parser, and the full command path from bytes to controller state and status reply.

An early version of the profile could overshoot its target and leave the software limit by a fraction of a millimetre; the clamping test found it and the profile now never crosses the target.

## Limitations

- Simple plant: one axis, linear damping and Coulomb friction, no gravity, no backlash, no current model, no sensor noise beyond encoder quantisation.
- Controller gains are hand-tuned for this plant, not optimised. The peak speed overshoots the profile limit by about 6 %.
- The protocol has no addressing, sequence numbers or acknowledgements.
- The GitHub Actions workflow is set up for GCC and Clang on Linux. Locally it has been built and tested with Apple Clang only.
