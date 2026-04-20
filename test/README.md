# Test

This folder is reserved for PlatformIO tests. There are currently no automated
tests for the Advanced Robotics Project firmware.

## Recommended Future Tests

- Test SBUS raw-to-PWM helper mapping.
- Test radio failsafe validation.
- Test the UDP GCS command parser.
- Test PID and trim clamp limits.
- Test the motor mixer for the diagonal/X layout.

## Running Tests

After tests are added, run:

```powershell
cd <project-folder>
platformio test -e esp32doit-devkit-v1
```

For logic that does not require hardware, keep the functions pure where
possible so they can be tested without the ESP32.
