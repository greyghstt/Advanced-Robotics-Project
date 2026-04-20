# Lib

This folder is reserved for local project libraries. PlatformIO compiles
libraries placed here and links them into the firmware.

## Current Contents

No custom local firmware library is currently required. The previous Teensy
FreeRTOS vendored library was removed because the active firmware targets ESP32
through the Arduino framework and PlatformIO registry dependencies.

## Adding a Local Library

If the project later needs a custom local library, place it in a structure like
this:

```text
lib/
`-- LibraryName/
    |-- src/
    |   |-- LibraryName.cpp
    |   `-- LibraryName.h
    `-- library.json
```

Libraries from the PlatformIO Registry should usually be listed in the
`lib_deps` section of `platformio.ini` instead of being copied manually into
this folder.
