# Lib

This folder is reserved for local project libraries. PlatformIO compiles
libraries placed here and links them into the firmware.

## Current Contents

This folder currently contains a vendored dependency:

```text
freertos-teensy-11.0.1_v1/
```

The README files inside that dependency are upstream library documentation and
do not need to be edited for the Advanced Robotics Project documentation.

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
