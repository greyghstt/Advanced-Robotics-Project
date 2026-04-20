# Lib

This folder is reserved for local project libraries. PlatformIO compiles
libraries placed here and links them into the firmware.

## Current Contents

This branch currently does not use any vendored local library folder contents.
Dependencies are pulled through the PlatformIO Registry from `platformio.ini`.

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
