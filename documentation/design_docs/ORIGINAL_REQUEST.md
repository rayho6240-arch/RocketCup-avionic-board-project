# Original User Request

## Initial Request — 2026-06-18T18:04:40Z

<USER_REQUEST>
We will build a comprehensive, line-by-line tutorial of the RocketCom C/C++ avionics codebase. The tutorial will be primarily written in a Markdown file, `教學(6/19).md`, with references to interactive HTML visualizers for complex concepts.

Working directory: `/Users/laizhiquan/coding/RocketCom/`
Integrity mode: development

## Requirements

### R1. Comprehensive Tutorial Markdown File (`教學(6/19).md`)
- Location: `/Users/laizhiquan/coding/RocketCom/教學(6/19).md`
- Language: Traditional Chinese (繁體中文).
- Explain the **overall architecture** of the avionics system (FreeRTOS scheduling, interrupts, double buffering).
- Teach **C/C++ syntax** (pointers, structs, enums, preprocessor, suffixes, bitwise ops) using real codebase snippets.
- Provide a **detailed line-by-line / section-by-section explanation** of the following custom files:
  - `main.c` / `main.h`
  - `fsm.c` / `fsm.h`
  - `adxl375.c` / `adxl375.h`
  - `bmi088.c` / `bmi088.h`
  - `bmp388.c` / `bmp388.h`
  - `ekf.c` / `ekf.h`
  - `gps.c` / `gps.h`
  - `telemetry.c` / `telemetry.h`
  - `w25qxx.c` / `w25qxx.h`
  - `mmc5983.c` / `mmc5983.h`
  - `soft_i2c.c` / `soft_i2c.h`
  - `link.c` / `link.h`, `link_hw.c` / `link_hw.h`, `link_proto.c` / `link_proto.h`
- For third-party library files (e.g. Bosch `bmp3.c`, FreeRTOS, FATFS), explain the interface API calls, initialization, and timing requirements, but do not explain them line-by-line.
- Reference the interactive HTML tools inside the `Tutorial_6_19/` directory when explaining complex concepts.

### R2. Interactive Learning Tools (HTML)
- Ensure the interactive HTML files in the `Tutorial_6_19/` directory are fully functioning, bug-free, and referenced correctly:
  - `fsm_tutorial.html` (FSM Visualizer)
  - `bitwise_tutorial.html` (Bitwise shifting register sandbox)
  - `architecture_tutorial.html` (RTOS task priorities, TIM interrupts, Ping-Pong double buffering, and Queue simulator)
  - `compilation_tutorial.html` (Make vs CubeIDE paths guide)

### R3. Compilation & Build Guide
- Explain compiler toolchains, what `make` and `Makefile` are, and how STM32CubeIDE handles Include Paths and Source Folders.

### R4. Manager Agent Verification
- A manager agent must review all 12 sets of custom C/H files and audit the final `教學(6/19).md` to ensure not a single line of custom code logic was left unexplained.

## Acceptance Criteria

### Completeness Check
- `教學(6/19).md` is successfully generated at the workspace root.
- Every listed custom file (12 components) has a dedicated walkthrough.
- No line of custom C/H code logic is omitted from the explanation.
- The interactive HTML files exist and are referenced in the MD file.
- The compiler and path configurations are detailed.
</USER_REQUEST>
