# Upstream preparation: ESP32-S3 NuttX HAL fixes

Target repository: [Espressif `esp-hal-3rdparty`](https://github.com/espressif/esp-hal-3rdparty).
The NuttX ESP32-S3 build pins the target at
`b90b1837cb5ad24747deb4c895246037cc206ce5` in
`arch/xtensa/src/esp32s3/Make.defs`. All paths below are relative to that
revision. The patches are deliberately retained in the NuttX fork until the
Hub Mini Plus SMP nightly soak has completed.

## 0001: Run local IPC calls before IPC task initialization

- Upstream file: `nuttx/src/components/esp_system/esp_ipc.c`.
- Bug: `esp_ipc_call_and_wait()` rejects a request when the per-CPU IPC task
  does not yet exist, including a request made by the calling CPU itself.
- NuttX manifestation: during `up_irqinitialize()`, GPIO ISR registration
  calls `esp_ipc_call_blocking()` before `esp_ipc_init()` can create IPC
  tasks. It returns `ESP_ERR_INVALID_STATE`; cleanup then enters the GPIO
  wrong-heap-free path described in the 2026-11-16 design findings. The board
  reaches only its two idle tasks and has no console.
- Fix: invoke a local callback directly and return `ESP_OK`; a local call
  needs neither IPC task nor scheduler.
- Evidence: on the WT32-SC01 Plus, JTAG showed the failed call during
  `up_irqinitialize()` and the resulting boot failure. With this patch plus
  the GPIO allocator fix, the board reached `BOARD`, mounted procfs, created
  10 tasks, and retained a healthy kernel heap.
- Suggested commit title: `esp_ipc: run local calls before IPC task init`.
- Suggested PR description: Allow a same-CPU IPC callback during early NuttX
  initialization. This avoids treating a synchronous local invocation as an
  IPC-task dependency and unblocks GPIO ISR registration before the scheduler
  is ready.
- Status: not submitted, waiting for nightly soak (D7).

## 0002: Free GPIO ISR-service allocations through heap caps

- Upstream file: `components/upper_hal_gpio/src/gpio.c`.
- Bug: `gpio_install_isr_service()` and `gpio_uninstall_isr_service()` free
  `heap_caps_*` allocations with `free()`, which maps to NuttX's user heap
  rather than the kernel heap used by heap caps.
- NuttX manifestation: the failed early GPIO IPC call triggers uninstall.
  `free()` links a kernel-heap node into the PSRAM user-heap list; later
  kernel allocations fail and bring-up falls through to the idle loop.
- Fix: use `heap_caps_free()` for both ISR-function-array cleanup paths.
- Evidence: WT32-SC01 Plus JTAG caught `mm_free(g_mmheap, 0x3fcc24e0)` from
  `gpio_uninstall_isr_service()`. The debug heap-membership assertion proves
  the allocation belongs to the other heap; with the patch, bring-up succeeds.
- Suggested commit title: `upper_hal_gpio: free ISR service allocations with heap_caps_free`.
- Suggested PR description: Preserve allocator pairing under NuttX, where
  heap-caps allocations are kernel-heap allocations and `free()` targets the
  PSRAM user heap.
- Status: not submitted, waiting for nightly soak (D7).

## 0003: Free DMA allocations through heap caps

- Upstream files: `components/upper_hal_dma/src/esp_dma_utils.c`,
  `components/upper_hal_dma/src/gdma.c`, and
  `components/upper_hal_dma/src/gdma_link.c`.
- Bug: DMA stash buffers, GDMA groups/pairs/channels, and GDMA link-list
  storage allocated through heap caps are released with `free()`.
- NuttX manifestation: this is the same kernel-heap-to-user-heap corruption
  class as the GPIO failure. The Hub Mini Plus LCD uses GDMA at run time, so
  it can corrupt a running SMP image after boot.
- Fix: replace each matching `free()` with `heap_caps_free()`.
- Evidence: the allocator audit was verified against the pinned SHA. The
  board SMP image with this patch set boots, paints, and retains a healthy
  heap; JTAG and the debug assertion established the failure mechanism on the
  identical GPIO/interrupt allocation pattern.
- Suggested commit title: `upper_hal_dma: pair heap-caps DMA allocations with heap_caps_free`.
- Suggested PR description: Keep all DMA error and teardown paths allocator
  symmetric for NuttX's separate kernel and user heaps.
- Status: not submitted, waiting for nightly soak (D7).

## 0004: Free RMT allocations through heap caps

- Upstream files: `components/upper_hal_rmt/src/rmt_common.c`,
  `rmt_encoder_bytes.c`, `rmt_encoder_copy.c`, `rmt_encoder_simple.c`,
  `rmt_rx.c`, and `rmt_tx.c`.
- Bug: RMT group, encoder, RX/TX channel, DMA-buffer, and sync-manager
  objects allocated through heap caps are released with `free()`.
- NuttX manifestation: a future RMT teardown or error path would place a
  kernel allocation in the PSRAM user heap, causing the same delayed heap
  corruption diagnosed by JTAG in the GPIO path.
- Fix: use `heap_caps_free()` for every listed ownership-matched cleanup.
- Evidence: audited against the pinned SHA; the board SMP boot/paint gate
  passes with the full patch set. The board debug assertion and JTAG trace
  demonstrate why a mismatched free is unsafe on NuttX.
- Suggested commit title: `upper_hal_rmt: free heap-caps objects with heap_caps_free`.
- Suggested PR description: Make RMT teardown and error paths preserve
  heap-caps allocator pairing on NuttX.
- Status: not submitted, waiting for nightly soak (D7).

## 0005: Free hardware-support allocations through heap caps

- Upstream files: `components/esp_hw_support/intr_alloc.c`, `rtc_module.c`,
  and `sleep_event.c`.
- Bug: interrupt descriptors/handles, RTC ISR entries, and sleep-event
  callback nodes allocated with heap caps are released with `free()`.
- NuttX manifestation: freeing an interrupt allocation to the user heap
  corrupts heap metadata. This path is directly exercised during SMP GPIO
  interrupt work.
- Fix: use `heap_caps_free()` for all matching cleanup and error paths.
- Evidence: on the WT32-SC01 Plus debug SMP image, the new heap-membership
  assertion stopped in `intr_free_for_current_cpu()` after it called `free()`
  on a heap-caps allocation; JTAG captured that backtrace. The panic text did
  not reach USB serial. This patch removes that assertion failure.
- Suggested commit title: `esp_hw_support: free heap-caps interrupt objects correctly`.
- Suggested PR description: Repair allocator pairing in interrupt, RTC, and
  sleep-event cleanup paths for NuttX's distinct kernel and user heaps.
- Status: not submitted, waiting for nightly soak (D7).

## 0006: Free SPI-flash allocations through heap caps

- Upstream files: `components/spi_flash/esp_flash_spi_init.c`, `flash_mmap.c`,
  and `spi_flash_os_func_app.c`.
- Bug: flash host/chip, mapping, and OS-function allocations obtained through
  heap caps are destroyed with `free()`.
- NuttX manifestation: an SPI-flash device removal, map failure, unmap, or
  deinitialization can cross-link the kernel and PSRAM user heaps, the defect
  class exposed by the failed GPIO registration.
- Fix: replace the matching releases with `heap_caps_free()`.
- Evidence: audited against the pinned SHA and included in the board SMP
  boot/paint patch set. The board JTAG investigation and debug assertion give
  the direct proof of the same allocator mismatch class.
- Suggested commit title: `spi_flash: free heap-caps allocations with heap_caps_free`.
- Suggested PR description: Keep SPI-flash allocation and cleanup APIs paired
  under NuttX's separate heap implementation.
- Status: not submitted, waiting for nightly soak (D7).

## 0007: Free ADC calibration allocations through heap caps

- Upstream file: `components/esp_adc/adc_cali_curve_fitting.c`.
- Bug: the curve-fitting scheme and context allocated with heap caps are
  released with `free()` on error and deletion.
- NuttX manifestation: ADC calibration teardown could corrupt the user heap
  with kernel-heap storage, as in the JTAG-diagnosed GPIO allocator failure.
- Fix: release the scheme, context, and handle with `heap_caps_free()`.
- Evidence: audited against the pinned SHA and part of the passing board SMP
  patch set; the WT32-SC01 Plus JTAG/debug-assertion evidence establishes the
  failure mode for this allocator mismatch.
- Suggested commit title: `esp_adc: pair calibration heap-caps allocations with heap_caps_free`.
- Suggested PR description: Correct ADC calibration error and destruction
  paths for NuttX heap-caps ownership.
- Status: not submitted, waiting for nightly soak (D7).

## 0008: Free power-management callback allocations through heap caps

- Upstream file: `components/esp_pm/pm_impl.c`.
- Bug: registered light-sleep callback nodes allocated through heap caps are
  removed with `free()`.
- NuttX manifestation: unregistering a callback can cause the same
  cross-heap free-list corruption found in the GPIO path before task creation.
- Fix: use `heap_caps_free()` for enter and exit callback removal.
- Evidence: audited against the pinned SHA and included in the board SMP
  patch set that boots and paints. The JTAG trace and debug assertion on the
  board directly prove the allocator mismatch class.
- Suggested commit title: `esp_pm: free light-sleep callbacks with heap_caps_free`.
- Suggested PR description: Preserve heap-caps allocator pairing when
  unregistering power-management callbacks on NuttX.
- Status: not submitted, waiting for nightly soak (D7).

## 0009: Free PHY backup allocations through heap caps

- Upstream file: `components/esp_phy/src/phy_init.c`.
- Bug: PHY modem-register and MAC/BB power-domain backup memory allocated
  through heap caps is released with `free()`.
- NuttX manifestation: PHY or Wi-Fi power-state teardown can corrupt the
  heap used by a running SMP system, in the same way as the early GPIO
  wrong-heap free found by JTAG.
- Fix: release both backup allocations with `heap_caps_free()`.
- Evidence: audited against the pinned SHA and present in the board SMP image
  that boots and paints with Wi-Fi enabled. The board JTAG/debug-assertion
  findings establish the consequence of this allocator mismatch class.
- Suggested commit title: `esp_phy: free heap-caps backup memory correctly`.
- Suggested PR description: Ensure PHY power-management cleanup uses the
  matching heap-caps allocator under NuttX.
- Status: not submitted, waiting for nightly soak (D7).

## Submission grouping

Submit 0001 as its own PR because it changes IPC initialization behavior.
Patches 0002 through 0009 can be one allocator-pairing PR (preferably kept as
component-scoped commits): they all replace `free()` with `heap_caps_free()`
for storage allocated through `heap_caps_*`, and share the same NuttX failure
mode and regression rationale.
