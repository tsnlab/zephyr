## Build and Run

```Bash
# qemu_x86
west build -p auto -b qemu_x86 .
west build -t run

# rpi_5
west build -p auto -b rpi_5 .
mv zephyr/samples/realtime_test_stress_on/build/zephyr/zephyr.bin <rpi_5 root directory>
```
