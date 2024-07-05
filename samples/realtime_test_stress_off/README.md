# How to build?

## 1. install dependencies
See link : https://docs.zephyrproject.org/latest/develop/getting_started/index.html#install-dependencies

## 2. Get zephyr source
See link : https://docs.zephyrproject.org/latest/develop/getting_started/index.html#get-zephyr-and-install-python-dependencies
  
![image](https://github.com/tsnlab/zephyr/assets/48320014/7585b781-57f9-490c-83e1-836606054f6a)  
The command in the picture is converted to `west init ~/zephyrproject -m https://github.com/tsnlab/zephyr --mr rpi_5_perf_test`.  

## 3. install zephyr SDK
See link : https://docs.zephyrproject.org/latest/develop/getting_started/index.html#install-the-zephyr-sdk

## 4. build source
```Bash
cd ~/zephyr/samples/realtime_test_stress_off

# qemu_x86
west build -p auto -b qemu_x86 .
west build -t run

# rpi_5
west build -p auto -b rpi_5 .
mv zephyr/samples/realtime_test_stress_on/build/zephyr/zephyr.bin <rpi_5 root directory>
```
