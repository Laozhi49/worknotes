# 主要降低功率的方法
## 1、查看支持的低功耗模式
```bash
cat /sys/power/state

# 可能会输出
freeze mem disk
```
其中，
* freeze：轻度休眠
* mem：Suspend to RAM
* disk：Suspend to Disk（嵌入式一般不用）

如果有 mem，说明内核支持休眠。

查看实际Suspend：
```bash
cat /sys/power/mem_sleep

# 可能会输出
s2idle [deep]
```
[]方框表示默认进入的低功耗模式

## 2、查看唤醒源
```bash
cat /sys/kernel/debug/wakeup_sources
```

## 3、将GPIO配置为唤醒源



## 4、系统进入低功耗模式
```bash
echo mem > /sys/power/state

# 或
echo mem | sudo tee /sys/power/state
```

切换模式：
```bash
echo s2idle | sudo tee /sys/power/mem_sleep

echo deep | sudo tee /sys/power/mem_sleep
```


# 其他降低功率的方法
## 1、查看当前CPU频率
```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
```

## 2、查看可调节的频率
```bash
# 查看核
ls /sys/devices/system/cpu/cpufreq/

# 查看某个核的配置
ls /sys/devices/system/cpu/cpufreq/policy0/

# 查看某个核可调节的频率
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
```

## 3、限制cluster上限
```bash
# 调整上限
echo 1200000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq
echo 1200000 > /sys/devices/system/cpu/cpufreq/policy4/scaling_max_freq

# 更激进的省电
echo powersave > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo powersave > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor

# 恢复
echo schedutil > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo schedutil > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
```

## 4、其他功耗大头
```bash
# 查看其他功耗设备
ls /sys/class/devfreq/
```
可能会得到以下结果
```bash
27700000.npu  27800000.gpu  27d00000.vop  dmc
```

查看其中一个的配置
```bash
ls 27700000.npu

available_frequencies  device    max_freq  power        trans_stat
available_governors    governor  min_freq  subsystem    uevent
cur_freq               load      name      target_freq
```

设置governor（策略）
```bash
echo powersave > governor

# 恢复
echo performance > governor
```
