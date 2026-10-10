# U-Boot 启动流程

整体关系：

```
上电
 |
 v
BootROM
 |
 v
SPL
 |
 v
ATF(BL31)
 |
 v
U-Boot proper
 |
 v
加载boot脚本 boot.scr
 |
 v
执行boot.cmd里的命令
 |
 v
加载 Kernel(Image)
加载 initrd
加载 DTB
应用 DTBO overlay
 |
 v
booti
 |
 v
Linux Kernel
```

---

# 1. BootROM阶段（芯片固化代码）

这是 Rockchip 芯片内部 ROM，不属于 U-Boot。

上电以后：

```
RK3576 BootROM
```

负责：

* 初始化最基本硬件
* 找启动介质

例如：

* eMMC
* SD
* SPI Nor Flash

你的日志：

```
Trying to boot from MMC2
MMC: no card present

Trying to boot from MMC1
```

就是 BootROM/SPL 阶段在尝试启动设备。

BootROM 会加载：

```
idbloader.img
```

里面通常包含：

```
SPL + DDR初始化代码
```

---

# 2. SPL阶段

SPL = Secondary Program Loader

第二阶段启动程序。

因为刚上电：

```
CPU
 |
 |-- SRAM很小
 |
 DDR还没初始化
```

不能直接运行完整 U-Boot。

所以先运行一个小版本：

```
U-Boot SPL
```

主要任务：

## ① 初始化DDR

你的日志：

```
DDR
LPDDR4X, 1848MHz

channel[0]
Size=1024MB

channel[1]
Size=1024MB
```

说明 SPL 完成了：

```
DDR训练
DDR频率设置
DDR控制器初始化
```

现在有：

```
2GB RAM
```

---

## ② 加载FIT镜像

你的日志：

```
Trying FIT image at 0x4000 sector
```

FIT：

```
fitImage
```

里面打包：

```
ATF BL31
U-Boot
DTB
OPTEE
```

你的日志：

```
Checking atf-1
Checking uboot
Checking fdt
Checking optee
```

说明 SPL 正在验证：

```
atf
 |
uboot
 |
device tree
 |
optee
```

---

然后：

```
Jumping to U-Boot(0x40200000)
via ARM Trusted Firmware(0x40040000)
```

意思：

SPL加载：

```
BL31 ---> 0x40040000

U-Boot --->0x40200000
```

然后跳到 ATF。

---

# 3. ATF阶段

ATF = ARM Trusted Firmware

现在叫：

```
Trusted Firmware-A
```

对应ARM异常等级：

```
EL3
```

ARM64有：

```
EL3
 |
 |-- Secure Monitor
 |
EL2
 |
 |-- Hypervisor
 |
EL1
 |
 |-- Linux Kernel
```

ATF运行在最高权限：

```
EL3
```

主要负责：

## ① 电源管理

## ② CPU启动

## ③ Secure Monitor

## ④ 切换到普通世界

你的日志：

```
BL31: Preparing for EL3 exit to normal world

Entry point address = 0x40200000
```

意思：

ATF准备退出EL3：

跳到：

```
U-Boot
```

---

# 4. U-Boot proper阶段

现在进入完整U-Boot：

日志：

```
U-Boot 2017.09
Model: Rockchip RK3576 Evaluation board
```

这里才是真正的U-Boot。

---

## 4.1 board初始化

调用：

```
board_init_f()
```

初始化：

* CPU
* 时钟
* 串口
* MMC
* 网络
* GPIO
* PMIC

你的日志：

```
PMIC: RK8060

vdd_cpu_big_s0 init

vdd_gpu_s0 init
```

就是这里。

---

## 4.2 重定位U-Boot

日志：

```
Relocation Offset: 7da0d000
```

U-Boot刚开始运行的位置：

Flash/SRAM

然后复制到：

DDR

以后运行DDR中的U-Boot。

---

## 4.3 初始化命令行

进入：

```
run_main_loop()
```

你之前贴：

```c
for (;;)
    main_loop();
```

进入：

```
main_loop()
```

---

# 5. main_loop阶段

流程：

```c
main_loop()
{
    cli_init();

    run_preboot_environment_command();

    s = bootdelay_process();

    autoboot_command(s);

    cli_loop();
}
```

重点：

```c
s = bootdelay_process();
```

这里获得：

```
bootcmd
```

例如：

```
run distro_bootcmd
```

---

# 6. bootdelay_process做什么？

核心：

```c
s = env_get("bootcmd");
```

读取环境变量：

```
bootcmd
```

你的RK3576：

大概率：

```
bootcmd=run distro_bootcmd
```

所以：

返回：

```
run distro_bootcmd
```

然后：

```c
autoboot_command(s);
```

执行它。

---

# 7. autoboot_command

这里：

```c
run_command_list(s,-1,0);
```

实际上执行：

```
run distro_bootcmd
```

---

# 8. distro_bootcmd

你贴过：

```c
"distro_bootcmd="
"for target in ${boot_targets}; do "
"run bootcmd_${target}; "
"done"
```

展开：

例如：

```
boot_targets=mmc0 mmc1 usb0
```

执行：

```
run bootcmd_mmc0

run bootcmd_mmc1

run bootcmd_usb0
```

---

# 9. 找到boot.scr

你的日志：

```
Scanning mmc 0:2

Found U-Boot script /boot.scr
```

说明：

U-Boot找到：

```
/boot/boot.scr
```

或者：

```
/boot.scr
```

boot.scr是什么？

它是：

```
boot.cmd
    |
    | mkimage
    |
    v
boot.scr
```

转换后的U-Boot脚本。

命令：

```bash
mkimage \
-C none \
-A arm \
-T script \
-d boot.cmd boot.scr
```

---

# 10. 执行boot.cmd

日志：

```
## Executing script at 40500000
```

进入你的：

```
boot.cmd
```

---

# 11. boot.cmd做什么？

boot.cmd的位置：LubanCat_SDK/kernel/arch/arm64/boot/dts/rockchip/uEnv/boot.cmd

## ① 加载uEnv.txt

```bash
load mmc 0:2 0x4a100000 /uEnv/uEnv.txt
```

得到：

```
bootargs
overlay配置
启动参数
```

---

## ② 设置kernel参数

例如：

```bash
setenv bootargs \
root=/dev/mmcblk0p3 \
console=ttyFIQ0
```

最后传给Linux。

---

## ③ 加载initrd

```bash
load ... /initrd
```

放到：

```
0x4a200000
```

---

## ④ 加载kernel

```bash
load ... /Image
```

放：

```
0x40400000
```

---

## ⑤ 加载DTB

```bash
load ... /rk-kernel.dtb
```

放：

```
0x48300000
```

---

## ⑥ 加载overlay

执行：

```
dtfile
```

例如：

```
uart8.dtbo
uart10.dtbo
pwm.dtbo
```

合并：

```
rk-kernel.dtb
 +
overlay
 =
最终dtb
```

---

# 12. booti启动Linux

最后：

你的日志：

```
booti 
0x40400000
0x4a200000
0x48300000
```

对应：

```bash
booti kernel_addr initrd_addr fdt_addr
```

也就是：

```
kernel
 |
initrd
 |
device tree
```

---

# 13. booti里面发生什么？

你贴的源码：

```c
do_booti()
```

流程：

```
booti
 |
 |
检查Image头
 |
 |
设置kernel入口地址
 |
 |
准备bootargs
 |
 |
传递DTB
 |
 |
跳转kernel
```

最终：

```c
do_bootm_states()
```

执行：

```
OS_PREP
OS_GO
```

最后：

CPU跳：

```
U-Boot
 |
 |
 v
Linux start_kernel()
```

---

# 最终完整图

```
            上电
              |
              v
        BootROM
              |
              |
        idbloader.img
              |
              v
        SPL
              |
      初始化DDR
      加载FIT
              |
              v
          ATF BL31
              |
        EL3切换
              |
              v
       U-Boot proper
              |
       board init
       relocation
       driver init
              |
              v
        main_loop()
              |
              v
        bootdelay_process()
              |
              v
        bootcmd
              |
              v
      distro_bootcmd
              |
              v
        boot.scr
              |
              v
        boot.cmd
              |
    -----------------
    |       |       |
 Image   initrd   dtb
              |
              |
        dtbo overlay
              |
              v
            booti
              |
              v
        Linux Kernel
              |
              v
          start_kernel()
```

---
