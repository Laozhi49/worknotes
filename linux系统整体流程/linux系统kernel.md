你前面已经把 **U-Boot → Kernel → RootFS → systemd** 这条链路走了一遍，所以现在总结 Kernel 流程最好不要只看函数，而是从“启动目的”来理解。

Linux Kernel 启动的核心目标：

> **把一个裸 CPU 环境，初始化成一个可以运行用户空间程序的完整操作系统环境。**

整体流程：

```
U-Boot
 |
 | 传递：
 |   - Kernel Image
 |   - DTB
 |   - bootargs
 |
 v
arch/arm64/kernel/head.S
 |
 v
start_kernel()
 |
 v
初始化内核子系统
 |
 v
创建 PID1
 |
 v
挂载 rootfs
 |
 v
启动 init(systemd)
```

---

# 1. U-Boot跳入Kernel

U-Boot做：

```text
加载:
    Image
    DTB

设置:
    x0 = 0
    x1 = DTB地址
    x2 = 0
    x3 = 0

跳转:
    kernel入口
```

ARM64入口：

```
arch/arm64/kernel/head.S
```

---

# 2. head.S阶段（汇编早期启动）

入口：

```asm
primary_entry
```

这里还没有：

* MMU
* C语言环境
* 栈
* 内存管理

主要工作：

---

## 2.1 保存boot参数

```asm
bl preserve_boot_args
```

保存：

```
x0-x3
```

尤其：

```
x1 = DTB地址
```

后面：

```c
setup_machine_fdt(__fdt_pointer)
```

会用。

---

## 2.2 确认CPU异常级别

```asm
bl init_kernel_el
```

ARM64有：

```
EL3
 |
EL2
 |
EL1
 |
EL0
```

Linux运行：

```
EL1
```

所以这里确保进入Kernel模式。

---

## 2.3 创建临时页表

```asm
bl create_idmap
```

这里就是你之前问的MMU。

创建：

```
VA = PA
```

的临时映射。

例如：

```
物理地址:

0x40000000


虚拟地址:

0x40000000
```

目的：

打开MMU后Kernel还能继续运行。

---

## 2.4 打开MMU

```asm
bl __enable_mmu
```

CPU从：

```
物理地址模式
```

进入：

```
虚拟地址模式
```

---

## 2.5 跳C语言入口

最后：

```asm
br x8
```

进入：

```
start_kernel()
```

---

# 3. start_kernel()

文件：

```
init/main.c
```

这是Kernel真正开始初始化的位置。

类似：

```
Linux Kernel main()
```

---

主要流程：

---

## 3.1 基础环境

```c
set_task_stack_end_magic()
```

初始化：

```
init_task
```

也就是：

```
PID0
```

---

## 3.2 架构初始化

```c
setup_arch(&command_line)
```

进入：

```
arch/arm64/kernel/setup.c
```

这里做：

---

### 解析DTB

你的代码：

```c
setup_machine_fdt(__fdt_pointer)
```

流程：

```
DTB地址
 |
 v
early_init_dt_verify()
 |
 v
解析/chosen
解析/memory
```

获得：

```
内存大小
bootargs
硬件信息
```

---

### 建立内存管理

```c
arm64_memblock_init();

paging_init();
```

初始化：

```
memblock
buddy allocator
page table
```

之后Kernel才能动态分配内存。

---

# 4. 初始化核心子系统

start_kernel继续：

---

## 内存管理

```c
mm_init()
```

建立：

```
页分配器
slab
vm
```

---

## 中断

```c
init_IRQ()
```

ARM64：

```
GIC
 |
 IRQ subsystem
```

---

## 调度器

```c
sched_init()
```

创建：

```
scheduler
```

但是此时还没有完整SMP。

---

## 时间系统

```c
timekeeping_init()
```

初始化：

```
timer
clocksource
```

---

## RCU

```c
rcu_init()
```

内核同步机制。

---

# 5. 创建PID1和PID2

到了：

```c
rest_init()
```

这里非常关键。

---

## PID1

代码：

```c
user_mode_thread(kernel_init,NULL,...)
```

产生：

```
PID1

kernel_init
```

作用：

后面启动用户空间。

---

## PID2

代码：

```c
kernel_thread(kthreadd,...)
```

产生：

```
PID2

kthreadd
```

负责创建：

```
kworker
ksoftirqd
mmcqd
...
```

---

关系：

```
PID0 swapper
 |
 +---- PID1 kernel_init
 |          |
 |          execve()
 |          |
 |          systemd
 |
 +---- PID2 kthreadd
            |
            +-- kworker
            +-- kernel threads
```

---

# 6. kernel_init阶段

函数：

```
kernel_init()
```

这里开始真正准备用户空间。

---

## 初始化CPU

```c
smp_init()
sched_init_smp()
```

启动其他CPU。

---

## 初始化驱动

关键：

```c
do_basic_setup()
```

里面：

```c
do_initcalls()
```

执行：

```
module_init()
device_initcall()
subsys_initcall()
```

也就是：

各种驱动入口。

例如：

```
uart driver
mmc driver
usb driver
net driver
```

开始注册。

---

流程：

```
driver_init()
 |
 do_initcalls()
 |
 driver_register()
 |
 bus匹配
 |
 probe()
```

---

# 7. rootfs挂载

之后：

```c
prepare_namespace()
```

开始挂根文件系统。

例如：

bootargs:

```
root=/dev/mmcblk0p2
```

流程：

```
找到mmcblk0p2

    |
    v

mount_block_root()

    |
    v

do_mount_root()

    |
    v

ext4_mount()

    |
    v

/
```

此时：

```
真正的根目录
```

建立。

---

# 8. exec init

最后：

```c
run_init_process("/sbin/init")
```

执行：

```
kernel_execve()
```

你的机器：

```
/sbin/init
      |
      v
/lib/systemd/systemd
```

---

Kernel任务完成。

---

# 9. 最终完整Kernel启动图

```
             U-Boot
                |
                |
                v
        arch/arm64/kernel/head.S
                |
        ----------------
        | 保存boot参数 |
        | 建页表      |
        | 开MMU       |
        ----------------
                |
                v
          start_kernel()
                |
        --------------------
        | setup_arch       |
        | DTB解析          |
        | 内存初始化      |
        | 中断初始化      |
        | 调度器初始化    |
        --------------------
                |
                v
             rest_init()
                |
        --------------------
        | PID0             |
        | PID1 kernel_init |
        | PID2 kthreadd    |
        --------------------
                |
                v
          kernel_init()
                |
        --------------------
        | 驱动initcall     |
        | 挂载rootfs       |
        --------------------
                |
                v
       kernel_execve("/sbin/init")
                |
                v
          systemd(PID1)
```

---

如果从嵌入式开发角度总结一句：

**U-Boot负责把Kernel叫醒，Kernel负责把硬件和资源准备好，然后启动RootFS里的第一个用户程序(systemd)，之后Linux才真正成为一个完整系统。**

你现在看的代码路径基本已经覆盖了 Linux ARM64 启动主干。下一步深入的话，建议选一个方向：

1. **驱动probe流程（设备树 → driver → probe）**
2. **MMU和页表细节**
3. **中断GIC初始化和中断触发流程**

这三个是嵌入式Linux最核心的三个知识点。
