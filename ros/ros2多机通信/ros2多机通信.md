在 ROS 2 中实现**多机通信**（多台机器上的节点可以互相通信）主要依赖 **DDS（Data Distribution Service）** 中间件。ROS 2 默认使用 DDS（如 Fast DDS、Cyclone DDS、RTI Connext 等）来实现跨主机的通信。下面我给你整理一份完整、可执行的指南：

---

## 1️⃣ 网络环境要求

1. 多台机器必须在同一局域网内，且能互相 ping 通。

2. 如果采用虚拟机的话，虚拟机网络模式需要改成桥接模式，相当于一个物理设备连接在同一个局域网，否则无法相互ping通。
    * 在 VMware 设置 → 网络适配器 → 桥接模式
    * 编辑→虚拟网络编辑器→手动选择要桥接到的网卡

3. 确保没有防火墙阻挡 DDS 端口：

   * Fast DDS 默认会使用 UDP 7400–7600 端口。
   * 可以临时关闭防火墙或开放相关端口：

     ```bash
     sudo ufw allow 7400:7600/udp
     ```

---

## 2️⃣ 配置 ROS\_DOMAIN\_ID

ROS 2 使用 **Domain ID** 区分不同的 ROS 网络，默认是 `0`。

* 多机通信时，所有节点必须使用相同的 **Domain ID**：

```bash
export ROS_DOMAIN_ID=0
```

可以在每台机器的 `~/.bashrc` 中加上，避免每次都设置。

---

## 3️⃣ 配置环境变量

主要是告诉 ROS 2 节点其它主机的 IP 地址或禁用内网自动发现：

* **方法 1：使用默认的 ROS 2 自动发现（推荐）**

  * ROS 2 默认通过 multicast 自动发现节点，无需额外配置，只要网络通畅即可。
  * 确保主机可以相互 ping 通，并且防火墙允许 UDP 广播。

* **方法 2：静态配置其他主机 IP**

  * 适用于无法使用 multicast 的环境：

  ```bash
  export ROS_DISCOVERY_SERVER=192.168.1.100:11811
  ```

  * 或者使用 `FASTRTPS_DEFAULT_PROFILES_FILE` 配置 XML 指定静态参与者。

---

## 4️⃣ 运行示例

假设有两台机器：

* **主机 A** IP: `192.168.1.101`
* **主机 B** IP: `192.168.1.102`

**步骤**：

1. **在主机 A 启动 talker**

```bash
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_cpp talker
```

2. **在主机 B 启动 listener**

```bash
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_cpp listener
```

如果网络和防火墙设置正确，B 会接收到 A 发的消息。

---

## 5️⃣ 常见问题排查

| 问题            | 解决方法                                        |
| ------------- | ------------------------------------------- |
| 节点互相发现不了      | 检查 `ROS_DOMAIN_ID` 是否一致；ping 对方 IP；检查防火墙    |
| 消息收不到         | 检查 DDS 端口是否开放；确保网络支持 multicast              |
| 不想用 multicast | 使用 `ROS_DISCOVERY_SERVER` 或 DDS XML 配置静态参与者 |

---

## 6️⃣ 高级优化

1. 使用 **Fast DDS XML 配置文件**，指定网络接口和参与者：

   ```xml
   <transport_descriptors>
       <transport_descriptor>
           <transport_id>UDP</transport_id>
           <type>UDPv4</type>
           <interfaceWhiteList>
               <address>192.168.1.101</address>
               <address>192.168.1.102</address>
           </interfaceWhiteList>
       </transport_descriptor>
   </transport_descriptors>
   ```
2. 对于跨网段通信，可以使用 **ROS 2 Discovery Server** 中继节点。

---

✅ 总结：

1. 确保多机可互通网络、Domain ID 一致。
2. 默认 DDS 支持自动发现节点。
3. 高级场景可通过 XML 或 Discovery Server 控制网络拓扑。

---

如果你需要，我可以帮你画一个 **ROS 2 多机通信的示意图**，直观展示节点如何跨主机通信。

你希望我画吗？
