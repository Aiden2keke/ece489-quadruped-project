# Go2 有线网络配置速查

项目根目录：

```bash
/home/yd/ece489/project/rsl_rl_teacher_student
```

建议设置 PC 有线网卡 IPv4：

```text
Address: 192.168.123.10
Netmask: 255.255.255.0
Gateway: 留空或按现场网络设置
```

检查机器人连通性：

```bash
ping 192.168.123.161
```

查看本机有线网口名：

```bash
ip addr
# 或
ifconfig
```

`enp52s0` 只是示例。正式启动底层 bridge 时必须替换为实际网口名：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/build
sudo ./lcm_position_go2 <your_interface_name>
```
