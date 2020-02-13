### 桥接网络概念

1. 服务器虚拟化网络，90%基本上都是网桥网络模型

2. VM1 --> br0(br0会关联一个物理网卡) --->真实的交换机连接

   虚拟机通过虚拟网卡和虚拟机网桥相连，虚拟网桥通过上行链路(br0的物理网卡和真实交换机相连的链路)和物理网络相连，从而将虚拟网络导入到物理网络上来

3. br0:这个虚拟网桥在虚拟网络看来，就是普通的二层交换机

### 配置桥接网络步骤(主要以修改配置的方式)

#### 1. 在物理机(宿主机)中添加一个网桥

- 使用配置方式在宿主机中添加一个网桥

```shell
cd /etc/sysconfig/network-scripts  #从原来的物理机网卡拷贝一份配置当做模板
cp ifcfg-ensp01 ifcfg-br0          #ifcfg-br0为虚拟网桥配置文件
vi ifcfg-br0
TYPE=Bridge                        #类型为桥接网络
BOOTPROTO=none
DEFROUTE=yes
IPV4_FAILURE_FATAL=no
IPV6INIT=yes
IPV6_AUTOCONF=yes
IPV6_DEFROUTE=yes
IPV6_FAILURE_FATAL=no
IPV6_ADDR_GEN_MODE=stable-privacy
NAME=br0
DEVICE=br0
ONBOOT=yes            #开机启动
DNS1=192.168.58.110   #宿主机配置DNS
IPADDR=172.29.32.52   #宿主机对外的IP地址
PREFIX=24
GATEWAY=172.29.32.254 #宿主机配置的网关
IPV6_PEERDNS=yes
IPV6_PEERROUTES=yes
IPV6_PRIVACY=no
ZONE=public
```

- 使用命令行方式在宿主机中添加一个网桥

```shell
nmcli connection add type bridge con-name br0 ifname br0
```

- **查看配置结果

```
ifconfig br0
ip addr show br0
ll /etc/sysconfig/network-scripts/ifcfg-br0
```



#### 2. 给(宿主机)网桥添加一个上行链路

- 修改配置文件方式

```shell
### 1. 这里主要是将原来的网卡配置的IP地址删掉(注释掉)
### 2. 增加一项将物理网卡桥接到网桥br0上 （BRIDGE=br0）

cat /etc/sysconfig/network-script/ifcfg-ensp01
TYPE=Ethernet
NAME=br0-eth0
DEVICE=eth0
ONBOOT=yes
BRIDGE=br0   #将物理网卡桥接到网桥上
```

- 命令行方式

```shell
### brctl 命令在开机后，配置丢失
### 把原来主机的eth0这个网卡添加到 br0上来
nmcli connection add type bridge-slave con-name br0-eth0 if eth0 master br0
```

- **查看配置结果

```shell
[root@cicd ~]# virsh domiflist master01
Interface  Type       Source     Model       MAC
-------------------------------------------------------
vnet1      bridge     br0        virtio      52:54:00:4b:9e:cf
```



#### 3. 把虚拟机添加到网桥

- 如果是用命令行创建虚拟机

  ```shell
  virt-install \
  --virt-type=kvm \
  --name=kvm-base \
  --vcpus=2 \
  --memory=4096 \
  --location=/home/CentOS-7-x86_64-Minimal-1611.iso \
  --disk path=/data/data1/vms/kvm-base.qcow2,size=40,format=qcow2 \
  --network bridge=br0 \    #将虚拟机桥接到网桥上
  --graphics none \
  --extra-args='console=ttyS0' \
  --force
  ```

- 如果是用virst-manager

  ​

#### 4. 给虚拟机配置IP地址

- 桥接网络模式，给虚拟机配置的IP地址，应当是物理网络中已经规划好的IP地址。并且网关也用物理网络的网关。

### 故障排查思路

```
1. 物理网络是不是通的？
2. vm是否桥接在br0上
3. br0是否有上行链路，br0有没有添加物理网卡，物理网卡和交换机链路是否正常
4. 看vm的网管是否正确
5. br0物理网卡 --> SW1 对接的口配置是否正确，vlan 和access
```

