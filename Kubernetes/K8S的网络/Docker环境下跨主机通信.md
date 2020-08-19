# CNM 模型

 Docker 基于```CNM模型```实现了容器网络库 ```libnetwork```，支持原生跨主机网络方案```overlay```和```macvlan```，以及第三方网络插件如：```flannel```、```weave```、```calico```。

在CNM模型中，容器网络被抽象层三类组件：

- Sandbox：容器网络栈，及容器所在的网络空间
- Network：表示一个容器子网，子网内容器可以相互通信。容器子网可以有不同的实现方式，如网桥、VLAN等等。
- Endpoint：将Sandbox接入到Network中，一个Sandbox中可以有多个Endpoint，并连接到不同的Network中。



# Overlay

Overlay是 docker 原生的跨主机容器通信方案，其底层工作原理是基于 VXLAN 的隧道。

在 Docker 中创建 Overlay 网络需要提供 Zookeeper、etcd 等发现服务实现 docker 进程之间的通信。

```
# 使用ETCD作为后端存储时，添加docker的启动项

--cluster-store=etcd://127.0.0.1:12379    # etcd存储地址
--cluster-advertise=127.0.0.1:2375        # 当前docker节点peer端口

# etcd 服务需要部署3.3的，docker-ce 不兼容3.4版本的etcd
```

创建一个Overlay，网络并且使用该网络

```
docker network create --driver overlay demo-overlay
docker run --rm -it --net=demo-overlay goblin.tencentcloudcr.com/goblin/busybox sh 
```

busybox 中容器的网卡设备如下：

- eth1@if33：连接 root/netns 中的网桥的endpoint ，这张网卡主要用来访问外网。这个网桥是多个Overlay网络共用的
- eth0@if31：连接Overlay子网的独立netns，这个子网中有个vxlan设备，子网内的通信通过这张网卡实现。

```
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
30: eth0@if31: <BROADCAST,MULTICAST,UP,LOWER_UP,M-DOWN> mtu 1450 qdisc noqueue 
    link/ether 02:42:0a:00:00:02 brd ff:ff:ff:ff:ff:ff
    inet 10.0.0.2/24 brd 10.0.0.255 scope global eth0
       valid_lft forever preferred_lft forever
32: eth1@if33: <BROADCAST,MULTICAST,UP,LOWER_UP,M-DOWN> mtu 1500 qdisc noqueue 
    link/ether 02:42:ac:12:00:02 brd ff:ff:ff:ff:ff:ff
    inet 172.18.0.2/16 brd 172.18.255.255 scope global eth1
       valid_lft forever preferred_lft forever
```

vxlan 子网空间中的设备：

- veth0@if30：一端连接到busybox的ns
- br0：子网的网桥，作为路由存在，连接了所有veth设备
- vxlan0@if29：vxlan设备

```
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
2: br0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1450 qdisc noqueue state UP group default 
    link/ether 0a:b4:15:b1:8a:d8 brd ff:ff:ff:ff:ff:ff
    inet 10.0.0.1/24 brd 10.0.0.255 scope global br0
       valid_lft forever preferred_lft forever
29: vxlan0@if29: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1450 qdisc noqueue master br0 state UNKNOWN group default 
    link/ether 0a:b4:15:b1:8a:d8 brd ff:ff:ff:ff:ff:ff link-netns default
31: veth0@if30: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1450 qdisc noqueue master br0 state UP group default 
    link/ether 1e:cc:fb:1b:0a:fa brd ff:ff:ff:ff:ff:ff link-netns 183f75f635e0
```

# macvlan

macvlan 也可以实现多个容器跨网络访问，工作原理是在宿主机网卡上配置多个MAC地址，并且每个interface可以设定自己的IP。由于macvlan方案中，数据包不用进行拆包、封包，因此性能优异。

PS：maclvan 要正常工作宿主机网卡要开启```promisc```模式，如果宿主机是虚拟机还要在底层将虚拟机网卡开胃混杂模式

```
混杂模式（英语：promiscuous mode）,是指一台机器的网卡能够接收所有经过它的数据流，而不论其目的地址（目的mac地址）是否是它，一般计算机网卡都工作在非混杂模式下。
```

测试macvlan网络方案：

```
docker network create \
-d macvlan \
--subnet=172.16.86.0/24 \           # 这个应该是一个真实可路由的子网
--gateway=172.16.86.1   \           # 这个路由地址应该真实存在
-o parent=eth0 \
macvlan
```



PS：macvlan 会独占整个网卡，但是可以使用sub-interface实现多个macvlan网络

