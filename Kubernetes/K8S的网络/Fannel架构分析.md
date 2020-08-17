# Flannel

Flannel 是 CoreOS 针对 Kubernetes 设计的一个**Overlay Network**工具。相比于其他 CNI 模型，Flannel 只专注于网络通信没有提供 Network Policy 功能，是目前最为精简的一款CNI工具。

当前Flannel 支持的后端包括：

- UDP：一种自定义的，基于UDP转发三层报文的隧道协议

- VXLAN：基于UDP转发二层报文的隧道协议

- host-gw：基于路由方案工作

- 基于云服务的后端：[AWS](https://github.com/coreos/flannel/blob/master/Documentation/aws-vpc-backend.md) 、[AliVPC](AliCloud VPC Backend for Flannel) 、[GCE](https://github.com/coreos/flannel/blob/master/Documentation/gce-backend.md)

- IPIP：IPIP也是Linux的原生隧道方案之一，相比VXLAN功能更加精简，但是只支持IPv4单播流量

- IPSec：是一种对IP协议进行加密和认证隧道协议

上述后端中，VXLAN和Host-gw是最常用的，UDP用于一些老旧内核环境的场景，IPIP和IPSec均是0.10版本之后提供的功能。



Flannel 基于每个节点的flanneld客户端和一个中心存储工作，基本原理是：为每个主机分配一个subnet，多个subnet组成大的容器网络。

- 节点对Subnet 的租期时间是24小时，节点每隔一小时会刷新他的租期
- 当flanneld重启时，会根据节点名称从etcd上获取上次的Subnet信息，并且重用这个网段的IP资源

- 用户可以通过手工的方式，控制etcd上节点信息的超期时间，从而实现固定的IP配置
- Flannel 是不提供主机名dns能力的

# Docker + Flannel

Flannel 可以应用在原生Docker的跨主机通信中，用户只要部署一个```etcd```服务，Flannel 就可以为docker提供跨主机容器网络。

PS：Flannel 只支持 ETCD V2版本的API

在ETCD 的```/coreos.com/network/config```写入Flannel的配置信息。所有配置项参考[官网](https://github.com/coreos/flannel/blob/master/Documentation/configuration.md)。

PS：不同Backend有各自独立的配置项（[参考](https://github.com/coreos/flannel/blob/master/Documentation/backends.md)）

```json
{
    "Network":"10.33.1.0/16",
    "SubnetLen":24,
    "Backend": {
        "Type":"udp"
    }
}
```

在ETCD上写入配置文件后，启动flanneld 进程

```
flanneld --etcd-endpoints=http://172.24.33.77:2379
```

flanneld 启动后会在 ```/run/flannel/subnet.env``` 中保存该节点的子网信息，如下：

```
FLANNEL_NETWORK=10.33.0.0/16
FLANNEL_SUBNET=10.33.15.1/24
FLANNEL_MTU=1472
FLANNEL_IPMASQ=false
```

设定docker进程的启动参数，编辑```/usr/lib/systemd/system/docker.service```

```
EnvironmentFile=/run/flannel/subnet.env
ExecStart=/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock --cluster-store=etcd://172.24.33.77:2379 --cluster-advertise=172.24.33.112:2375 --bip=${FLANNEL_SUBNET} --mtu=${FLANNEL_MTU}
```

## 2. UDP/VXLAN 模式

- docker的默认网桥docker0会从主机对应的sunnet获取ip

- 所有容器的veth一端连接到docker0
- 宿主机上会出现一个特殊的网络设备flannel0，查看路由可知所有Subnet的包从该设备进出。
- 容器的外网能力，依然通过docker0的NAT能力来实现

## 3. host-gw 模式





