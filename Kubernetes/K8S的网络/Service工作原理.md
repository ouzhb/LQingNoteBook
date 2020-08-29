# Service

K8S中Service能力由每个节点**Kube-Proxy**服务提供，其中包括三种实现模式：

- userspace：在用户空间监听一个端口，所有的 service 都转发到这个端口，然后 kube-proxy 在内部应用层对其进行转发。
- iptables：iptables 完全实现 iptables 来实现 service
- ipvs：

# Kube-proxy

Kube-proxy 通常以DS的方式运行在Kubernetes集群中，并且使用宿主机网络，启动参数如下：

```shell
/usr/local/bin/kube-proxy --config=/var/lib/kube-proxy/config.conf --hostname-override=$(NODE_NAME)
```

上述启动命令中，/var/lib/kube-proxy/config.conf 是挂载到容器内部的configmap，包含两个文件：

- config.conf：proxy的配置文件
- kubeconfig.conf：访问K8S的证书配置

PS：原则上kube-proxy也可以使用Service的方式部署在物理机上。

- cluster-cidr：帮助 `kube-proxy` 区分内外流量
  - 当值为空时，`kube-proxy` 认为所有流量都是内部流量，不做 SNAT(MASQ)。
  - 当值非空时，来自 `cluster-cidr` 网络（即 Pod 网络）的流量被当成内部流量，访问 Service 时不做 SNAT(MASQ)，来自其他网络的流量被当成外部流量，访问 Service 时需要做 SNAT(MASQ)。
- masquerade-all：如果设置了该参数，那所有访问 Service 的流量都会做 SNAT(MASQ)，不再区分内外流量。

- 

## 1. Iptables 模式

不同于Pod的IP，Service 的 IP 本质上是 VIP，没有绑定到具体的网络接口上。

Kube-Proxy 主要使用了 iptables 的 filter 表和 nat 表，基于下面的自定义链工作：

- KUBE-SERVICES
- KUBE-SVC-XXX
- KUBE-SEP-XXX

**PS：上面只是列出主要的表和链，实际上还有很多！！**

PREROUTING 阶段所有报文本转到KUBE-SERVICES，```iptables -t nat -nvL PREROUTING ```命令的输出如下：

![image-20200819113351944](..\..\images\k8s\kube-proxy-iptables1.png)

在KUBE-SERVICES链中，每个Service对应一条规则，匹配到了对应的ServiceIP和端口号，执行 ```iptables -t nat -nvL KUBE-SERVICES```，输出如下：

![image-20200819133733313](..\..\images\k8s\kube-proxy-iptables2.png)

以DNS服务为例，Kube-DNS暴露了dns、dns-tcp、metrics 三个端口，对应到KUBE-SERVICES链中实际上是三条记录，执行```iptables -t nat -nvL  KUBE-SVC-ERIFXISQEP7F7OF4```检查kube-dns的tcp端口对应的IPTable规则，输出如下：

![image-20200819185311093](..\..\images\k8s\kube-proxy-iptables3.png)

可以看出每个 KUBE-SEP-XXX 规则实际上对应了一个后端pod，iptable 的 ```statistic```模块使网络包实际流入三个规则中的一个。

执行```iptables -t nat -nvL  KUBE-SEP-E7U3FADDJQ7POFHE```，输出如下：

![image-20200819190159776](..\..\images\k8s\kube-proxy-iptables4.png)

可以看到 KUBE-SEP-xxx 终于进行了DNAT，将目的地址替换为Pod的地址，后续CNI发挥作用将报文送达到实际容器网络接口。

KUBE-MARK-MASQ链会将报文打上`0x4000`标记，KUBE-POSTROUTING（在POSTROUTING链的NAT表中）会统一对有这些标记的回城报文进行SNAT。

![image-20200819192246073](..\..\images\k8s\kube-proxy-iptables5.png)

网络的流向如下图：

<img src="..\..\images\k8s\Chain4Kube.png" style="zoom:48%;" />



## 2.IPVS模式

IPVS 是专门用于负载均衡的内核模块，它和iptables一致同样基于**netfilter**工作。由于ipvs已经加入到了内核的主干，开启ipvs的前提是需要加载以下的内核模块：

```
ip_vs
ip_vs_rr
ip_vs_wrr
ip_vs_sh
nf_conntrack_ipv4
```

每个节点安装以下依赖包：

```shell
yum install ipvsadm ipset -y
```

如果Kube-Proxy 是部署在容器中的，修改 ```kubectl edit cm kube-proxy -n kube-system``` 中修改```KubeProxyConfiguration```的mode类型为ipvs，然后重新部署DaemonSet即可。

如果Kube-Proxy 是部署的物理机上，可以参考[官方文档](https://kubernetes.io/docs/reference/command-line-tools-reference/kube-proxy/)修改启动参数，然后重启ipvs服务。

PS：**需要注意几个问题**

- 按照上面的方式重启Kube-Proxy 之后，iptable模式时遗留的Rule不会自动清除，需要手动或者重启机器清除
- Kubernetes 1.18 使用的IPVS依赖较高版本的内核模块，需要升级内核版本，否则Kube-Proxy可能一直报错（参考：[iSSUE 89520](https://github.com/kubernetes/kubernetes/issues/89520)）。

### IPVS的工作原理

IPVS支持以下几种模式的负载均衡：

- NAT模式
- Direct Routing模式
- IP Tunneling模式
- Full NAT模式

PS：原生IPVS中NAT模式不会进行SNAT，只有一些基于IPVS二次开发的发行版支持Full NAT模式。
```
通常报文的请求、转发、返回路径有以下两种模式：

- 双臂模式：数据的进出都经过director
- 三角模式：数据从director进入，响应从real server直接回到client
```

上述模式中，Kubernetes主要工作在NAT模式中，并且借助IPtable为package进行SNAT。

IPVS 常用命令：
```shell
# 查看当前主机的所有ipvs转发
ipvsadm -Ln
```

## 3.关于SNAT

