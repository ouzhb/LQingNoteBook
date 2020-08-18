# 网络测评

参考博客：[Benchmark results of Kubernetes network plugins (CNI) over 10Gbit/s network](https://itnext.io/benchmark-results-of-kubernetes-network-plugins-cni-over-10gbit-s-network-updated-april-2019-4a9886efe9c4) 

## 1. CNI

测试了以下版本的CNI的性能：

- Calico v3.6

- Canal v3.6 (which is, in fact, Flannel for network + Calico for firewalling)

- Cilium 1.4.2

- Flannel 0.11.0

- Kube-router 0.2.5

- WeaveNet 2.5.1


## 2.指标

其评定指标包括以下几点：

- 配置便利性（主要表现为：能否正确识别MTU）
- 支持Network Policies（即和Istio集成，提供Ingress和Egress）
- 支持加密传输
- TCP、UDP、HTTP、FTP、SCP带宽
- 资源消耗

## 3.测试结果

- 只有 Flannel、Cilium 能够正确识别MTU
- Flannel 不支持，Kube-router  只支持Ingress，其他CNI都能和 istio 比较好的集成
- WeaveNet、Cilium支持加密传输，并且在加密场景下前者的性能优于后者
- TCP、UDP、HTTP、FTP、SCP带宽测试
  - WeaveNet最差性能损耗接近15%~20%
  - 其他CNI性能损失和裸机相比差距不大，均不超过10%
- Calico, Canal, Flannel, Kube-router 对性能的需求较少，Cilium非常耗性能

## 4. 结论

- 精简集群，同时对功能没有太多要求，Fannel 是第一选择
- 大多数场景可以选择 Calico 
- 需要加密的场景可以选择 WeaveNet

# MTU 对性能的影响

MTU是指[数据链接层](https://zh.wikipedia.org/wiki/資料連結層)上面所能通过的最大[数据包](https://zh.wikipedia.org/wiki/数据包)大小（以[字节](https://zh.wikipedia.org/wiki/字节)为单位），最大传输单元这个参数通常与通信接口有关（[网络卡](https://zh.wikipedia.org/wiki/网络卡)、[串口](https://zh.wikipedia.org/wiki/串口)等）。一条因特网传输路径的“路径最大传输单元”被定义为从源地址到目的地址所经过“路径”上的所有IP的最大传输单元的最小值。

如果IP包在传输过程中包的大小大于链路上某个设备MTU值，那么设备会对IP包进行分片并继续发往下游，并在目的地重新组装IP包。在一些需要传输大包的性能中，频繁进行IP分片是会影响性能的，可以考虑将设备的MTU设置的更大。

## 1.Jumbo Frames

Jumbo frames 又称为大型帧，是指有效负载超过IEEE 802.3标准所限制的1500字节的以太网帧，通常Jumbo Frames 的MTU 值为9000。

Linux 中只要在网卡配置中指定 MTU=9000 即可，当相应的物理交换机也要进行 MTU 的配置。

##2. Kubernetes指定CNI的MTU值

在Kubernetes中CNI的MTU配置决定了容器中网络设备的MTU，如果配置不正确容器间可能无法正常通信。

kubelet  支持```--network-plugin-mtu``` 参数调整CNI的默认MTU，但是通常情况下可能需要根据CNI的不同手工配置MTU的值。

# 参考

[Benchmark results of Kubernetes network plugins (CNI) over 10Gbit/s network](https://itnext.io/benchmark-results-of-kubernetes-network-plugins-cni-over-10gbit-s-network-updated-april-2019-4a9886efe9c4)

[CNI LIST](https://kubernetes.io/docs/concepts/cluster-administration/networking/)