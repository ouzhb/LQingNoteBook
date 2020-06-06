# OVS 与 OVN

- OVS: OpenvSwitch，单机网络进行虚拟化的一个工具（类比docker）

    - 单机软件
    - 能够实现虚拟交换机，二层转发
    - 可以把虚拟网卡（虚拟机中的网卡）和虚拟交换机的端口连接
    - 支持OpenFlow（可编程的流量控制语言）、网络流量监控协议

- OVN：Open Virtual Switch，集中式的OVS控制器（类比Kubernetes），可以从集群角度对整个网络设施进行编排

    - 为OpenvSwitch开发SDN控制器

# Kube-OVN

灵雀云主导的开源项目，目标是将OVN的功能完整移植到Kubernetes中，使Kubernetes能和IaaS有相当的网络编排能力。

# 其他概念

## VPC

虚拟私有云（VPC，Virtual Private Cloud）是云计算的混合模型，其中在**公共云提供商的基础架构内提供私有云解决方案**，即公共云提供商为VPC用户保留的专用云服务器，虚拟网络，云存储和私有 ID 地址。

## 弹性IP

通过修改DNS的规则，将用户帐号、虚拟机ID和静态IP地址关联，保证重建虚拟机时能够分配稳定的IP地址。

# 参考

[基于OVN的Kubernetes网络架构解析](https://blog.csdn.net/M2l0ZgSsVc7r69eFdTj/article/details/88967801)

[SDN指南](https://feisky.gitbooks.io/sdn/container/kubernetes.html)

[alauda/kube-ovn](https://github.com/alauda/kube-ovn/wiki)