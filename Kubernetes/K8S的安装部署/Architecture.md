# 核心组件

|组件|功能|核心组件|
|----|----|----|
|etcd|集群状态共享|核心|
|apiserver|资源操作的唯一入口，并提供认证、授权、访问控制、API注册和发现等机制|核心|
|controller manager|维护集群的状态，比如故障检测、自动扩展、滚动更新等|核心|
|scheduler|资源的调度，按照预定的调度策略将Pod调度到相应的机器|核心|
|kubelet|维护容器的生命周期，同时也负责Volume（CVI）和网络（CNI）的管理|核心|
|Container runtime|镜像管理以及Pod和容器的真正运行（CRI）|核心|
|kube-proxy|为Service提供cluster内部的服务发现和负载均衡|核心|
|kube-dns|为整个集群提供DNS服务||
|Ingress Controller|为服务提供外网入口||
|Heapster|资源监控|
|Dashboard|提供GUI|
|Federation|提供跨可用区的集群|
|Fluentd-elasticsearch|集群日志采集、存储与查询|


# Nodes

Node 节点包括以下内容：container runtime、kubelet、kube-proxy

Node 的状态集，即包含的一些node label集合
|状态集|状态说明|备注|
|----|----|----|
|Addresses|包括主机名、外部IP、内部IP||
|Condition|Condition label描述了所有running状态的node信息，包括OutOfDisk、Ready、MemoryPressure等|Ready condition需要尤其注意。当主机下电时，该状态会变成Unknown（40s没有收到心跳，从True变成Unknown）或者False（其他故障可能是该状态变成False）。此时驱逐Pod不会立即生效，需要5min在才开始重新调度pod。此时在分区节点上的pod会被k8s标记为Terminating状态或者Unknow状态，APIServer并不会把这些pod直接抹除。|
|Capacity|node节点的资源分配情况|
|Info|软件版本信息，包括Kubernetes、kernel、docker、OS|

## Node Controller

node controller是K8S的master组件用来管理node对象的。在Node生命周期中，Node Controller完成以下工作：

- 在Node注册时，为node申请CIDR空间（提供网路保障）；
- 如果Node运行在云服务商的虚机上，Node Controller和云服务商check VM是否可用；
- node状态检查，并驱逐不健康node上的pod。Node通过NodeStatus和lease（1.13版本）和Node Controller通信（lease比NodeStatus更轻量，汇报更频繁）。
- 驱逐NoExecute 节点上的POD
- 负责计算node上所有容器占用的容量（如果需要为Node预留资源，可以创建一个哑pod）。

## 节点自注册

当kubelet的启动项--register-node 为true时，kubelet会自动向API server注册节点，此时管理员无需手工创建Node对象。

## CCM

CCM和API server、scheduler、controller manager类似，插件的方式运行在master机器上是云服务商能够为Kubernetes提供机器迁移、虚机等服务。

云上的Kubernetes节点，Kubelet需要包含云相关的功能。

## Command

```shell
kubectl cordon $NODENAME // 将节点配置为不可调度
```

