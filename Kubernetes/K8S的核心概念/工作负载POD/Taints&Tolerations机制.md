# Taints 

Taints 机制运用在Node节点上，通过该机制可以防止pod运行在一些节点上。

Taints包括几个部分：Key，Value，以及taint effect ，表示当key的值为value时，产生效果（taint effect）。

效果包括以下几种：

- NoSchedule ： 避免Pod调度
- PreferNoSchedule ：尽量避免Pod调度
- NoExecute ： 避免Pod调度，同时驱逐已经在该节点运行的Pod
  
1.6版本以后当Kubernetes满足以下条件时，会自动引入Taints：[参考](https://kubernetes.io/docs/concepts/workloads/controllers/daemonset/#taints-and-tolerations)
```shell
# 除非某些Pod能够容忍（Tolerations）NoSchedule标签，否者pod不会放置在node1
kubectl taint nodes node1 key=value:NoSchedule

# 解除Node1的 NoSchedule标签
kubectl taint nodes node1 key:NoSchedule-

```

# Toleration 

Toleration 机制施加在pod上，表示pod能够容忍node的某些状况
一个Node可以包括多种Taint，一个Pod也可包括多种Toleration

```yaml

# case1：
tolerations:
- key: "key"
  operator: "Equal"
  value: "value"
  effect: "NoSchedule"

# case2 
tolerations:
- key: "key"
  operator: "Exists"
  effect: "NoSchedule"

# case3：容忍key的任意value，任意效果
tolerations:
- key: "key"
  operator: "Exists"

# case4：容忍任意Key，任意Value，任意效果
tolerations:
- operator: "Exists"
```

# 通过NoExecute效果驱逐Pod

NoExecute不会立即驱逐Pod，如果没有指定tolerationSeconds不会发生驱逐，如果指定了tolerationSeconds则在该时间后进行驱逐。


# 使用场景

- 某些任务的专用节点
- 特殊硬件节点
- 通过NoExecute效果驱逐Node上的Pod