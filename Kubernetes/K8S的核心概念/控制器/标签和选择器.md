# 标签

- key包括两部分：prefix（可选）/ name，name以字母或者数字开头，允许“-”，“_”,“.”作为中间字符。前缀必须是一个DNS域名，如kubernetes.io/表示Kubernetes核心组件。

# 选择器

-  equality-based 选择器：支持=（==），！= 两种语义。Service 和 ReplicationController的选择器支持该语义
``` yaml
"selector": {
    "component" : "redis"
}
```
-  set-based 选择器： 支持in，notin，exists 三种语义。Job, Deployment, Replica Set, Daemon Set支持该语义。
```yaml
selector:
  matchLabels:
    component: redis # 等价于equality-based语义
  matchExpressions:
    - {key: tier, operator: In, values: [cache]}
    - {key: environment, operator: NotIn, values: [dev]}
    - {key: key-exist, operator: Exists}
    - {key: key-not-exist, operator: DoesNotExist}
```

# 节点选择

## nodeSelector

通过pod的.spec.nodeSelector选择节点，nodeSelector似乎支持equality-based的选择器。


Kubernetes Node包括以下内建的labels：

```
kubernetes.io/hostname
failure-domain.beta.kubernetes.io/zone
failure-domain.beta.kubernetes.io/region
beta.kubernetes.io/instance-type
beta.kubernetes.io/os
beta.kubernetes.io/arch
```

## Affinity and anti-affinity

- 该特性是软性限制，而非硬限制；
- 表达关系更加丰富；
- 允许限制pod和pod之间的关系；

### Node affinity

包括两种类型的Node限制：requiredDuringSchedulingIgnoredDuringExecution、preferredDuringSchedulingIgnoredDuringExecution。前者是硬限制，后者是软限制。
"IgnoredDuringExecution"表示，如果运行过程中node的label发生变化，pod将会继续运行在该node上。

如果同时指定nodeSelector和nodeAffinity，需要同时满足两者。

### Inter-pod affinity and anti-affinity 

通过Node上运行pod的label来控制接下来的pod能否运行到这个节点上。这个特性在大集群中，可能会极大的影响Kubernetes的调度性能。

包括以下几种标签：
requiredDuringSchedulingIgnoredDuringExecution：硬性要求
preferredDuringSchedulingIgnoredDuringExecution：软性要求

```yaml
# 下面的效果
  affinity:
    podAffinity:
      requiredDuringSchedulingIgnoredDuringExecution:
      - labelSelector:
          matchExpressions:
          - {key: security, operator: In, values: [S1]}
        topologyKey: failure-domain.beta.kubernetes.io/zone # 这里topologyKey表示指定范围内的Pod，常见的限制范围kubernetes.io/hostname、failure-domain.beta.kubernetes.io/zone、failure-domain.beta.kubernetes.io/region，编制节点、Zone、Region
    podAntiAffinity:
      requiredDuringSchedulingIgnoredDuringExecution:
      - labelSelector:
          matchExpressions:
          - {key: security, operator: In, values: [S2]}
        topologyKey: failure-domain.beta.kubernetes.io/zone
```

使用案例：https://kubernetes.io/docs/concepts/configuration/assign-pod-node/#an-example-of-a-pod-that-uses-pod-affinity