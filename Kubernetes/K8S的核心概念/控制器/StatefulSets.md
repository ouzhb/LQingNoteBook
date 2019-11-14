# StatefulSets

## 限制条件
- StatefulSets,在1.9版本成为stable属性
- StatefulSets中存储必须是PV提供的，或者管理员预先分配的。
- 删除或者缩放StatefulSet将不会删除与StatefulSet关联的Volume
- StatefulSets的Service要求是Headless Service
- 建议通过在StatefulSet 中将副本数据调整为0来删除pod
  
## 原理

StatefulSets控制器包括以下内容：

### Pod Selector

.spec.selector 和 .spec.template.metadata.labels 需要相同，表示该StatefulSets纳管这些Pod

### Pod Identity

- StatefulSets的每一个Pod副本，都有一个唯一的Pod Identity。这个ID从1到N-1（N为Pod的副本数量）
- Pod Identity 被用来标识Pod独立的网络、存储、以及PodName。
- 在StatefulSets中Pod Name属性是确定的，格式为$(statefulset name)-$(ordinal)

### Stable Network ID
- 在每个Pod中，主机名的格式为$(statefulset name)-$(ordinal)。
- 用户可以使用 Headless Service 来访问每一个Pod。用户可以在.spec.serviceName中定义统一的服务名，此时Pod的访问方式为：$(statefulset name)-$(ordinal).$(serviceName).$(ns).$(svc.cluster.local)

```
PS: 关于Headless Service，即将clusterip指定为None的Service。用户没有办法通过clusterip访问服务，进行负载均衡和路由。但是，用户在定义了selector后，DNS服务会自动为Service对应的Pod配置域名。用户可以通过Pod域名访问每个Pod。
```

### Stable Storage

- StatefulSets使用 .spec.volumeClaimTemplates ,根据这个模板每个Pod会创建独立的PVC（名称为：${volumeClaimTemplates-name}-{pod-name}）
- 当Pod被重新调度或者删除时，该Pod专用的PVC不会变化。
- 

### Pod Name Label

- 在StatefulSets中每一个POD在创建完成后，会有一个statefulset.kubernetes.io/pod-name标签。

## 部署&Scaling

- Kubernetes保证部署是按顺序的，即当pod-name-1处于Runing状态时，pod-name-2才会开始部署；
- Kubernetes保证删除是按顺序的，即当pod-name-N被完全删除时，pod-name-N-1才会开始删除；
- Scaling操作同样按照顺序
- 通过.spec.podManagementPolicy字段，StatefulSets可以放宽关于顺序部署的保证，包括：OrderedReady（顺序串行）和 Parallel（并行）
  
## 更新策略

通过.spec.updateStrategy可以控制StatefulSet的更新策略（即修改containers, labels, resource request/limits, annotations 这些配置时Pod的行为）。

- OnDelete： 不会自动更新Pod，需要用户手工删除；
- RollingUpdate： 默认配置，按照顺序滚动升级Pod。该策略下可以指定.spec.updateStrategy.rollingUpdate.partition，当Pod的Index小于该值时不会发生自动升级。
