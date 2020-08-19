# Endpoint

Endpoint 是 pod 实例用来暴露服务的 ```IP：Port``` 列表，Kubernetes 会动态检查所有Pod的label，并将满足条件的 pod 添加到对应 Endpoint中。

- 用户创建Service时会伴生创建一个同名的 Endpoint ，Kubernetes 根据 Service 中设定的选择器，实时更新 Endpoint 中的 pod 列表。
- 如果创建的 Service 没有指定选择器，那么不会创建伴生 Endpoint
- Service 和 Endpoint 根据名称来对应

自定义 Endpoint 常用来将外部服务引入到Kubernetes集群中。

## Endpoints Controller

Endpoints Controller 控制器负责管理所有Endpoint，它是 kube-controller-manager 下的一个独立进程。


缺乏分析源码的能力！！ to be continue!!

# EndpointSlice

to be continue!!

## 参考
[【Endpointslices】](https://kubernetes.io/zh/docs/tasks/administer-cluster/enabling-endpointslices/)