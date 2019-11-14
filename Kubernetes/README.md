# Kubernetes的核心概念

这篇文章简要分享Kubernetes中的一些有用的概念，文章只从Kubernetes使用层面介绍IBNS项目中大数据组件的容器化经验，不涉及Kubernetes的安装部署、架构分析等内容。

## Pod

Pod是Kubernetes中承载业务的核心，包括以下特点：

- Pod具有独立的独立网络、文件、进程等命名空间。但是，可以通过特殊配置使Pod和宿主机共用某种命名空间。常见操作是，使用hostNetwork指定Pod使用主机网络。
- Pod可以包含一个或者多个Docker容器。在一些容器平台中，Pod是有一定的设计模式的（[参考](https://github.com/LinQing2017/notes/blob/master/kube-learn/workloads/pods/pod.md#pod%E7%9A%84%E8%AE%BE%E8%AE%A1%E6%A8%A1%E5%BC%8F)）。

- 同一个Pod中容器被调度到同一个主机上。

**Kubernetes中设计Pod时的一些常用feature**：

- Init Container([参考](https://github.com/LinQing2017/notes/blob/master/kube-learn/workloads/pods/pod.md#init-container)): 通过Init Container可以将修改配置、初始化数据库等准备工作和业务逻辑分离；
- 容器Hook：通过Hook可以在容器启动前、启动后控制容器的行为（[参考1](https://github.com/LinQing2017/notes/blob/master/kube-learn/workloads/pods/pod.md#init-container)、[参考2](https://github.com/LinQing2017/k8s-big-data/blob/master/elasticsearch/chart/templates/data-statefulset.yaml)）；
- Pod位置编排：通过NodeSelector指定固定节点，通过亲和性可以Pod之间的位置编排以及Pod和Node之间的位置编排（[参考1](https://github.com/LinQing2017/notes/blob/master/kube-learn/workloads/controllers/LabelAndSelectors.md)）
- 向Pod中注入相关配置：
  - 注入环境变量：通过env字段
  - 注入文件：通过 ConfigMap + Volume 字段
  - 注入Pod的元数据信息：如容器的资源配置、控制器即时生成的Pod名称等，可以通过Downward API（[参考1](https://kubernetes.io/docs/tasks/inject-data-application/downward-api-volume-expose-pod-information/)）

- 在容器访问Kubernetes的API：在容器中访问Kubernetes的API需要为Pod创建相应的ServiceAccount，并且绑定相关的Role和权限（[参考1](https://github.com/LinQing2017/notes/tree/master/kube-learn/API)）。


## 网络通信

通常情况下Pod的IP是一直变动的（某些情况下主机名也是变动的），每次控制器每次更新pod，pod的IP信息都会发生变化。Pod之间相互访问需要通过Service服务进行通信。

Service具有固定的IP、固定域名，能够将请求路由到指定的Pod。 可以将Service看做Kubernetes中的HAProxy！！

Service正常工作需要和endpoints绑定，绑定方式是同名service和endpoints自动绑定。Service有如下特点：

- 指定selector时，service会自动创建同名endpoints，endpoints会自动检查相关pod的端口信息，并实时更新。

- 不指定selector时，service不会创建同名endpoints，用户需要手工创建endpoints。这种方式称为ExternalName Service，主要应用场景是将外部服务的端口引入到Kubernetes中

- 通常一个Service可以配置多个端口，但是要求每个端口有唯一的端口名

- 通过.spec.clusterIP可以指定service的名称

- Service被创建后kubelet会向pod中插入一系列环境变量，这些环境变量可以用来进行服务发现
    - 格式为 {Service名称}_XXX
    - 每个Service对应多个环境变量
    - 每个pod都会插入
    - 这些**环境变量应该不是实时更新的**！
- Headless Services通过将.spec.clusterIP 配置为None实现。无头Service不会分配IP，直接通过 pod_name.services_name访问

Kubernetes通过flannel、CoreDNS等组件在集群中建立了一张内网，通过Service可以解决pod之间的相互访问。

如果需要在K8S外网访问Pod的指定端口，可以通过以下手段：

- Pod使用主机网络
- 创建NodePort类型Serivce，将容器的指定端口映射到宿主机
- 复杂的服务映射可以使用ingress([参考](https://github.com/LinQing2017/notes/blob/master/kube-learn/%E7%BD%91%E7%BB%9C/ingress.md))

## 常用控制器

控制器主要是指：控制Pod升级、数量、回滚的一系列声明。

常用的控制器包括：

|控制器|作用|备注|
|----|----|----|
|Deployment|1. 保证指定个数的Pod副本<br /> 2. 修改Deployment的参数时自动更新Pod ||
|DaemonSet|1. 保证每个Node都运行一个Pod副本||
|Cronjob、job|1. Job保证Pod运行一次<br /> 2. CronJob保证在指定时间运行Job||
|StatefulSet|用于部署有状态服务||

## 有状态服务

通常有状态服务有以下特点：

1. 需要存储状态数据，Pod在任意一个节点启动都需要能够访问到该数据。
2. 服务内部有比较复杂的网络结构，Pod之间需要进行通信。
3. Pod的不同副本需要固定标识。

StatusfulSet 解决了上面的问题（[参考](https://github.com/LinQing2017/notes/blob/master/kube-learn/workloads/controllers/StatefulSets.md)）

## 共享存储

从目前看来，需要比较优雅的使用Kubernetes构建有状态服务，共享存储必不可少。

目前调研发现，Rook可以在K8S中快速部署Ceph为集群提供共享存储，并且性能损失不大。（[参考](https://github.com/LinQing2017/notes/blob/master/kube-learn/storage/rook/quick-start.md)）

# 大数据服务

参考 [k8s-big-data](https://github.com/LinQing2017/k8s-big-data)

## 制作Docker镜像

个人建议，Docker中的只包含JDK、安装包等必要内容。 

如配置文件，启动脚本等内容，通过ConfigMap以文件形式挂载到Pod中。

## 制作Helm包

略

## Operator工具

Operator是Kubernetes二次开发后的资源([参考](https://github.com/LinQing2017/notes/tree/master/kube-learn/operators/spark/app-yaml))

# 参考资料

[Kubernetes-handbook](https://jimmysong.io/kubernetes-handbook/)

[SIG和工作组](https://jimmysong.io/kubernetes-handbook/develop/sigs-and-working-group.html)

[sig-big-data](https://github.com/kubernetes/community/tree/master/sig-big-data)

[HDFS](https://github.com/apache-spark-on-k8s/kubernetes-HDFS)

https://jimmysong.io/posts/kuberentes-crd-custom-resource/




