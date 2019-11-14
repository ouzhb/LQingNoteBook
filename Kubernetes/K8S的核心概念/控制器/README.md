# ReplicationController

ReplicationController确保一个pod或一组同类pod总是可用。

RC是最基础的控制器，其他高级控制器如ReplicaSet、Deployment 、DaemonSet是基于RC的高级API。

## 备注

- .spec.template下是Pod的模板信息
- 有两个地方可以配置labels，包括：.metadata.labels 和 .spec.template.metadata.labels。 前者无需配置（以后者为默认值），并且不会影响ReplicationController的默认行为。.spec.selector对后者产生影响。** RC 通过 .spec.selector 判断当前的Pod数目，所以不能出现量RC 他们 .spec.template.metadata.labels 的配置相同 **。 .spec.selector的默认配置和.spec.template.metadata.labels一致。
- 通过修改.spec.replicas = 0 来停止正在运行的实例。
- 通过--cascade=false 参数可以只删除控制器，而不删除pod实例！重建控制器时，只要.spec.selector一致，新的控制器可以重新接管旧的Pod
  
#  ReplicaSet

第二代RC，RpelicationController和RelicaSet的区别在于：RelicaSet支持新一代的选择器（set-based）！

# Deployments

Deployments 在RS的基础上提供了版本升级/版本回滚的功能，用户应该使用Deploy来部署应用，而不是使用RS！

[参考](https://kubernetes.io/docs/concepts/workloads/controllers/deployment/)

# DaemonSet

DaemonSet在所有节点运行一个完全一样的Pod副本，适合部署以下服务：

- 存储守护进程，如glusterd,ceph
- 日志收集器，如fluentd，logstash
- 监控进程

特性：

- 支持通过.spec.template.spec.affinity或者.spec.template.spec.nodeSelector，使pod只在部分节点运行
- DaemonSet即使Kubernetes的调度器没有工作，也能正常工作（即DaemonSet Controller本身能够完成调度），1.12版本以后由于调度器引入了preemption和优先级机制，默认场景下DaemonSet的调度同样被纳入到Scheduler中。此时，schduler是通过nodeAffinity来实现调度的。
- 当DaemonSet调度Pod时，是忽略node.kubernetes.io/unschedulable:NoSchedule标签的
- 当修改DaemonSet的模板时旧的Pod不会同步更新，需要手工删除旧Pod。

# Garbage Collection

K8S再删除对象（cascading deletion）时会同步删除该对象的依赖项，[参考](https://kubernetes.io/docs/concepts/workloads/controllers/garbage-collection/)。

默认的垃圾收集策略是 orphan。因此，除非指定其它的垃圾收集策略，否则所有 Dependent 对象使用的都是 orphan 策略。

通过为 Owner 对象设置 deleteOptions.propagationPolicy 字段，可以控制级联删除策略。

cascading deletion有两种模式：

## Background 
  Kubernetes 会立即删除 Owner 对象，然后垃圾收集器会在后台删除这些 Dependent。
## Foreground

根对象首先进入 “删除中” 状态该状态下：
- REST API可见
- 会配置deletionTimestamp 字段
- root对象的metadata.finalizers 字段包含了值 “foregroundDeletion”

一旦被设置为 “删除中” 状态，垃圾收集器会删除对象的所有 Dependent。垃圾收集器删除了所有 “Blocking” 的 Dependent（对象的 ownerReference.blockOwnerDeletion=true）之后，它会删除 Owner 对象。

## 手工指定依赖
通过在指定metadata.ownerReferences可以指定Pod的依赖关系，大多数情况下这些依赖关系是自动由Kubernetes创建的

```yaml
apiVersion: v1
kind: Pod
metadata:
  ...
  ownerReferences:
  - apiVersion: apps/v1
    controller: true
    blockOwnerDeletion: true
    kind: ReplicaSet
    name: my-repset
    uid: d9607e19-f88f-11e6-a518-42010a800195
  ...
```

# TTL Controller

用来清理已经处于完成状态（Complete Or Failed）的Job，在Job中配置如下：

```shell
apiVersion: batch/v1
kind: Job
metadata:
  name: pi-with-ttl
spec:
  ttlSecondsAfterFinished: 100
  template:
    spec:
      containers:
      - name: pi
        image: perl
        command: ["perl",  "-Mbignum=bpi", "-wle", "print bpi(2000)"]
      restartPolicy: Never
```