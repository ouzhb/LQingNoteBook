# Kubernetes Storage

Kubernetes中保存持久化的数据可以通过两种方式:
- Volumes
- Persistent Volumes + PersistentVolumeClaims
  

## Volumes

早期Kubernetes使用volumes保存持久化数据。这种方式无法管理volumes的生命周期（volumes的生命周期和pod一致），用户难以统一管理。
volumes支持[多种类型](https://kubernetes.io/docs/concepts/storage/volumes/#types-of-volumes),以下几个可以关注：

|volumes|说明|
|----|----|
|[configMap](https://kubernetes.io/docs/concepts/storage/volumes/#types-of-volumes)|将ConfigMap中的内容以文件的形式挂载到pod|
|[downwardAPI](https://kubernetes.io/docs/concepts/storage/volumes/#downwardapi)|[downwardAPI](https://www.qikqiak.com/post/use-downward-api-get-pod-info/)可以将Pod的元数据信息通过文件或者环境变量的方式注入到Pod中。|
|[emptyDir](https://kubernetes.io/docs/concepts/storage/volumes/#emptydir)|Pod的临时空间，pod删除后emptyDir中的数据会被清除。注意，crashes时emptyDir中的数据依然还在。配置emptyDir.medium=Memory时，K8s会创建tmpfs作为后备存储|
|[hostPath](https://kubernetes.io/docs/concepts/storage/volumes/#hostpath)|最常用！！！|
|[projected](https://kubernetes.io/docs/concepts/storage/volumes/#projected)|允许用户挂载secret、downwardAPI、configMap到pod内部|
|[glusterfs](https://kubernetes.io/docs/concepts/storage/volumes/#glusterfs)|挂载glusterfs预先创建的Volumes|


## Persistent Volumes + PersistentVolumeClaims

PersistentVolume（PV）是群集中由管理员配置的一块存储，其后端可能是NFS、ISCSI、Ceph、本地目录等不同设备。

PersistentVolumeClaim（PVC）是用户使用存储的请求。

用户使用存储需要构建PVC，Kubernetes根据PVC从当前的PV中挑选一个，将PVC和PV绑定，用户即可在POD声明中使用该存储。

PV和PVC相比于Volumes的优势在于，以下几点：
- 存储的生命周期和Pod的生命周期分离，用户能够独立管理存储
- 支持更多类型的存储后端
- 支持权限、挂载管理等更多高级功能


### PVC&PV的生命周期

  从用户申请存储到使用完成，PVC和PV之间经历以下几个阶段：
  
  1. Provisioning：PVC和PV的匹配阶段，包括静态、动态。
        - 静态：预先创建多个PV
        - 动态：当前的PV都和PVC不匹配时，PVC请求指定的storageClass自动创建PV
  2. Binding：绑定后PVC等到的容量不小于申请的值（可能超过）。如果找不到匹配PV，PVC会一直保持未绑定状态。
  3. Using：Pod使用PVC。对于正在使用的PVC，K8s提供保护机制：当PVC被Pod使用时，删除PVC的命令不会立即生效。当PV被绑定到PVC时，删除PV的命令不会立即生效。
  4. Reclaiming：删除PVC引发的资源回收动作，回收策略包括以下。
      - Retain：手工回收，PV保留，状态为“Retain”，但是其他PVC不能使用这个PV。用户需要手工删除数据，并删除PV。
      - Delete：同时删除PV
      - Recycle：已经被废弃
  
### 配置参数

[PV参考](https://kubernetes.io/docs/concepts/storage/persistent-volumes/#persistent-volumes)

[PVC参考](https://kubernetes.io/docs/concepts/storage/persistent-volumes/#persistentvolumeclaims)

### 其他功能

- 卷扩展
- 快照功能

## 动态申请PVC
动态PV配置允许按需创建PV，不使用动态PV时用户申请PVC前需要手工创建一个PV。动态PV特性基于StorageClass插件（后端存储的配置器）。

当用户没有在PVC中指定storageClassName时，使用预先配置DefaultStorageClass作为动态存储申请PV。当用户指定storageClassName=“”时，PVC不启动动态申请功能，从已经创建的PV中选择。

### StorageClass

StorageClass为用户提供了一种描述底层存储类型的方法，可以认为是glusterfs、Ceph、S3等存储设备，对接到K8S的配置文件。

StorageClass包括以下字段：

|字段|说明|
|----|----|
|Provisioner|表示StorageClass所需要使用的存储插件。Kubernetes内部提供的插件以kubernetes.io开头。[参考](https://kubernetes.io/docs/concepts/storage/storage-classes/#the-storageclass-resource)。用户也可以使用[外部存储插件](https://github.com/kubernetes-incubator/external-storage).|
|Reclaim Policy|存储回收策略，包括Delete、Retain，默认情况下是Delete。注意PV、PVC、StorageClass都可以配置回收策略，这几个策略有相互覆盖的关系。|
|Mount Options|动态申请的卷，遵从这个配置|
|Volume Binding Mode|默认情况下为Immediate，表示一旦创建了PVC，就会发生卷绑定和动态配置。此外还有WaitForFirstConsumer|
|Allowed Topologies|拓扑限定|
|parameters|与存储后端相关的配置|


## 参考

https://kubernetes.io/docs/concepts/storage/


