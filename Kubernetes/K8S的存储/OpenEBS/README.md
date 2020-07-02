# OpenEBS

OpenEBS 是 MayaData 开源的一款专门应用于 Kubernetes 的CAS存储方案。MayaData 是一家针对K8s提供关联服务的创业公司，主要开源了 LitmusChaos（提供K8S集群混乱测试工具） 和 OpenEBS 俩个产品。

## 安装部署

安装部署参考： https://docs.openebs.io/docs/next/installation.html。

需要注意以下几点：

- 使用cStor和Jiva时，需要在node上安装iSCSI 客户端
- 如果node上原有其他iSCSI连接，可能会导致Pod挂载Volume失败
- 使用yaml文件安装时，注意修改openebs-ndm-config中ndm自动发现磁盘的filter规则

整体架构如下：

![](https://docs.openebs.io/docs/assets/svg/openebs-arch.svg)

## 管理面

安装完成后，**管理平面**包括以下服务：

- maya-apiserver：提供 OpenEBS REST APIs，同时还负责创建每个PVC的 Controller Pod 和 Replica Pod。 
- openebs-admission-server：准入控制器，该服务会拦截用户发送到Kubernetes API的PV/PVC请求，判断用户的操作是否被允许。
- openebs-ndm：daemonset，检查本地磁盘信息，创建 block device 资源（bd可以由ndm自动创建，也可以手工创建SPC资源，但是后者似乎是老版本的方案）
- openebs-ndm-operator
- openebs-provisioner：接受PVC请求，动态创建PV并绑定。这个过程中会和 maya-apiserver 进行交互
- openebs-localpv-provisioner 
- openebs-snapshot-operator

## 数据面

OpenEBS 中数据面提供了卷的实际 IO path，不同的存储引擎实现有所不同。

OpenEBS当前支持三种存储引擎：cStor、Jiva、LocalPV

### 1. cStor

cStor 是基于C语言实现存储引擎，支持以下特性：

- 使用Node上的普通磁盘，为Pod提供基于iSCSI的网络存储服务
- 提供副本、快照、Clone、瘦分配、一致性、数据弹性扩容等商业存储的特性

cStor工作中涉及到的CRD：

- BlockDevice：表示node上的一个存储设备
	- BlockDevice 被分配给 StoragePoolClaim 时，OpenEBS 会自动创建相应的 BlockDeviceClaims 资源
	
- StoragePoolClaim：逻辑上的数据存储池，由不同节点上的若干个CStorPool构成
	- 通过修改 blockDeviceList 列表，可以随时扩容 StoragePoolClaim ，包括：CStorPool的数量、容量
	
- CStorPool：管理同一个主机下的若干个 BlockDevice
	- 每一个CStorPool都是独立的，存储了不同数量的CStorVolumeReplica
	- CStorPool 中的磁盘支持不同的raid方式，包括：striped, mirrored, raidz, raidz2
	
- CStorVolume：每个PVC对应一个CStorVolume资源，表示逻辑上的Volume

- CStorVolumeReplica：CStorVolume对应的数据副本，分布在不同的CStorPool中
    - 用户写入volume时，所有CStorVolumeReplica都会写入数据。假设 CStorVolume配置为3副本，只有2个以上CVR正常时可以写入数据，否则CStorVolume会变成只读状态。
    - 当故障CVR恢复后，不同CVR之间会相互同步数据。

cStor的数据面，包括以下服务：

- cStor Pool Pods：对应一个 CStorPool 资源，负责存储实际数据
- cStor Target Pods：提供一个虚拟的iSCSI Target
    - 当pod连接到这个Target时，数据被转发到Volume Replicas上。
    - 在多个Volume Replicas提供选举，数据复制的能力。

上述两种Pods运行时，注入了maya-apiserver 的 sidecar 容器：

- maya-volume-exporter：采集卷/Pool的统计信息（如，IOPS、容量......）
- cstor-volume-mgmt：同步配置信息

参考：https://docs.openebs.io/docs/next/cstor.html

### 2. Jiva

略

参考：https://docs.openebs.io/docs/next/jiva.html

### 3. LocalPV

LocalPV 包括：Local PV Hostpath 和 Local PV Device 俩种类型，只适合部分本身支持数据高可用的应用。

OpenEBS LocalPV 相比于 Kubernetes 原生 LocalPV，有以下新特性：

- 动态 PV Provisioners （自动创建本地目录，删除PVC时清楚本地文件）
- 监控存储设备的健康状态
- 具有容量管理功能（实际测试中发现，不能限制Volume的写入的数据大小）
- 如果后端使用ZFS等具备快照、Clone等功能的文件系统，那么这些特性可以被复用到volume中
- 支持使用Velero进行数据备份

参考：https://docs.openebs.io/docs/next/localpv.html

## 参考

[architecture](https://docs.openebs.io/docs/next/architecture.html)

[usecases](https://docs.openebs.io/docs/next/usecases.html)