# ROOK

Rook是专用于Cloud-Native环境的文件、块、对象存储管理服务。

Rook支持对Ceph、NFS、Minio等存储系统进行：自动部署、启动、配置、分配（provisioning）、扩容/缩容、升级、迁移、灾难恢复、监控，以及资源管理。

[官网](https://rook.io)

# Ceph Storage Quickstart

当前Rook提供多个存储集群的[Storage Provider](https://github.com/rook/rook/blob/master/README.md#project-status)，目前只有Ceph的API接口是稳定版本接口，其余存储均为Alpha版本。

## 部署[Rook operator](https://rook.io/docs/rook/v0.9/ceph-quickstart.html#deploy-the-rook-operator)

Operator安装完成后包括以下资源：

|名称|类型|资源|
|----|----|----|
|rook-ceph-operator|Deployment||
|rook-ceph-agent|daemonset|应该是operator自动创建的|
|rook-discover|daemonset|应该是operator自动创建的|

## 创建Ceph集群
部署完成Operator后，用户可以定义 CephCluster 等对象来创建Ceph集群，[参考](https://rook.io/docs/rook/v0.9/ceph-cluster-crd.html)。

用户定义CephCluster对象后会自动创建mon，mgr，osd等实例。


## 使用Ceph

### Block Storage方式
块存储允许rdb卷挂载到单个pod中。

通过下面两个步骤即可在Pod中使用RBD卷：

1. 创建一个CephBlockPool、StorageClass

2. 通过PV/PVC动态申请存储资源

### Object Storage方式
Object 方式允许用户开放一个S3 API进行数据存储、读取。

当前spark大批量存取数据，使用S3也是一个可以考虑的方案。

通过下面的步骤创建Object Storage：

1. 创建CephObjectStore对象，该对象会自动创建metadataPool和dataPool，以及在指定节点创建RGW POD
2. 创建CephObjectStoreUser对象，该对象会自动在指定目录项创建对应的screte，包括AccessKey 和 SecretKey 。同过base64 命令能够获取秘钥明文
```bash
kubectl -n rook-ceph get secret rook-ceph-object-user-my-store-my-user -o yaml | grep AccessKey | awk '{print $2}' | base64 --decode
kubectl -n rook-ceph get secret rook-ceph-object-user-my-store-my-user -o yaml | grep SecretKey | awk '{print $2}' | base64 --decode
```
3. 通过s3cmd可以验证可用性：

```bash

# AWS_HOST: 格式为{RGW-DNS-POD-DNS}.{ceph-cluster-name}
# AWS_ENDPOINT: RGW对应的SVC IP地址
# AWS_ACCESS_KEY_ID：CephObjectStoreUser对象中获取
# AWS_SECRET_ACCESS_KEY：CephObjectStoreUser对象中获取
export AWS_HOST=rook-ceph-rgw-my-store.rook-ceph
export AWS_ENDPOINT=10.106.64.122:80 
export AWS_ACCESS_KEY_ID=T4FXYWJMMNIXBIY2KDDJ
export AWS_SECRET_ACCESS_KEY=pywu02FKrNfBZUUGnZWDNgtobmJybQQv6HNRNahH

# S3常用命令
s3cmd mb --no-ssl --host=${AWS_HOST} --host-bucket=s3://rookbucket
s3cmd ls --no-ssl --host=${AWS_HOST}
echo "Hello Rook" > /tmp/rookObj
s3cmd put /tmp/rookObj --no-ssl --host=${AWS_HOST} --host-bucket=s3://rookbucket
s3cmd get s3://rookbucket/rookObj /tmp/rookObj-download --no-ssl --host=${AWS_HOST} --host-bucket=s3://rookbucket
cat /tmp/rookObj-download

```

### Shared File System方式

当前cephfs并不是一个十分稳定的特性，在生产方式中建议慎用该方式保存重要数据。

通过创建CephFilesystem 对象可以实例化Ceph的文件系统,自动创建mds实例。当用户需要使用共享目录时，可以通过flexVolume将Volume挂载到Pod的指定路径

## [前期准备](https://rook.io/docs/rook/v0.9/k8s-pre-reqs.html)
- 支持Kubernetes v1.8以及以上版本
- [RBAC支持](https://rook.io/docs/rook/v0.9/rbac.html): 需要创建若干Kubernetes对象
- [FlexVolume 支持](https://rook.io/docs/rook/v0.9/flexvolume.html)
- [helm-operator 配置](https://rook.io/docs/rook/v0.9/helm-operator.html)

## 参考

[Ceph Docker](https://hub.docker.com/r/ceph/ceph/tags/)

[Rook Docker](https://hub.docker.com/r/rook/rook/tags)

[Cluster CRD](https://rook.io/docs/rook/v0.9/ceph-cluster-crd.html#storage-selection-settings)

[Block Pool CRD](https://rook.io/docs/rook/v0.9/ceph-pool-crd.html)

[Object Store CRD](https://rook.io/docs/rook/v0.9/ceph-object-store-crd.html)

[Object Store User CRD](https://rook.io/docs/rook/v0.9/ceph-object-store-user-crd.html)

[Shared File System CRD](https://rook.io/docs/rook/v0.9/ceph-filesystem-crd.html)

