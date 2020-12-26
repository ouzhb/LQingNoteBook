# 说明

迁移无锡生产环境**etcd集群**到新节点，并且剥离原有的节点**Kubernetes控制面**到新节点。

## 1. 环境信息

| 内网IP       | 当前服务                                | ETCD镜像版本              | 备注 |
| ------------ | --------------------------------------- | ------------------------- | ---- |
| 10.33.38.185 | Kubernetes Controller、etcd、prometheus | k8s.gcr.io/etcd:3.2.24    |      |
| 10.33.38.186 | Kubernetes Controlleretcd               | k8s.gcr.io/etcd:3.2.24    |      |
| 10.33.38.187 | Kubernetes Controller、etcd、nginx-ws   | k8s.gcr.io/etcd:3.2.24    |      |
| 10.33.38.202 | ali环境etcd服务                         | k8s.gcr.io/etcd:3.3.15-0  | 新增 |
| 10.33.38.140 | ali环境etcd服务                         | k8s.gcr.io/etcd :3.3.15-0 | 新增 |
| 10.33.38.152 | ali环境etcd服务                         | k8s.gcr.io/etcd :3.3.15-0 | 新增 |

## 2.方案

参考一下方案执行迁移工作：

- 扩容原有ETCD节点为6节点
- 在Kubernetes上新添加三个Controller节点，并且指向新etcd节点
- 修改keepalived配置到新节点
- 降级原有Controller节点为Worker
- 删除原有etcd节点

# 操作步骤

 [参考](https://github.com/LinQing2017/DevOpsTools/tree/master/k8s-awstools/script/k8s%E8%BF%90%E7%BB%B4%E6%96%87%E6%A1%A3/Kubernetes%E7%AE%A1%E7%90%86%E6%9C%8D%E5%8A%A1%E8%BF%81%E7%A7%BB)



