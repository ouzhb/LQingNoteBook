# 1.背景

## 1. vSphere

- vSphere：VMWare退出的用于云计算场景的虚拟化平台，包括：vCenter 、vMotion 等套件

- ESXi：企业级的[hypervisor](https://en.wikipedia.org/wiki/Hypervisor)，有独立的内核和虚拟化环境，直接安装在物理设备上（可以理解为是Linux+KVM）。

## 2. vSphere with Tanzu

在vSphere群集上启用Tanzu后，vSphere可提供直接在ESXi主机上运行Kubernetes工作负载，使vCenter同时具备管理：虚拟机和vSphere Pods两种工作复杂的能力。

- **VMware Tanzu™ Kubernetes Grid™ Service**：在vSphere中部署多个K8S集群
- **vSphere Pods**：在概念上等价于K8S中的Pod，但是底层实际上是ESXI的精简虚拟机，有更强的隔离性，其底层的操作系统是**Photon OS**（VMware专为ESXi定制的容器操作系统，系统内置Docker），并兼容**OCI**标准的所有镜像。
  - 在容器中使用超级权限，而不用担心权限逃逸
  - Local Path Volume 是容器独占的
  - 容器独占整个虚拟机的网络，Pod有更好的网络性能
- **Tanzu Kubernetes Clusters**：
  - 通过Kubernetes Grid一键部署
  - 兼容Kubernetes的开源版本

## 2. 相关项目

一下是Github上，Tanzu项目下的主要仓库：

- Octant：图形化的K8S管理平台，类似Rancher、KubeSphere GUI

- Velero：Kubernetes集群备份、迁移工具

- Sonobuoy：Kuberenetes API 一致新检查工具

- antrea：基于Open Switch的k8s网络插件

- Carvel ：应用构建、配置、部署工具集
- Pinniped：K8S集群的认证管理工具
- vm-operator-api：在Kubernetes中集成vSphere虚拟机的API



# 参考

[GitHub](https://github.com/vmware-tanzu)