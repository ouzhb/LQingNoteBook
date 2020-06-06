# KubeVirt

## 1. 安装

KubeVirt本身依赖libvirt套件，因此需要在node上安装以下依赖：

```shell
# 安装以下依赖
yum -y install qemu-kvm libvirt virt-install bridge-utils

# 使用下面命令检查环境是否支持虚拟化
virt-host-validate qemu

```

在Kubernetes中通过Operator安装：

```shell
kubectl create namespace kubevirt
kubectl create configmap -n kubevirt kubevirt-config --from-literal debug.useEmulation=true

# 截止到2020.05.18 最新版本是v0.29.0
export VERSION=v0.29.0
kubectl apply -f https://github.com/kubevirt/kubevirt/releases/download/${VERSION}/kubevirt-operator.yaml
kubectl apply -f https://github.com/kubevirt/kubevirt/releases/download/${VERSION}/kubevirt-cr.yaml

# 检查virt相关服务是否就绪
kubectl -n kubevirt wait kv kubevirt --for condition=Available

```

## 2. 网络插件

如果用户希望虚拟机拥有完整的IP地址（基于L2的负载均衡），那么需要在Kubernetes集群上安装OpenSwitch的CNI插件。

安装Openvswitch插件可以参考以下文章:

- 宿主机安装[Openvswitch](https://gist.github.com/umardx/a31bf6a13600a55c0d07d4ca33133834)


## 3.虚拟机镜像

略～～

# 参考

[安装](https://kubevirt.io/user-guide/#/installation/installation)

