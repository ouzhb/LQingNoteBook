# 安装包下载

从Github可以获取[tarballs ](https://github.com/kubernetes/kubernetes/releases/tag/v1.13.0)，解压后执行以下命令可以获取所有安装需要的二进制文件，以及部分镜像。

```shell
./kubernetes/cluster/centos/build.sh download # 下载所有文件到/tmp/downloads

./kubernetes/cluster/centos/build.sh unpack #  解压下载到文件

# 所有压缩包的二进制文件会被抽取到./kubernetes/cluster/centos/binaries，包含etcd、docker、Kubernetes、flannal的可执行程序。

# /tmp/downloads/kubernetes/server/bin 目录下包含kube-apiserver等组件的docker镜像

```

# 裸部署

当获取了所有组件的binaries文件，用户可以通过配置service文件将所有服务纳入systemctl管理。

## 证书配置

当使用HTTPS访问集群时，需要为Kubernetes生成证书。官方提供了使用easyrsa制作证书的[一般步骤](https://kubernetes.io/docs/concepts/cluster-administration/certificates/)。

IBNS的证书似乎是使用Openssl生成的，所有证书文件和证书配置放在/etc/kubernetes目录。

使用./kubernetes/cluster/centos/make-ca-cert.sh 可以一键生成cert证书

```shell
export CERT_GROUP=root
./make-ca-cert.sh ${MASTER_IP}

#生成的证书位于/srv/kubernetes 目录下
```
证书文件包括：

|证书文件|配置节点|文件名|备注|
|----|----|----|----|     
|CA_CERT|apiserver的运行节点|/srv/kubernetes/ca.crt|
|MASTER_CERT|apiserver的运行节点|/srv/kubernetes/server.crt|
|MASTER_KEY|apiserver的运行节点|/srv/kubernetes/server.key|
|KUBELET_CERT|kubelet运行的节点|/srv/kubernetes/kubecfg.crt|
|KUBELET_KEY|kubelet运行的节点|/srv/kubernetes/kubecfg.key|

apiserver还需要配置一个TOKEN，并在token的值写入到/var/lib/kube-apiserver/known_tokens.csv中。

在apiserver中添加以下配置：
``` shell
--client-ca-file=/yourdirectory/ca.crt
--tls-cert-file=/yourdirectory/server.crt
--tls-private-key-file=/yourdirectory/server.key
```

在各个节点分发配置证书参考以下配置：


### 为kubectl配置证书
``` shell
# 如果不使用HTTPS配置，在kubectl节点配置

kubectl config set-cluster $CLUSTER_NAME --server=http://$MASTER_IP --insecure-skip-tls-verify=true

# 否则

kubectl config set-cluster $CLUSTER_NAME --certificate-authority=$CA_CERT --embed-certs=true --server=https://$MASTER_IP

kubectl config set-credentials $USER --client-certificate=$CLI_CERT --client-key=$CLI_KEY --embed-certs=true --token=$TOKEN

# 指定当前是使用集群配置
kubectl config set-context $CONTEXT_NAME --cluster=$CLUSTER_NAME --user=$USER
kubectl config use-context $CONTEXT_NAME
```
### 为kubelets和kube-proxy配置证书
创建以下的kubeconfig文件,并在启动时指定--kubeconfig="path/kubeconfig"

IBNS项目中还指定了--experimental-bootstrap-kubeconfig？？？

```yaml
apiVersion: v1
kind: Config
users:
- name: kubelet
  user:
    token: ${KUBELET_TOKEN}
clusters:
- name: local
  cluster:
    certificate-authority: /srv/kubernetes/ca.crt
contexts:
- context:
    cluster: local
    user: kubelet
  name: service-account-context
current-context: service-account-context
```

## 网络规划

- 配置CIDR subnet : 配置10.10.0.0/16作为整个集群的网段，该集群可容纳256个nodes，对应的网段为10.10.0.0/24 到 10.10.255.0/24
- 配置SERVICE_CLUSTER_IP_RANGE="10.0.0.0/16"
- 配置MASTER_IP
- 配置net.ipv4.ip_forward = 1  
- 配置[flannel](https://github.com/coreos/flannel)---参考IBNS的方案flannel是容器化部署的。

# yum安装

## Kubernetes官方yum源

```shell
cat <<EOF > /etc/yum.repos.d/kubernetes.repo
[kubernetes]
name=Kubernetes
baseurl=https://packages.cloud.google.com/yum/repos/kubernetes-el7-x86_64
enabled=1
gpgcheck=1
repo_gpgcheck=1
gpgkey=https://packages.cloud.google.com/yum/doc/yum-key.gpg https://packages.cloud.google.com/yum/doc/rpm-package-key.gpg
EOF
```

## 安装Kubectl

```shell
yum install -y kubectl
yum install bash-completion -y
echo "source <(kubectl completion bash)" >> ~/.bashrc
```
## 安装docker

需要注意一点：如果node原先安装了Docker，那么安装Kubernetes时需要将docker创建docker0 网桥删除。
```shell
iptables -t nat -F
ip link set docker0 down
ip link delete docker0
```
同时用户docker的需要进行和Kubernetes兼容的一些配置：[参考](https://kubernetes.io/docs/setup/scratch/#docker)

## 安装Kubernetes集群

添加Kubernetes官方源后，可以找到以下两个rpm：

|rpm包名|包含组件|备注|
|----|----|----|
|kubernetes-master|kube-apiserver、kube-controller-manager、kube-scheduler||
|kubernetes-node|docker、 kube-proxy、kubelet、kubectl||

上述组件的配置文件位于/etc/kubernetes。

官方建议kubelet、kube-proxy、docker本地化部署，其余组件通过放在container中运行。

PS： 容器化部署etcd等组件实际上使用了Kubernetes的[Static Pods](https://kubernetes.io/docs/tasks/administer-cluster/static-pod/) 特性，用户只要将etcd等组件的yaml配置放到--pod-manifest-path目录下，当kubelet启动时会同步拉起对应的pod

使用yum手工安装Kubernetes的一般步骤如下：

- 添加yum源
- 所有节点安装kubernetes-node
- 在master节点准备证书文件，并分发到所有节点（可选）
- 修改/etc/kubernetes目录下的kube-proxy，kubelet的配置文件
- 在master节点配置kube-apiserver、kube-controller-manager、kube-scheduler、etcd的yaml，并放到pod-manifest-path对应目录
- 启动master的docker、kube-proxy、kubelet
- 启动其他节点的服务


# 参考

[自定义安装](https://kubernetes.io/docs/setup/scratch/)

