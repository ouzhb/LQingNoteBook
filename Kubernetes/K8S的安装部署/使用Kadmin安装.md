# 安装 kubeadm（控制面单点）

## 1. 前提条件

- 关闭swap

    - echo vm.swappiness=0 >> /etc/sysctl.conf
    - 注释掉fstab里的swap

- 进行网络参数配置

```shell
sudo modprobe br_netfilter
cat <<EOF | sudo tee /etc/sysctl.d/k8s.conf
net.bridge.bridge-nf-call-ip6tables = 1
net.bridge.bridge-nf-call-iptables = 1
EOF
sudo sysctl --system
```

## 2. 安装kubeadm、kubelet、kubectl

Kubeadm不会替用户安装kubelet和kubectl，用户可以通过yum、二进制安装包在所有node安装这些服务。

PS：安装时需要kubelet、kubectl和kubeadm版本匹配

三个服务都可以通过下面的repo安装：

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

setenforce 0
sed -i 's/^SELINUX=enforcing$/SELINUX=permissive/' /etc/selinux/config

yum install -y kubelet kubeadm kubectl --disableexcludes=kubernetes

systemctl enable kubelet
systemctl start kubelet
```
国内源

```shell
cat <<EOF > kubernetes.repo
[kubernetes]
name=Kubernetes
baseurl=https://mirrors.aliyun.com/kubernetes/yum/repos/kubernetes-el7-x86_64
enabled=1
gpgcheck=1
repo_gpgcheck=1
gpgkey=https://mirrors.aliyun.com/kubernetes/yum/doc/yum-key.gpg https://mirrors.aliyun.com/kubernetes/yum/doc/rpm-package-key.gpg
EOF
```
## 3. kubeadmin介绍

通过kubeadmin可以快速的安装部署kubernetes集群：

- init：用于创建kubernetes中的管理面节点（control-plane node），即运行 etcd 、API Service的节点

    - init执行了哪些工作，可以参考[Kubernetes官网](https://kubernetes.io/docs/reference/setup-tools/kubeadm/kubeadm-init/)

    - 相关重要参数：

        - apiserver-advertise-address：API Service的监听地址，默认情况下如果不指定，那么走的就是默认ip地址
        - apiserver-bind-port：默认6443
        - cert-dir：证书文件目录，默认/etc/kubernetes/pki
        - config：kubeadm 配置文件
        - control-plane-endpoint：控制面板的稳定地址
        - image-repository：镜像地址，默认k8s.gcr.io
        - kubernetes-version：kubernetes版本
        - pod-network-cidr：pod分配的ip网段
        - service-cidr：Default: "10.96.0.0/12"，service分配到的ip网段

    - init命令会创建静态pod的yaml文件到/etc/kubernetes/manifests目录，包括：manager 、etcd、scheduler等

    - init命令执行完成后，可以使用以下命令配置kubectl

        ```shell
        mkdir -p $HOME/.kube
        sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
        sudo chown $(id -u):$(id -g) $HOME/.kube/config
        ```

- config：可以查看相关k8s集群的配置

    - images list：列出k8s相关镜像
    - images pull：批量拉取k8s相关镜像

- join：添加一个node到一个指定的control-plane node控制面

    ```shell

    kubeadm join 172.24.33.110:6443 --token wm28gj.xoeiagyk2epva317 \
        --discovery-token-ca-cert-hash  sha256:b6a91123760c45b7e7ae6eca075674b43b31d1103bdd890256c318640e4e5d4b

    # token可以使用 kubeadm token list 查看，默认情况下Token会在24小时后过期，此时用户可以通过 kubeadm token create 重新创建

    # cert的hash编码值，可以使用以下命令获取：
    openssl x509 -pubkey -in /etc/kubernetes/pki/ca.crt | openssl rsa -pubin    -outform der 2>/dev/null | openssl dgst -sha256 -hex | sed 's/^.* //'

    ```

默认情况，如果安装单机场景的K8S（即控制平面单机），只需要用户安装完成docker，并下载好镜像后，执行 kubeadm init 就完成了控制平面的安装。

## 4. 安装网络插件

网络插件称为Container Network Interface (CNI)，是K8S中容器间通信的接口规范，实现了这个规范后，就能提供IP地址、网关、路由、DNS等相关的网络参数。

当前K8S提供了非常多的CNI实现，并且全都是通过K8s工作负载的方式安装的。

```shell

# 安装calico插件
kubectl apply -f https://docs.projectcalico.org/manifests/calico.yaml

```

## 5. 其他配置

当使用 kubeadmin 安装时，控制面相关服务的YAML文件保存在/etc/kubernetes/manifests/目录下（包括：etcd、apiserver、controller-manager、shceduler），需要修改Kubernetes服务时，可以修改这些 YAML 文件中的配置。

```shell
# 移除master节点隔离，服务可以调度到Master节点
kubectl taint nodes --all node-role.kubernetes.io/master-

# 部署dashboad
kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/v2.0.0/aio/deploy/recommended.yaml

# 获取临时登录的token。PS：需要暴露服务到nodeport，并且创建相应的账号
kubectl -n kubernetes-dashboard describe secret $(kubectl -n kubernetes-dashboard get secret | grep kubernetes-dashboard | awk '{print $1}')

```

## 6. 清理环境

略


# 参考

[Installing kubeadm](https://kubernetes.io/docs/setup/production-environment/tools/kubeadm/install-kubeadm/)

[kubeadm Doc](https://kubernetes.io/docs/reference/setup-tools/kubeadm/kubeadm/)

[Kubernetes安装calico插件](https://docs.projectcalico.org/getting-started/kubernetes/quickstart)

[Clean Up](https://kubernetes.io/docs/setup/production-environment/tools/kubeadm/create-cluster-kubeadm/#tear-down)

[web-ui-dashboard](https://kubernetes.io/docs/tasks/access-application-cluster/web-ui-dashboard/)

[集群外访问Dashboad](https://github.com/kubernetes/dashboard/blob/master/docs/user/access-control/creating-sample-user.md)