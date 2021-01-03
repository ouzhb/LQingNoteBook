# 安装部署CRI-O

```shell
OS=CentOS_7
VERSION=1.17
curl -L -o /etc/yum.repos.d/devel:kubic:libcontainers:stable.repo https://download.opensuse.org/repositories/devel:/kubic:/libcontainers:/stable/$OS/devel:kubic:libcontainers:stable.repo
curl -L -o /etc/yum.repos.d/devel:kubic:libcontainers:stable:cri-o:$VERSION.repo https://download.opensuse.org/repositories/devel:kubic:libcontainers:stable:cri-o:$VERSION/$OS/devel:kubic:libcontainers:stable:cri-o:$VERSION.repo
yum install cri-o-1.17.5-2.el7 podman-2.2.1-1.el7 

```
主要配置文件目录：

- /etc/containers
    - registries.conf、registries.conf.d/* 镜像仓库相关配置
    - storage.conf 存储相关配置(graphroot指定存储根目录)
    - policy.json 权限配置文件
    - certs.d 证书目录
    - oci 
   
-  /etc/crio/crio.conf 主配置文件

- /etc/crictl.yaml 客户端工具配置文件

修改配置后启动cri-o
```shell
systemctl enable crio
systemctl start crio
curl -v --unix-socket /var/run/crio/crio.sock http://localhost/info | jq
```

#  导入k8s镜像

由于crictl工具不能用来导入本地镜像，所以我们额外安装podman作为镜像工具。

```shell
wget --ftp-user=sdpuser2 --ftp-password=0V92trNW ftp://sdpftp.cl.sdp/k8s/k8s-1.16.13-image.zip
unzip k8s-1.16.13-image.zip

cd k8s-1.16.13-image

podman image load < k8s.gcr.io_coredns_1.6.2.image
podman image load < k8s.gcr.io_etcd_3.3.15-0.image
podman image load < k8s.gcr.io_kube-apiserver_v1.16.13.image
podman image load < k8s.gcr.io_kube-controller-manager_v1.16.13.image
podman image load < k8s.gcr.io_kube-proxy_v1.16.13.image
podman image load < k8s.gcr.io_kube-scheduler_v1.16.13.image
podman image load < k8s.gcr.io_pause_3.1.image
podman image load < nd-sdp-keepalived.1.1.image
podman image load < nginx_nginx-ingress_1.5.4.image
podman image load < quay.io_coreos_flannel_v0.12.0-amd64.image
podman images 
```

# 加入k8s节点
加入前请删除节点，并重启服务器
```shell

kubeadm reset phase cleanup-node       
reboot
K8S_MAX_POD=64
KUBELET_DIR=/data/var/lib/kubelet
cat << EOF > /etc/sysconfig/kubelet
KUBELET_EXTRA_ARGS=--max-pods=${K8S_MAX_POD} --root-dir=${KUBELET_DIR}  --container-runtime=remote --runtime-request-timeout=15m --container-runtime-endpoint=unix:///run/crio/crio.sock --cgroup-driver=systemd 
EOF

kubeadm join <ip:port>  --token <...>     --discovery-token-ca-cert-hash <...>  --cri-socket /var/run/crio/crio.sock
```

# 参考
[cri-o#installing-cri-o](https://github.com/cri-o/cri-o#installing-cri-o)