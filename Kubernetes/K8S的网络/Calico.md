# Calico

- 支持的存储环境
    - Kubernetes API datastore（使用该存储时要求 node 数量小于50，否则使用 部署  Typha 服务）
    - 外部 etcd（主要用于混合部署场景，如 OpenStack 和 Kubernetes 混合部署的场景）
    
- *workload* ：**workload endpoint**是指workload用于连接Calico network的网络接口

    - pod
    - VM

- IP Pools

    - 通常IP Pools 是 pod CIDR的子集

    ```yaml
    apiVersion: projectcalico.org/v3
    kind: IPPool
    metadata:
      name: pool1
    spec:
      cidr: 192.168.0.0/18
      ipipMode: Never
      natOutgoing: true
      disabled: false  # 不会在池中创建具有地址的新Pod，但仍会将具有这些地址的Pod识别为Calico网络的一部分。
      nodeSelector: all() # 那些节点使用这个ippool
    ```

    

# Calicoctl

常用命令列表

```shell
export KUBECONFIG=/root/.kube/config 
export DATASTORE_TYPE=kubernetes

calicoctl get nodes
calicoctl get ippools
```

# 附录

## 1. Kubernetes API datastore

Calico 中的 Kubernetes API datastore 是指和 k8s 共用 etcd，

~~PS：**没搞清楚Calico是直接访问ETCD，还是通过API-Service的接口访问etcd？如果是通过API-Service的接口的话，是普通的存储接口，还是网络相关的专用接口。**~~

**Calico实际上将需要存储的数据，转化成的CRD资源来存储，并没有直接访问ETCD或者调用什么特殊的API-Service。Calico定义的CRD[【参考】](https://docs.projectcalico.org/manifests/crds.yaml)**。

使用```calicoctl```可以查看ETCD的这些自定义资源。

### 访问ETCD

使用kubeadm安装时，etcd运行的证书保存在```/etc/kubernetes/pki/etcd```目录下，并且etcd是通过静态pod的模式，使用本地网络部署的。

API-Service 访问etcd时使用的证书是，并且是通过localhost去访问的：

```
/etc/kubernetes/pki/etcd/ca.crt（这个文件签发了etcd服务使用的certificate，以及其他各种certificate）
/etc/kubernetes/pki/apiserver-etcd-client.key
/etc/kubernetes/pki/apiserver-etcd-client.crt
```

通过下面的命令能够访问，K8S的etcd存储

```
ETCDCTL_API=3 etcdctl --endpoints=https://127.0.0.1:2379 \
--cacert=/etc/kubernetes/pki/etcd/ca.crt \
--cert=/etc/kubernetes/pki/apiserver-etcd-client.crt \
--key=/etc/kubernetes/pki/apiserver-etcd-client.key \
get /calico --prefix --keys-only
```



## 2. Typha

https://docs.projectcalico.org/reference/typha/overview

https://github.com/projectcalico/typha

# 参考

[Calico Hard Way](https://docs.projectcalico.org/getting-started/kubernetes/hardway/overview)

[本地部署](https://docs.projectcalico.org/getting-started/kubernetes/self-managed-onprem/onpremises)