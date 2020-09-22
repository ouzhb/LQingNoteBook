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

## 1.ETCD节点扩容

参考以下步骤进行ETCD节点的扩容，改步骤不会影响线上环境，但是需要技术部配合。

其中，步骤一有我预先操作形成压缩包提供给技术步，步骤二、三由技术部在每个待扩容节点上依次执行。

### 1.准备新节点的ETCD证书

可以在本地虚拟机上准备。

```shell
# 安装kubeadm工具
yum install -y kubelet-1.13.10
yum install -y kubectl-1.13.10 kubeadm-1.13.10

# 从无锡环境的Master上获取etcd的ca.crt以及ca.key
###############################################
#  可以使用开发集成服务的 s3_download 获取，路径为：
# /etc/etcd/ca.crt
# /etc/etcd/ca.key
###############################################

# 上传etcd-ca到/etc/kubernetes/pki/etcd
mkdir -p /etc/kubernetes/pki/etcd

# 生成证书
cd /etc/kubernetes/pki
find /etc/kubernetes/pki -not -name ca.crt -not -name ca.key -type f -delete
cat << EOF > wx-etcd-cert.yaml
apiVersion: "kubeadm.k8s.io/v1beta1"
kind: ClusterConfiguration
etcd:
    local:
        serverCertSANs:
        - "10.33.38.202"
        - "10.33.38.140"
        - "10.33.38.152"
        peerCertSANs:
        - "10.33.38.202"
        - "10.33.38.140"
        - "10.33.38.152"
EOF

kubeadm init phase certs etcd-server --config=wx-etcd-cert.yaml
kubeadm init phase certs etcd-peer --config=wx-etcd-cert.yaml
kubeadm init phase certs etcd-healthcheck-client --config=wx-etcd-cert.yaml
kubeadm init phase certs apiserver-etcd-client --config=wx-etcd-cert.yaml
mv /etc/kubernetes/pki/apiserver-etcd-client.* /etc/kubernetes/pki/etcd/

# 打包证书
tar -zcvf  wx-etcd-cert.tar.gz -C etcd .
```

### 2. ETCD扩容操作（所有扩容节点）

**ETCD 扩容时需要逐个操作，等新加入节点同步后，才可以继续操作第二个节点，每执行一个节点时建议新开窗口防止环境变量干扰。**

```shell
# 以下命令在任意master节点（如，10.33.38.185）上操作

# 申请以下环境变量
CURRENT_HOST= xxxx    # 当前要扩容的节点

# 执行以下命令在集群中添加一个节点
ETCDCTL_API=3 etcdctl \
--endpoints=https://10.33.38.185:2379,https://10.33.38.185:2379,https://10.33.38.185:2379 \
--command-timeout=30s \
--cacert=/etc/etcd/ca.crt \
--cert=/etc/etcd/apiserver-etcd-client.crt \
--key=/etc/etcd/apiserver-etcd-client.key \
member add ${CURRENT_HOST} --peer-urls=https://${CURRENT_HOST}:2380

# 上述命令返回以下类似内容，这些内容用于在${CURRENT_HOST}节点上启动 etcd 的 docker容器
###############################################################################
# ETCD_NAME="172.24.135.46"
# ETCD_INITIAL_CLUSTER="172.24.135.12=https://172.24.135.12:2380,172.24.135.46=https://172.24.135.46:2380,172.24.135.11=https://172.24.135.11:2380,172.24.135.10=https://172.24.135.10:2380"
# ETCD_INITIAL_ADVERTISE_PEER_URLS="https://172.24.135.46:2380"
# ETCD_INITIAL_CLUSTER_STATE="new"
###############################################################################

```

登录 **${CURRENT_HOST}** 执行以下操作，启动etcd进程：

```shell
# 申请以下环境变量
CURRENT_HOST= xxxx                   # 当前节点的内网IP
ETCD_NAME=xxxx                       # 当前节点的内网IP

ETCD_INITIAL_CLUSTER=                # 上一个步骤的返回值           
ETCD_INITIAL_ADVERTISE_PEER_URLS=    # 上一个步骤的返回值       
ETCD_INITIAL_CLUSTER_STATE=          # 上一个步骤的返回值       

ETCD_CONTAINER_NAME=etcd_wx
ETCD_PKI=/etc/etcd_wx
ETCD_DIR=/data/var/lib/etcd_wx
```

执行以下命令，启动etcd的docker容器：

```shell
# 获取etcd的cert文件，上传到${CURRENT_HOST},执行以下命令解压
mkdir -p ${ETCD_PKI}
tar -zxvf wx-etcd-cert.tar.gz -C ${ETCD_PKI}

# 启动docker容器
docker run --restart=always -d --network=host --name ${ETCD_CONTAINER_NAME} \
-v ${ETCD_DIR}:/var/lib/etcd \
-v ${ETCD_PKI}:/etc/pki \
k8s.gcr.io/etcd:3.2.24 \
etcd \
--name=${ETCD_NAME} \
--initial-advertise-peer-urls=${ETCD_INITIAL_ADVERTISE_PEER_URLS} \
--initial-cluster="${ETCD_INITIAL_CLUSTER}"  \
--initial-cluster-state=${ETCD_INITIAL_CLUSTER_STATE} \
--listen-peer-urls=${ETCD_INITIAL_ADVERTISE_PEER_URLS} \
--advertise-client-urls=https://${CURRENT_HOST}:2379 \
--listen-client-urls=https://${CURRENT_HOST}:2379,https://127.0.0.1:2379 \
--data-dir=/var/lib/etcd \
--cert-file=/etc/pki/server.crt \
--client-cert-auth=true \
--key-file=/etc/pki/server.key \
--peer-cert-file=/etc/pki/peer.crt \
--peer-client-cert-auth=true \
--peer-key-file=/etc/pki/peer.key \
--peer-trusted-ca-file=/etc/pki/ca.crt \
--snapshot-count=10000 \
--trusted-ca-file=/etc/pki/ca.crt
```

### 3.检查扩容是否成功（所有扩容节点）

**每次扩容后，请按照下面方式进行检查，OK后在进行下一个节点的扩容。如果一次扩容多个节点会造成ETCD脑裂，破坏集群！！**

```shell
# 在CURRENT_HOST执行以下命令

# 1. 在新扩容机器：检查etcd节点是否会退出，如果退出说明扩容
docker ps |grep ${ETCD_CONTAINER_NAME}

# 2. 在原有Master机器：检查新加节点是否已经同步完成
CURRENT_HOST= xxxx #扩容机器内网IP
ETCD_ENDPOINTS=https://${CURRENT_HOST}:2379,https://10.33.38.185:2379,https://10.33.38.186:2379,https://10.33.38.187:2379

ETCDCTL_API=3 etcdctl \
--endpoints=${ETCD_ENDPOINTS} \
--command-timeout=60s \
--cacert=/etc/etcd/ca.crt \
--cert=/etc/etcd/apiserver-etcd-client.crt \
--key=/etc/etcd/apiserver-etcd-client.key \
--write-out=table \
endpoint status

# 上述命令的输出大概如下：观察RAFT INDEX直到所有节点一致

+---------------------------+------------------+---------+---------+-----------+-----------+------------+
|         ENDPOINT          |        ID        | VERSION | DB SIZE | IS LEADER | RAFT TERM | RAFT INDEX |
+---------------------------+------------------+---------+---------+-----------+-----------+------------+
| https://10.33.38.185:2379 | d143a4e7aa9267bd |  3.2.24 |  102 MB |     false |       114 |  123548649 |
| https://10.33.38.186:2379 | 682be5c6fccdf929 |  3.2.24 |  102 MB |      true |       114 |  123548649 |
| https://10.33.38.187:2379 | c8c676401719cb14 |  3.2.24 |  102 MB |     false |       114 |  123548652 |
+---------------------------+------------------+---------+---------+-----------+-----------+------------+

```

## 2.Kubernetes控制面迁移

```shell
# Kube Node节点维护
# 节点设置维护状态

# 静止调度节点
kubectl cordon <node-name>
# 节点pod驱离，需要注意的：Daemonset 管理的pod不能驱离，并且使用local storage的pod驱离时会删除本地数据
kubectl drain --ignore-daemonsets  --delete-local-data --ignore-daemonsets <node-name>
# 解除静止调度
kubectl uncordon <node-name>
# 删除node
kubectl  delete node <node-name>

```



目标机器：10.33.38.202、10.33.38.140、10.33.38.152

- Centos版本：Centos 6.5 64bit
- Docker 版本是否一样，是否需要升级
- 迁移之后 ETCD Endpoint 发生变化

- 先将etcd从3个扩容成6。当ETCD状态稳定后。
- 部署新的三个master，这三个master的etcd地址填写性的etcd节点。
- 删除原有master
- 删除原有etcd
- 删除旧的master，重新作为work加入到集群



# 附录

## 1. 操作无锡环境ETCD

找一个master部署以下服务：

```shell
apiVersion: apps/v1
kind: Deployment
metadata:
  name: etcdctl
spec:
  selector:
    matchLabels:
      name: etcdctl
  template:
    metadata:
      labels:
        name: etcdctl
    spec:
      tolerations:
        - key: node-role.kubernetes.io/master
          effect: NoSchedule
      containers:
        - name: etcdctl
          image: k8s.gcr.io/etcd:3.3.10
          imagePullPolicy: IfNotPresent
          command:
            - sleep
            - "3600000"
          volumeMounts:
            - mountPath: /etc/etcd
              name: cert
              readOnly: true
      nodeSelector:
        kubernetes.io/hostname: 172.24.135.11
      volumes:
        - name: cert
          hostPath:
            path: /etc/etcd
            type: Directory
```



