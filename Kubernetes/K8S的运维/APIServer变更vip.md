# 1. Keepalived

通过Docker部署Keepalived提供VIP

- 所有Master节点部署Keepalived镜像

- Keepalived 配置参数如下

存在问题：

- VIP没有监控6443端口可用性，可能出现端口异常但是VIP没有漂移的场景
- 使用LVS进行负载均衡
  - 使用ipvs转发对网络有一定要求，不是所有场景都适用

PS：解决方案，virtual_server的端口不配置6443，随便配置一个别的端口。RS监听本机6443端口，当服务异常时kill掉keepalived使ip切换。LVS此时应该配置为NAT方式

容器化Keepalived地址：

```shell
# 仓库地址 https://github.com/LinQing2017/docker-keepalived
docker run -d --privileged --cap-add=NET_ADMIN --restart=always --net=host \
--name="keepalived-kube-api-internel" \
-e KEEPALIVED_INTERFACE="em2" \
-e KEEPALIVED_PASSWORD="linqing" \
-e KEEPALIVED_VIRTUAL_IPS="10.33.17.243" \
-e KEEPALIVED_UNICAST_PEERS="#PYTHON2BASH:[]" \
-e KEEPALIVED_ROUTER_ID="132" \
wxext-registry.101.com/sdp/keepalived:2.0.20-origin
```



# 2. 证书添加VIP别名

修改kubelet集群配置

```shell
# 修改kubeadm配置，添加 apiServer.certSANs 参数
kubectl -n kube-system edit cm kubeadm-config 
```
重新生成 apiserver 证书，并重启APIServer服务

```shell
cp -r /etc/kubernetes/ /etc/kubernetes_`date +%Y%m%d%H%M%S`
kubeadm config view > /root/cluster.yaml
rm -rf /etc/kubernetes/pki/apiserver.crt /etc/kubernetes/pki/apiserver.key 
kubeadm init phase certs   apiserver --config /root/cluster.yaml 

docker ps | grep kube-apiserver | grep -v pause
docker kill {api-id}
```

修改所有节点中APIServer地址，并重启服务

- controller-manager.conf
- scheduler.conf 
- kubelet.conf
- API-Server.yaml
- kube-proxy 服务
- 修改kubectl -n kube-public edit cm cluster-info 
- SDP相关服务

PS：对于kubelet.conf可以删除旧文件，并创建 bootstrap-kubelet.conf 重新引导

```yaml
apiVersion: v1
clusters:
- cluster:
    certificate-authority-data: {...}
    server: {...}
  name: kubernetes
contexts:
- context:
    cluster: kubernetes
    user: tls-bootstrap-token-user
  name: tls-bootstrap-token-user@kubernetes
current-context: tls-bootstrap-token-user@kubernetes
kind: Config
preferences: {}
users:
- name: tls-bootstrap-token-user
  user:
    token: {...}
```



