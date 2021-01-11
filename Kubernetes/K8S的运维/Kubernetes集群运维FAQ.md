# 问题列表

## 1.cni0 IP冲突

新加入节点无法创建容器：

```
network: failed to set bridge addr: "cni0" already has an IP address different from xxxxx
```

解决方案：
- 驱逐有问题的节点删除，并删除
- 执行以下命令，删除cni0设备：

```shell
# PS:只在worker上执行这个操作

mv /etc/kubernetes /etc/kubernetes_bak
systemctl stop kubelet
systemctl stop docker
kubeadm reset
rm -rf /var/lib/cni/
rm -rf /var/lib/kubelet/*
rm -rf /etc/cni/
ifconfig cni0 down
ifconfig flannel.1 down
ifconfig docker0 down
ip link delete cni0
ip link delete flannel.1
systemctl start docker
```
- 重新将将worker加入kubernetes集群

## 2. Harbor 相关问题

- 1.9.0 版本多实例时有同步bug，可以升级到1.9.2版本进行规避

## 3. 容器内操作

- 查看宿主机kubelet日志：

```
cd /hostrun/log/journal
# 检查kubelet对应的id，grep -R "kubelet" *
 journalctl --directory=/hostrun/log/journal/ -u kubelet

```

## 4.强制删除

```shell script
kubectl delete pod <pod-name> --grace-period=0 --force -n kube-system
```

## 5. ETCD异常日志

ETCD 容器一直打打印以下日志：

```
embed: rejected connection from "xx.xx.xx.xx:xxxx" (error "EOF", ServerName "")
```

上述日志有以下特征：

- 相同的IP地址每隔60s输出一条日志，但是每条日志的端口不一样
- 绝大多数日志的源IP是K8S集群的宿主机IP，
- 有一个源IP是容器IP，并且这个源IP和当前的ETCD容器运行在同一个节点上

问题原因：

- ETCD的日志表明：K8S集群中有一些服务发起了异常的ETCD连接请求，这些连接请求被Server端拒绝（进一步定位问题，需要查看对端的日志）。
- 由于ETCD日志中包含几乎集群中所有Node的宿主机IP，因此首要怀疑是DS对象， 以及Kubelet等服务
- 对应发现sdp-monitor中的py脚本会每隔60s对连接一次etcd的2379，连接代码如下：

```python
def socket_test(host, port):
	port_sla = 0
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		result = s.connect_ex((host, port))
		#connect success
		if result == 0:
			port_sla = 1
		s.close()
	except:
		logging.error( "failed to connect to host=%s, port=%s, %s" %(host, port, str(traceback.format_exc())))
	
	logging.info("%s:%d sla=%d" % (host, port, port_sla))
	return port_sla
```

- 虽然sdp-monitor使用的是容器网络，但是由于整个集群用vxlan组网。因此服务端实际看到的是宿主机ip，只有当sdp-monitor和etcd位于相同节点时，看到的才是容器ip。

## 6. nginx-ingress 证书异常

安全检查工具扫描Nginx的443端口，发现主机使用自签名证书（实际上所有的Ingress都已经使用了*.101.com）。

问题原因：安全工具扫描时直接走的IP没有走域名，ingress针对IP访问是直接使用内置的SSL证书返回，使用curl命令可以验证：

```shell script
 curl -vv https://172.24.135.12
############################################################################
# Server certificate:
# 	subject: CN=Kubernetes Ingress Controller Fake Certificate,O=Acme Co
############################################################################
```

解决方案：

- controller 添加默认证书：--default-ssl-certificate=<ns-name>/<secret-name> PS: ns-name必须指定
- 重启所有controller

参考 nginx-ingress TLS相关配置：https://kubernetes.github.io/ingress-nginx/user-guide/tls/

## 7. iptables 问题造成的 fannel 网络异常

Flannel/Calico 的 VXLAN Overlay 网络使用的是 8472 端口，并非Linux内核vxlan机制使用4789端口。

```shell script
ip -d link show flannel.1
iptables -I INPUT -p udp -s $IDC_INNER_SEGMET --dport 8472 -j ACCEPT
```

## 8. 限制容器Core文件

docker 添加以下配置
```yaml
  "default-ulimits": {
   "core": {
      "Name": "core",
      "Hard": 0,
      "Soft": 0
    }
  }
```
