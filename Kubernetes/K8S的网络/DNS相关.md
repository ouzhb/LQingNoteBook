# Kubernetes 网络知识笔记

## 1. Service的DNS域名

- 普通服务 clusterIP or NodePortNodePort：**<SVC>.<namespace>.svc.cluster.local**。

- Headless Services：**<SVC>.<namespace>.svc.cluster.local** 表示多个IP，解析的情况可能是每次不同的。pod的确定IP通过**<StatefulSet-Name>-<index>.<SVC>.<namespace>.svc.cluster.local,

- SVC包含多个端口：通过 **<_my-port-name>.<_my-port-protocol>.<SVC>.<namespace>.svc.cluster.local** 的方式访问服务，Kubernetes将这个域名解析为SRV records，表示domain:port

## 2. 关于DNS服务

CoreDNS通过Deployment的方式部署，对应的SVC **kube-dns**，默认情况的这个服务的IP非常重要。



每个Pod可以独立设定DNS解析策略，包括：

|DNS策略|说明|
|----|----|
|Default|完全宿主机继承resolv.conf|
|None|用户需要额外配置 dnsConfig 来描述DNS参数|
|ClusterFirst|**默认配置**，kube-dns作为DNS服务的地址。如果用户使用HostNetWork=true，ClusterFirst 就会被强制转换成 Default。|
|ClusterFirstWithHostNet|使用 HostNetwork 同时使用ClusterFirst。|

### Pod’s DNS Config
通过 DNS Config 手工指定Pod的DNS解析方式。

```yaml
apiVersion: v1
kind: Pod
metadata:
  namespace: default
  name: dns-example
spec:
  containers:
    - name: test
      image: nginx
  dnsPolicy: "None"
  dnsConfig:
    nameservers:
      - 1.2.3.4
    searches:
      - ns1.svc.cluster.local
      - my.dns.search.suffix
    options:
      - name: ndots
        value: "2"
      - name: edns0
```

### DNS解析规则

默认情况，resolv.conf的配置如下,参考这个[blog](https://ieevee.com/tech/2019/06/22/ndots.html)理解配置的含义。

```shell

nameserver 10.96.0.10
search <namespace>.svc.cluster.local svc.cluster.local cluster.local
options ndots:5

```
关于DNS解析失败时如何排除故障：[参考](https://kubernetes.io/docs/tasks/administer-cluster/dns-debugging-resolution/)


## 3. Pod的DNS

Pod 同样可以分配到相应的DNS记录，格式为：pod-ip-address.my-namespace.pod.cluster.local。但是一般不会使用该记录访问Pod，pod间通信依然通过svc。

在 Pod 的Spec中可以指定 Hostname 以及 subdomain。Kubernetes会将在Pod中
my_hostname.my_default-subdomain.default.svc.cluster.local 的映射关系添加到/etc/hosts文件中。