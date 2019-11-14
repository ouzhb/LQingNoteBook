# DNS

## Service

通常情况下Services对应IP的 DNS record 为 my-svc.my-namespace.svc.cluster.local。

对Headless Services而言，my-svc.my-namespace.svc.cluster.local 被解析为一个或者多个IP，用户实际得到的IP是round-robin的。

如果用户在Service中指定了多个port，那么可以通过 _my-port-name._my-port-protocol.my-svc.my-namespace.svc.cluster.local 的方式访问服务，Kubernetes将这个玉米解析为SRV records，表示domain:port

## Pods

Pod 同样可以分配到相应的DNS记录，格式为：pod-ip-address.my-namespace.pod.cluster.local。但是一般不会使用该记录访问Pod，pod间通信依然通过svc。

在 Pod 的Spec中可以指定 Hostname 以及 subdomain。Kubernetes会将在Pod中
my_hostname.my_default-subdomain.default.svc.cluster.local 的映射关系添加到/etc/hosts文件中。

每个Pod可以独立设定DNS解析策略，包括：

|DNS策略|说明|
|----|----|
|Default|完全宿主机继承resolv.conf|
|None|Pod 预先载入任何自身逻辑判断得到的 DNS 配置，用户需要额外配置 dnsConfig 来描述DNS参数|
|ClusterFirst|默认配置，预先把  kube-dns（或 CoreDNS）的信息当作预设参数写入到该 Pod 内的 DNS 配置。如果用户使用HostNetWork=true，ClusterFirst 就会被强制转换成 Default。|
|ClusterFirstWithHostNet|使用 HostNetwork 同时使用 k8s DNS 作为我 Pod 预设 DNS 的配置。|

## Pod’s DNS Config

当用户配置 --feature-gates=CustomPodDNS=true 并且指定dnsPolicy=None时，可以通过 DNS Config 手工指定Pod的DNS解析方式。

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

# Ingress

Ingress 是为进入集群的请求提供路由规则的集合。

Ingress 可以给 service 提供集群外部访问的 URL、负载均衡、SSL、HTTP 路由等功能。

Ingress根据用户的HTTP地址，将用户请求发送到对应Service上。

![NodPort](http://s5.51cto.com/oss/201804/12/06794486d035874ef620b98b069f9d68.jpg)

![Load Balancer](http://s3.51cto.com/oss/201804/12/ea0faab259a6d5a6ec1178e12e6cd295.jpg)

![ingress](http://s5.51cto.com/oss/201804/12/e3a3ac2d7720431dcffc16a4ae3ea497.jpg)

## Ingress controllers

创建Ingress资源，用户需要在Kubernetes中额外运行Ingress controllers。Kubernetes中可以部署HAProxy、NGINX等Ingress controllers。

## Ingress Resource

```yaml
apiVersion: extensions/v1beta1
kind: Ingress
metadata:
  name: simple-fanout-example
  annotations:
    nginx.ingress.kubernetes.io/rewrite-target: /
spec:
  rules:
  - host: foo.bar.com
    http:
      paths:
      - path: /foo
        backend:
          serviceName: service1
          servicePort: 4200
      - path: /bar
        backend:
          serviceName: service2
          servicePort: 8080
```