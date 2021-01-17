# 私有环境LoadBalancer

## 1. Metallb

```shell script
helm3 repo add bitnami https://charts.bitnami.com/bitnami
helm3 pull bitnami/metallb
```

### 1. Layer2 模式

基于ARP/NDP协议播报vip信息（ARP负责IPv4地址，NDP负责IPv6地址）

## ARP协议相关

ARP协议是地址解析协议的一种，

ARP协议的特征：
- ARP协议是地址解析协议属于TCP/IP协议簇，当网络层的IP包进入链路层时，ARP协议负责添加头部信息。
- 在TCP/IP模型中，ARP协议属于IP层
- 在OSI模型中，ARP协议属于链路层
- 工作流程：
    - 广播目标IP地址的ARP请求
    - 接受返回信息，并且添加IP包地址的头部
    - 缓存目标IP和MAC的对应关系，以便后续直接读取缓存
- ARP映射方式
    - 静态映射：手工维护ARP表
    - 动态映射：通过ARP协议（RARP协议）来动态维护ARP表
    
# 测试

```shell script
apiVersion: v1
kind: Service
metadata:
  labels:
    app: demo-app
  name: demo-lb
spec:
  type: LoadBalancer
  ports:
  - port: 80
    targetPort: 8080
  selector:
    app: demo-app
```

# 参考文档

[Porter](https://porterlb.io/)

[metallb](https://metallb.universe.tf/)