# TODOLIST

- ~~Pod的安全组设置（delay）~~
- ~~iptable 基础知识~~
- Kubernetes 的kube-proxy工作原理
    - ~~iptable模式~~
    - ~~ipvs 模式~~
- 了解Kubernetes中的网格服务
  
    - 网格服务的功能、实践
    - 边车的工作原理，Kuberenetes 是如何注入边车的
    - 如何设计灰度发布功能
- Jenkins和K8S在持续集成中如何配合，K8S应用的持续集成
- 日志收集聚合

    - ~~安装部署loki~~ 
    - 安装部署ELK
- pod的指标采集，如何自动进行动态伸缩
- Promethus 监控方案
- kube-status-metric等服务能够采集到那些指标
- 关于DNS
    - DNS Server中几种记录的区别， 是怎么工作的？
    - ~~如何搭建一个DNS服务（CoreDNS）？~~
    - CoreDNS在k8s中是如何工作的？
    - ~~添加自定义DNS记录到K8S中？~~
- 关于pause 容器

    - 提供Pod的基础命名空间
    - 作为init进程，回收僵尸进程
- 了解不同的CNI的工作方式 
- 了解以下几种网络模型的工作原理
  
    - flannel
    - calico
    - culium
- Kubernetes 详细安装步骤

    - [kubernetes-the-hard-way](https://github.com/kelseyhightower/kubernetes-the-hard-way)
- 用go语言开发一个基于ssh/autossh 的隧道工具，能够从公网ssh连接内网的服务器
- 一个分布式的docker管理工具 [【portainer/portainer】](https://github.com/portainer/portainer)

- Kubernetes扩展编程，参考：https://cloudnative.to/blog/kubernetes-programming-base/
- Kubernetes扩展API-Server的接口，参考：https://kubernetes.io/docs/tasks/extend-kubernetes/setup-extension-api-server/
- Kubernetes API接口文档：https://kubernetes.io/docs/reference/kubernetes-api/