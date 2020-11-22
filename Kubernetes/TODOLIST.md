# TODOLIST


- Kubernetes 基础概念
    - ~~Pod的安全组设置（delay）~~
        - 权限相关知识深入了解
        - Linux cap
    - Kubernetes 的kube-proxy工作原理
        - ~~iptable模式~~
        - ~~ipvs 模式~~
    - 了解Kubernetes中的网格服务
        - 网格服务的功能、实践
        - 边车的工作原理，Kuberenetes 是如何注入边车的
        - 如何设计灰度发布功能
    - Jenkins和K8S在持续集成中如何配合，K8S应用的持续集成
    - Promethus 监控方案
        - ~~部署，服务发现~~
        - 和其他k8s服务的集成
        - Operator
        - pod的指标采集，如何自动进行动态伸缩
    - kube-status-metric等服务能够采集到那些指标
    - ~~关于pause 容器~~
        - 提供Pod的基础命名空间
        - 作为init进程，回收僵尸进程
    - Kubernetes 详细安装步骤
        - [kubernetes-the-hard-way](https://github.com/kelseyhightower/kubernetes-the-hard-way)
    
- Kubernetes 源码阅读、定制化、工具
    
    - sdpctl工具
        - ~~shell 批量执行~~
        - ~~删除掉一些冗余的功能~~
        - ~~docker接口整合，查看运行的容器的Image信息~~
        - ~~shell 命令行工具能够支持 &&、管道、~~选择node~~等~~
        - ~~定制Kubectl，将功能集成~~
        - falcon、salt-minion 模块
            - 安装、检查
            

- Go语言基础
    - ~~基础编程：《Go语言编程》~~
    - ~~用go语言开发一个基于ssh/autossh 的隧道工具，能够从公网ssh连接内网的服务器（用docker+nsenter替代）~~
    - ~~cobra~~
    - json/yaml 读写解析
    - go 语言相关博客：https://www.cnblogs.com/nickchen121/p/11517502.html
        - go template
            - https://www.cnblogs.com/f-ck-need-u/p/10035768.html

- Kubernetes新特性 
    - 地址kube-schedule，需求如下（参考[iSSUE](https://github.com/kubernetes/kubernetes/issues/1574)）：
        - pod调度不变更Node
        - configmap 重建相关POD
    - **阿里新控制器项目**：https://github.com/openkruise/kruise/tree/master/docs/tutorial
    
- Docker相关
    - blob分层机制了解
    - ~~go-client~~
    - docker push机制是什么样的
    
    
- 网络相关知识，Kube的网络模型  
    - ~~iptable 基础知识~~
    - 关于DNS
        - ~~DNS Server中几种记录的区别， 是怎么工作的？~~
        - ~~如何搭建一个DNS服务（CoreDNS）？~~
        - ~~CoreDNS在k8s中是如何工作的？~~
        - ~~添加自定义DNS记录到K8S中？~~
    - 了解不同的CNI的工作方式 
    - 了解以下几种网络模型的工作原理
        - flannel
        - calico
        - culium
    - Nginx 相关知识学习
    
    
- 相关生态工具
    - 一个分布式的docker管理工具 [【portainer/portainer】](https://github.com/portainer/portainer)
    - Harbor升级遇到的种种问题复盘
    - harbor 2.1.0 新版本功能
        - proxy镜像代理（可以替代镜像同步功能，减少代码开发量）
        - p2p预热和相关工具更加深入的集成
        - 不停机垃圾回收
    - 安全扫描插件triy、clair、notry
    - 镜像分发工具 dragonfly / Uber Kraken

- 阅读
    - Kubernetes扩展编程，参考：https://cloudnative.to/blog/kubernetes-programming-base/
    - Kubernetes扩展API-Server的接口，参考：https://kubernetes.io/docs/tasks/extend-kubernetes/setup-extension-api-server/
    - Kubernetes API接口文档：https://kubernetes.io/docs/reference/kubernetes-api/
    - Kubernetes 高质量博客：https://github.com/chenzongshu/Kubernetes
    


    



