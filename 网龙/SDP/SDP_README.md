# 概述

SDP 本质上是一个基于混合云的PAAS平台，平台架构如下：

![image-20200829204529483](..\..\images\ND\SDP架构.png)

功能涵盖**研发管理、开发、测试、发布、运维**整个软件周期的方方面面。

其中项目管理涉及，以下模块：WIKI、DOC、RMS、PMS、BBS、Console

开发运维工具涉及以下：NPM、GIT、Jenkins、Elastic Web、Kubernetes、ELK、Falcon、Monitor、Nexus

SDP涉及的软件基础服务，如下图：

![image-20200829205725919](..\..\images\ND\SDP软件基础服务.png)

## K8S

### 1. 弹性伸缩方案







# SDP服务Kubernetes部分架构

- kube-system
	- coreDNS
	- ipvs
	- flannel
- java-init-agent: registry.101.com/sdp/java-init-agent:1.8 ???
- log-download: registry.101.com/sdp/nginx
- main-domain：用来引入外部服务
- nginx-ingress/nginx-ws-ingress
- sdp-monitor：registry.101.com/sdp/centos7.6:v2 




# 参考

[【Wiki】]([http://wiki.doc.101.com/index.php?title=%E5%85%B1%E4%BA%AB%E5%B9%B3%E5%8F%B0](http://wiki.doc.101.com/index.php?title=共享平台))

[【SDP登录】](http://d.101.com/)