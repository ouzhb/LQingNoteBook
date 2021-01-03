# OCI 标准

OCI 标准是2015由Docker发起的容器运行时标准，主要包括以下两个方面的内容：

- image-spec：描述如何分发、压缩/解压缩“文件系统包（filesystem bundle / OCI Image）”
- runtime-spec：描述如何运行在磁盘上解压缩的“文件系统包（filesystem bundle / OCI Image）”



## 1. Runc

Runc 是docker 发布的轻量级跨平台的容器运行时，直接依赖的系统的libcontainer运行容器。

## 2. CRI-O

## 3. Containerd

Containerd 是一个工业级标准的容器运行时，负责以下工作：

- 管理容器的生命周期(从创建容器到销毁容器)
- 拉取/推送容器镜像
- 存储管理(管理镜像及容器数据的存储)
- 调用 runC 运行容器(与 runC 等容器运行时交互)
- 管理容器网络接口及网络

# Docker



docker实际上包括了以下组件：

- docker：客户端工具
- dockerd：docker engine 服务端
- docker-containerd：
- docker-containerd-ctr
- docker-containerd-shim
- docker-init
- docker-proxy
- docker-runc