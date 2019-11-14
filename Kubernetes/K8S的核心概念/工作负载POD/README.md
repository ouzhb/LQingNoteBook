# OverView

Pod中的容器共享网络空间（IP和端口号），并且可以使用 SystemV 信号量和 POSIX 共享内存进行通信。

共享卷被定义成Pod的一部分，他的生命周期独立于Container。

唯一标识POD的是UID，以下场景Pod的UID会发生变化：
    - 调度失败
    - node失败
    - 由于资源不足等场景，Pod被Kubernetes驱逐
# Pod和Controller

Pod本身不具备自我管理功能。Kubernetes使用更高级别的抽象，称为Controller，它处理管理Pod实例的工作。

# Delete Pod

## Gracefully terminate

当用户删除Pod时，Kubernetes立即向用户的主进程发送TERM信号，并等待“grace period（30S）”发送Kill信号。之后，pod被API Server移除。
默认时，grace-period为30s，通过--grace-period=seconds指定。
```
 --> Client Request Delete 

 --> API Server 内部更新POD为Dead

 --> Pod显示Terminating状态

 --> Kubelet调用 preStop hook，grace period时间内preStop未停止，发送TERM并等待2（如果没有配置preStop，直接发送TERM等待grace period时间）。同时，Kubelet开始移除endpoint、Service等Pod的对外访问接口

 --> 超过grace period，发送SIGKILL

 --> Pod不再可见
```
## Force terminate

强制删除时，Pod立即被从Kubernetes上移除，新pod被立即创建。
但是在Node上看，POD任然有一定宽限期直到其退出。

配置--force 和 --grace-period=0 可以强制删除Pod。

# Privileged Pod

配置 spec.containers[0].securityContext.privileged 可以让容器内的进程获得与容器外部进程可用的几乎相同的权限。

# 容器Hook


## PostStart

无法保证PostStart比ENTRYPOINT先执行，并且无法传递参数。

## PreStop

删除容器时立即执行，PreStop是阻塞的，同样无法传递参数。

# Init Container

一个Pod中可以有一个或者多个Init Container，这些Container优先于其他容器运行。
Init Container遵循以下特性：

- 状态总是运行到completion
- 多个Init Container顺序运行
- 如果某个Init Container失败，那么Pod退出
- 所有InitContainer成功运行完成后，启动其他常规Container
- 更改InitContainer的image field相当于重启Pod
- 资源限制取以下两个的高者：
    - Init Container中最高的request/limit值
    -  app Containers中 request/limit值之和
- 如果App Container的image 内容发生改变，那么只会重启App Container
[用法参考](https://kubernetes.io/docs/concepts/workloads/pods/init-containers/#init-containers-in-use)

# Pod Preset

pod运行前可以通过preset信息注入到Pod中。

使用Preset需要配置，需要修改以下配置：

编辑/etc/systemd/system/kube-apiserver.service 添加--admission-control=PodPreset，以及--runtime-config=settings.k8s.io/v1alpha1=true

注意以下几点：
- Preset似乎一旦生成无法修改
- Preset 只能在Pod生成时注入信息。
- 当在Deploy中使用preset时，删除POD时新POD会重新加载Preset的内容

# Disruptions

Pod中断包括：自愿中断和非自愿中断。使用PodDisruptionBudget对象可以控制Pod自愿中断时，一次删除Pod的数目。

假设pod配置spec.replicas=5，当用户使用kubectl drain命令（让主机进入维护模式）时，Kubernetes可能一次关闭5个Pod，并在另一个node一次启动5个pod。

假设配置PDB保证最小Pod数目为4，那么kubectl drain命令一次只会关闭一个Pod，线上提供服务的Pod始终不小于4个。


# Pod的设计模式

[Pod设计模式](https://kubernetes.io/blog/2015/06/the-distributed-system-toolkit-patterns/)的核心思想：常用的功能封装到单独容器中，对外提供一致接口，[参考](https://www.usenix.org/conference/hotcloud16/workshop-program/presentation/burns)

- 单容器&单pod
- Sidecar containers：在pod中创建一个Sidecar，该容器主要用来同步Container之间的共享配置文件。
    ![](https://3.bp.blogspot.com/-IVsNKDqS0jE/WRnPX21pxEI/AAAAAAAABJg/lAj3NIFwhPwvJYrmCdVbq1bqNq3E4AkhwCLcB/s1600/Example%2B%25231-%2BSidecar%2Bcontainers%2B.png)
- Ambassador containers：在pod中创建一个Ambassador，该容器是一个代理。pod中的主容器可以通过localhost的方式连接其他Pod的服务。这种设计可以将业务设计和网络设计分离。
    ![](https://4.bp.blogspot.com/-yEmqGZ86mNQ/WRnPYG1m3jI/AAAAAAAABJo/94DlN54LA-oTsORjEBHfHS_UQTIbNPvcgCEw/s1600/Example%2B%25232-%2BAmbassador%2Bcontainers.png)
- Adapter containers: 适配器模式是指在Pod内部启动一个适配器容器，适配器容器为主容器向外提供统一的服务接口。如下图,适配器为Mon Sys提供了统一的数据接口。这样的设计将功能边界很好的隔离开。
    ![](https://4.bp.blogspot.com/-4rfSCMwvSwo/WRnPYLLQZqI/AAAAAAAABJk/c29uQgM2lSMHaUL013scJo_z4O8w38mJgCEw/s1600/Example%2B%25233-%2BAdapter%2Bcontainers%2B.png)
