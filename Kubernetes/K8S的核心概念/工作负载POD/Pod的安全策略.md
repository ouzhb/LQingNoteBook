# Pod安全策略

Pod安全策略是**集群级别**的用于控制 pod 安全相关选项的一种资源，对应的资源是**PodSecurityPolicy**。

通过Pod的安全策略可以限制以下内容（[【参考】](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.18/#podsecuritypolicyspec-v1beta1-policy)）：

| **Control Aspect** | **Field Names**                                              | 说明                                                         |
| ------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 容器Root权限       | privileged                                                   | 1.为true时，容器内的root用户在容器外有真正的root权限，否则该用户只是一个普通用户。 |
| 虚拟化ns相关       | hostPID<br />hostIPC                                         | 1. 使用数组机的PID和IPC空间                                  |
| 网络相关           | hostNetwork<br />hostPorts                                   | 使用宿主机网络；<br />hostPort通过iptable的手段连通容器和调度node的一个端口； |
| 存储相关           | volumes<br />allowedHostPaths<br />allowedFlexVolumes        | 1. 允许使用volume的类型<br />2.允许使用的hostPath路径列表<br />3.flex存储卷的白名单 |
| 只读root文件系统   | readOnlyRootFilesystem                                       | 将除了volume以外的文件全部设定为只读                         |
| 容器的用户id和组id | runAsUser<br />runAsGroup<br /><br />fsGroup<br />supplementalGroups | 1. 指定容器的 user ID 和 primary group ID。默认情况下，用户使用root(0)<br />2. fsGroup 用拿来指定volume所属的Group ID，并且将fsGroup指定的ID作为容器进程补充Group<br />3. supplementalGroups和fsGroup同样用于存储，但是主要用于共享存储 |
| 禁止提升到root权限 | allowPrivilegeEscalation<br />defaultAllowPrivilegeEscalation | 决定容器进程能否进行权限提升，该配置直接影响容器的 [NO_NEW_PRIVS](https://www.kernel.org/doc/Documentation/prctl/no_new_privs.txt) |
| Linux Capabilities | defaultAddCapabilities<br />requiredDropCapabilities<br />allowedCapabilities | 基于Linux Capabilities功能来进行权限控制。                   |
| SELinux上下文      | seLinux                                                      |                                                              |

# 使用PodSecurityPolicy

使用`PodSecurityPolicy`时，需要在 KUBE-APISERVER 中设置 `--enable-adminssion-plugins=PodSecurityPolicy`。

默认情况下，K8S不会启动PodSecurityPolicy，可以通过在enable-adminssion-plugins中附加PodSecurityPolicy开启这个功能。

PodSecurityPolicy产生作用的路径：

- 创建PodSecurityPolicy资源

- 将PodSecurityPolicy和serviceaccount绑定

- 在Deploy或者其他控制器中指定serviceaccount

- 创建准入的Pod

  





# 设置Security Context

用户可以在容器级别，包含 `securityContext` 字段， 前者覆盖后者。

`securityContext`字段可以指定以下内容：[【参考SecurityContext 】](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.18/)



# 设置PodSecurityContext 

`PodSecurityContext` 类似`SecurityContext`，但是在Pod级别控制所有容器。

`PodSecurityContext`包含`SecurityContext`大部分选项（更少一些），但是多出一个`sysctls`，通过这个配置可以控制启动容器的系统参数。



# 参考

[Pod 安全策略](https://k8smeetup.github.io/docs/concepts/policy/pod-security-policy/)

[Kubernetes API](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.18/)