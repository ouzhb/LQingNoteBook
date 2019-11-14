# FlexVolume

 FlexVolume是Kubernetes提供给存储供应商的标准卷接口，用户能够编写自己的驱动程序并在Kubernetes中添加对卷的支持。

 当Kubelet配置了enable-controller-attach-detach选项，则Kubernetes的各个节点自动安装指定路径（通过--volume-plugin-dir配置，默认为/usr/libexec/kubernetes/kubelet-plugins/volume/exec/）下的卷驱动程序。

  v1.8以后的版本，Kubernetes的FlexVolume支持动态发现、升级、滚动等操作。

  [参考](https://github.com/kubernetes/community/blob/master/contributors/devel/flexvolume.md)