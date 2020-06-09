# Kube-Keepalived-Vip

Kube-Keepalived-Vip 工具可以在Kubenetes集群中定义VIP，为用户的NodePort服务提供VIP访问地址。

参考[aledbf/kube-keepalived-vip](https://github.com/aledbf/kube-keepalived-vip)，通过DaemonSet在每个服务启动Keepalived服务，用户通过修改预先定义的ConfigMap添加或者删除VIP。

使用中发现几个问题：

- 生成的宿主机IP在主机层面可见，并且可以当作普通VIP使用
- 可以动态配置多个VIP
- 定义VIP时需要绑定Service，不太清楚当Service对应的Pod漂移时，VIP是否会跟着漂！！（**~~应该要实现这样的功能，否则还不如在集群之外部署Keepalived~~，如果是NodePort映射似乎VIP不用跟着漂～～～！！！**）


# 参考

[keepalived-vip](https://www.bookstack.cn/read/feiskyer-kubernetes-handbook/plugins-keepalived-vip.md)