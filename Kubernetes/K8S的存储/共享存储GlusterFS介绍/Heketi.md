# 说明

glusterfs接入Kubernetes时可以通过volumes的方式和PV的方式。

以volumes方式接入时，用户需要事先在glusterfs服务器上创建卷并启动，volume只能选择已经启动的卷挂载。
以PV方式接入时，glusterfs需要和管理工具Heketi对接。当PVC请求存储时，Kubernetes通过对应StorageClass请求Heketi动态创建PV。

# 安装部署

完整的Kubernetes对接Gluster方案参考：

[gluster-kubernetes](https://github.com/gluster/gluster-kubernetes)

[heketi](https://github.com/heketi/heketi)

[deploy](https://github.com/gluster/gluster-kubernetes/blob/master/docs/setup-guide.md)