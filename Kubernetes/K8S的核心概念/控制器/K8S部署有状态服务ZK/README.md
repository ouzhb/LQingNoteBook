# 设计思路

设计者在镜像中，定制了一个启动脚本该脚本收集Kubernetes传入的信息，自动生成配置文件，并启动ZK进程。

每个POD需要知道以下信息，才能生成配置文件：

- ZK集群共用几个节点：该配置通过启动脚本的--servers参数传入，并且该配置和.spec.replicas保持一致。当用户进行scala操作时只要修改这两个参数会自动触发更新；
- 当前POD对应ZK节点myid：每个POD的主机名是独立的，格式为$(statefulsets-name)-$(index)。因此可以从hostname推测出当前Pod的myid。
- 其他POD对应网络地址：格式为$(statefulset name)-$(ordinal).$(serviceName).$(ns).$(svc.cluster.local)
  
Kubernetes提供的其他保证：
- 当用户进行scale操作时，永远先销毁myid大的pod
- 通过PDB可以保证线上总用至少一个ZK pod在运行，任意scala、Node下线等操作不会中断ZK服务
- 通过PodAntiAffinity功能可以保证，任何集群上只运行一个zk对应的POD
- 整个ZK集群对外只暴露2181一个端口



# 验证各个节点的IP信息
```shell
# 查看所有的主机名
for i in 0 1 2; do kubectl exec zk-$i -- hostname; done

# ZK的myid
for i in 0 1 2; do echo "myid zk-$i";kubectl exec zk-$i -- cat /var/lib/zookeeper/data/myid; done

# FQDN信息
for i in 0 1 2; do kubectl exec zk-$i -- hostname -f; done

# ZK的配置文件
kubectl exec zk-0 -- cat /opt/zookeeper/conf/zoo.cfg
```


# 存在问题

- 没有共享存储时，scale存在隐患，原因是：ZK-x不会运行在同一个节点，这样会导致ZK-x的访问持久化数据出现不一致。