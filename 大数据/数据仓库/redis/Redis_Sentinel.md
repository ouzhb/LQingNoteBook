# Redis Sentinel

Redis-Sentinel是Redis官方推荐的高可用性(HA)解决方案，当用Redis做Master-slave的高可用方案时，假如master宕机了，Redis本身(包括它的很多客户端)都没有实现自动进行主备切换，而Redis-sentinel本身也是一个独立运行的进程，它能监控多个master-slave集群，发现master宕机后能进行自动切换。

作用：

- 监控：Sentinel会不断检查您的主实例和副本实例是否按预期工作。
- 通知：Sentinel可以通过API通知系统管理员或redis客户端，其中一个受监视的Redis实例出了问题。
- 自动故障转移
- 服务发现：充当Redis集群地址的服务发现工具。


特性：

 - sentinel 本身支持集群部署
 - 由于Redis使用异步复制，因此Sentinel + Redis分布式系统不能保证在故障期间保留已确认的写入。


# 参考

[Redis Sentinel](https://redis.io/topics/sentinel)

[博客介绍](https://www.cnblogs.com/duanxz/p/4701831.html)