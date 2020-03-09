# 架构

## Join设计

支持以下类型的Join：

- Inner Join：内连接，此外还有一个特例称为Equi Join（个人理解是改变写法的内连接）
- Outer Join
	- Left Join
	- Right Join
	- Full Join
- Cross Join：直接返回笛卡尔积

上述连接基于 Nested Loop Join 算法实现（Equi Join基于 Hash Join 算法）。
需要注意，在分布式环境下，除了一些特殊场景以外，大部分Join涉及到分片重分布的操作，因此会产生严重的网络传输和读写。

## Cluster方案

集群中所有节点对等，属于 shared-nothing 的架构，每个节点包括一下组成：

- SQL Handler
	- 接受sql请求：当前支持 PostgreSQL协议、HTTP、TCP
	- SQL解析、创建执行计划
- Job Execution Service
- Cluster State Service
	- Cluster状态维护
	- Master节点选举
	- 节点发现
- Data Storage：每个shard是基于Lucene index存储

# 参考

[【架构】](https://crate.io/docs/crate/guide/en/latest/architecture/index.html)：包括Join、Cluster、存储等核心功能的实现。

[【在容器中运行】](https://crate.io/docs/crate/guide/en/latest/deployment/containers/docker.html#docker-compose)
