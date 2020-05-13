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

### Master节点

Master 节点用于管理 Cluster 的状态，这些状态包括：

- 全局集群配置
- 发现集群的节点和他们的状态
- table的schema
- shard的主从位置和状态

每一个节点都会保存上述状态，并且由master节点来负责更新。

### 存储

Cratedb中所有表都是分片表，每个分片是一个Lucene索引，并分解成不同的Segments存储在物理文件中。

每个Shard可以配置任意个数的Replica。每个Replica 实时同步主shard的全量数据。

- 用户读操作可以从：Master Shard 或者 Replica Shard 读取数据，用户请求会被随机路由到其中之一；
- 用户写操作需要经过以下流程：
	- Master 确认主分片和所有副本分片的状态，只有分片数目达到法定的数量，才能继续写入；
	- 在主分片写入
	- 在副本分片写入



CrateDB支持行级别的原子操作，每行记录实际上以 document 的方式存储，并且每个 记录都包含一个隐藏的 _id 列，这个列确定了记录的分片位置。

CrateDB不提供事务能力~！

### 一致性

CrateDB 提供最终一致性，当Master Shard流失可能出现并发的dirty read。

# 性能

官方的性能调优配置：https://crate.io/docs/crate/guide/en/latest/performance/index.html


# 总结


- 调研 IOT 场景的开源数据库 cratedb 

	- cratedb 支持分布部署和动态扩容，官方宣称能够支持大并发场景的实时更新
	- cratedb 支持SQL，并且提供了JDBC，使用过程中有以下感受：
		- cratedb的SQL语法和标准SQL相比区别巨大，并且其数据结构更接近ES、Mongodb等NoSQL存储
		- JDBC 十分鸡肋，完全没有ORM工具也没有和Spring Data等框架整合的方案，要直接调用jdbc的接口进行编码，效率非常低下
		- 支持http接口存/取数据（这个是主要的读写方式）。http 接口是直接 post SQL语句到CrateDB，并且在返回值的body中以json方式来获取封装数据。
		- 国内几乎没有人使用，查找资料只能去官网（要翻墙，并且官方文档不是太详细）
		
	总结：接口不是太友好，开发效率不高，并且可以借鉴的资料几乎没有！

 

# 参考

[【架构】](https://crate.io/docs/crate/guide/en/latest/architecture/index.html)：包括Join、Cluster、存储等核心功能的实现。

[【在容器中运行】](https://crate.io/docs/crate/guide/en/latest/deployment/containers/docker.html#docker-compose)
