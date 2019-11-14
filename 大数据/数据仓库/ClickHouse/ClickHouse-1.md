---
title: 数据库调研笔记 -- ClickHouse
categories: "笔记"
date: 2019-06-10
comments: true
toc: true
tags:
	- 数据库
	- ClickHouse
---


数据库调研笔记 -- ClickHouse


<!--more-->

# 特性

ClickHouse最大的优势是：

- OLAP
- *列式*数据

同时支持以下功能：

- 支持SQL查询
- 支持Shard分片、Replica容错
- 支持数据进行向量计算
- 支持在表中定义组件、以及创建索引，能够实现亚秒级查询
- 支持近似计算，即通过牺牲精度的方式，挺高查询性能
- 支持命令行、HTTP、JDBC、ODBC、以及其他第三方客户端
- 支持多种语言的SDK
- 支持多种UI工具，如Tabix、HouseOps、LightHouse、DBeaver
- 支持专门的负载代理工具，如ClickHouse-Bulk、KittenHouse、chproxy 

不支持以下功能：

- 不支持二级索引
- 没有完整的事务支持
- 只能批量删除或者修改，高频率修改单条数据的能力很差
- 不适合通过主键索引单行数据，原因是ClickHouse的索引是稀疏索引
- ClickHouse的join逻辑和标准SQL有很大区别，因此如果迁移到ClickHouse中所有含有Join的SQL要重写（改为使用using关键字，或者subquery来实现）

## OLAP场景

Click House是专门为OLAP场景设计，以下是 ClickHouse 官方对 OLAP 场景特征的总结：

- 大多数是读请求
- 数据总是大批量写入
- 不修改已有数据
- 每次查询都从数据库中读取大量的行，但是同时又仅需要少量的列
- 宽表，即每个表包含着大量的列
- 较少的查询(通常每台服务器每秒数百个查询或更少)，即对DB的并发能力要求不高
- 对于简单查询，允许延迟大约50毫秒
- 列中的数据相对较小： 数字和短字符串(例如，每个URL 60个字节)
- 处理单个查询时需要高吞吐量（每个服务器每秒高达数十亿行）
- 事务不是必须的
- 对数据一致性要求低
- 每一个查询除了一个大表外都很小
- 查询结果明显小于源数据，换句话说，数据被过滤或聚合后能够被盛放在单台服务器的内存中

对大部分应用来说，实际情况可能不一定满足上面所有特点，并非是纯OLAP应用。

## 列式存储

列式存储在OLAP场景中有以下优势：

- 只读取必要的数据、并且便于压缩，极大的减少IO消耗
- 按列查询能够极大的提高CPU的利用率


# 性能

## 官方介绍

按照官方介绍，ClickHouse 相比同类产品傲视群雄，并且公布了一份[测试报告](https://clickhouse.yandex/benchmark.html)。报告中ClickHouse的性能是Greenplum的8倍，Vertica的3倍。

官方ClickHouse的性能，有以下指标描述（[参考](https://clickhouse.yandex/docs/zh/introduction/performance/)）:

- 吞吐量
  - 缓存数据：2~10GB/s，对于简单的查询，速度可以达到30GB／s
  - 非缓存数据：取决于数据的压缩比，以及磁盘IO
- 延时时间
  - 缓存数据：小于50ms
  - 非缓存数据：在使用HDD时，时延为：查找时间（10 ms） * 查询的列的数量 * 查询的数据块的数量
- 并能力
  - 建议每秒最多查询100次
- 写入性能
  - 建议每次写入不少于1000行
  - 每秒不超过一个写入请求
  - 单线程时，写入速度大约在50~200MB/s

## 其他测试

从网络上收集到的资料，基本上可以得到下面的结论：

1. 对于单表操作在不涉及Join的情况下，ClickHouse比其他的组件有非常大优势
2. ClickHouse同时也可以作为一个非常有竞争力的时序数据库来使用
3. 多表Join操作性能，对ClickHouse来说并不理想，之比SparkSQL稍好一点，但是Impala、Presto之类相比依然有非常大差距

|案例|比较对象|数据集|测试结果|
|---|---|---|---|
|[知乎上一篇blog](https://zhuanlan.zhihu.com/p/54907288)|Spark SQL</br>Clickhouse</br>Presto</br>HAWQ</br>GreenPlum|基于TPC-DS 10GB数据集，包括：</br>多表关联测试</br>单表查询测试|在多表关联查询（基于TPC-DS数据集）中，Impala性能最佳，ClickHouse 表现只比SparkSQL稍好。</br> 单表查询查询中，ClickHouse的性能非常优秀，基本比第二名 presto/impala 快3倍，比Spark SQL 快4倍！！</br>[参考](https://github.com/analysys/public-docs)|
|[博客](https://www.percona.com/blog/2017/02/13/clickhouse-new-opensource-columnar-database/)|SparkSQL|使用的测试集是Wiki Pagecount，数据规模是1.2TB。</br>这个测试主要比较单表查询性能，包括进行：聚合、Group by等操作|Clickhouse的性能差不多是SparkSQL的10倍，同时更加节省磁盘和内存|
|[博客](https://www.percona.com/blog/2017/03/17/column-store-database-benchmarks-mariadb-columnstore-vs-clickhouse-vs-apache-spark/)|MariaDB ColumnStore </br>Apache Spark|和上一个block类似，但是加入了MariaDB ColumnStore|测试中没有索引的情况下，MariaDB ColumnStore的性能是最差的，但是这个方案成功的在单节点中搞定了260亿规模的数据集。创建索引之后MariaDB KO了Spark，但是依然被ClickHouse吊打|
|[Altinity](https://www.altinity.com/benchmarks)|InfluxDB</br>|[TSBS](https://github.com/timescale/tsbs)|ClickHouse有最好的插入性能，是第二名InfluxDB的2~3倍</br> Influx 最省磁盘</br>毫秒级跨度的查询Influx性能最好</br>大跨度的查询ClickHouse性能最优|


## 基准测试

ClickHouse 官方文档中给出了不少的示例数据集可以用来进行基准测试，但是比较遗憾的是没有说明可以使用TPC-DS的测试工具。

个人猜测主要因为以下原因：

- ClickHouse并不擅长多表关联操作，官方只好藏拙
- TPC-DS的Case中大量包含join语法，完全应用到Clickhouse需要改造

官方BenchMark:
- 单表测试：[维基访问数据](https://clickhouse.yandex/docs/zh/getting_started/example_datasets/wikistat/)
- [星型图测试](https://clickhouse.yandex/docs/zh/getting_started/example_datasets/star_schema/)

# 表引擎

表引擎指的是Clickhouse中表的不同类型，决定了：

- 表的分片、副本情况；
- 如何支持并发访问，是否能够进行多线程请求；
- 如何进行索引；
- 数据复制参数；

ClickHouse中最主要的表引擎是MergeTree族下的表引擎。

## MergeTree表

这种表引擎的原理实际上和HBase的原理类似，基于合并树将以主键排序的数据顺序写入到后台文件中，并在必要的时候对文件进行合并。

创建MergeTree表的命令如下：

```SQL

CREATE TABLE [IF NOT EXISTS] [db.]table_name [ON CLUSTER cluster]
(
    name1 [type1] [DEFAULT|MATERIALIZED|ALIAS expr1],
    name2 [type2] [DEFAULT|MATERIALIZED|ALIAS expr2],
    ...
    INDEX index_name1 expr1 TYPE type1(...) GRANULARITY value1,
    INDEX index_name2 expr2 TYPE type2(...) GRANULARITY value2
) ENGINE = MergeTree()   # 指定使用MergeTree引擎
[PARTITION BY expr]      # 指定分区键，如按月分区可以指定 toYYYYMM(date_column)  
[ORDER BY expr]          # 表的排序键
[PRIMARY KEY expr]       # 表的主键，默认应该和排序键相同   
[SAMPLE BY expr]         # 采样表达式
[SETTINGS name=value, ...]      # 其他MergeTree的表达参数

```

当一批数据写入到MergeTree时，遵循以下过程：

- 根据**分区键**数据被分成不同part
- 每个part中的row按照**排序键**排序存储存储（ClickHouse后台会定期合并part）
- 为每个part创建一个索引，索引文件中包含每个索引行的**主键**

通常情况下**主键**是**排序键**的前缀，或者两者相等！

## 分布式表

ClickHouse通过Distributed引擎实现集群，即表的分片功能，需要注意的是，分片和分区是两种不同的逻辑。

Distributed引擎本身不存储数据，可以他是表分片的一个统一视图，通过这张表可以实现并行读写分片的功能。


```sql

Distributed(logs, default, hits[, sharding_key])
# 上面的Distributed参数从logs集群中的default.hits表所有节点上读写数据
# sharding_key 可以是任何能够返回常量的表达式，比如可以使用rand(),或者intHash64(UserID)

```

向Distributed表写数据的方式有以下两种：

- 自己制定要将数据写入那个分片，这时候实际上不是从Distributed写入，而是直接在分片表上写入
- 直接在Distributed表上写入，通过片键来决定实际写入到哪张表。

Distributed表的Shard路由方式是：

- 计算片键表达式
- 计算结果除以所有分片的权重总和得到余数
- 发送row到余数落在[ prev_weight, prev_weights + weight) 的分片。这个区间是这样形成的，假设有三个分片，权重分别为1、2、3，那么形成这样三个区间 [0,1), [1,3), 3,6)。


上述方式中，区间的划分可能由于shard的排列顺序不同出现差别。这样会导致一个row可能会被分到不同的shard。但是对于一批row来说，分到每个shard的row数目不会因为shard的排列出现差异。

通常，来说写Distributed的过程是异步的，即先将数据全部写到本地，然后在发送到各个分片

通常情况下，需要在config.xml文件中定义一个shard集群：

```xml

<remote_servers>
    <!-- logs表示集群的名称，可以是任意值 -->
    <logs>
        <!-- 下面定义了表的1个分片，这个分片有二个副本 -->
        <shard>
            <!-- Optional. Shard weight when writing data. Default: 1. -->
            <weight>1</weight>
            <!-- 
                通常情况下internal_replication配置应该是ture，让副本由底层表的副本机制来同步
             -->
            <internal_replication>true</internal_replication>
            <replica>
                <host>node1</host>
                <port>9000</port>
            </replica>
            <replica>
                <host>node2</host>
                <port>9000</port>
            </replica>
        </shard>
        <shard>
        <!-- 其他分片 -->
        </shard>
    </logs>
</remote_servers>

```


## 复制表

只有 MergeTree 系列里的表可支持副本，我们只需要在他们的建表语句中加上Replicated前缀即可创建复制表。

创建复制表时需要配置Zookeeper，用户可以参考以下SQL创建副本表：

```sql
CREATE TABLE table_name
(
    EventDate DateTime,
    CounterID UInt32,
    UserID UInt32
) ENGINE = ReplicatedMergeTree('/clickhouse/tables/{layer}-{shard}/table_name', '{replica}') 
PARTITION BY toYYYYMM(EventDate)
ORDER BY (CounterID, EventDate, intHash32(UserID))
SAMPLE BY intHash32(UserID)

# 上面的SQL中，ReplicatedMergeTree函数传入了两参数，参数中大括号的内容会被config.xml中macros中的宏替换掉

# 1. 第一个参数是表在Zookeeper中path，这个路径每张表应该是唯一的，{layer}-{shard}部分实际上是表的分片信息。示例中给出layer，shard两个字段，原因是因为这是一种两级分片的方案。
# 2. 第二个参数是副本名称，这个配置要求每个副本唯一，表示同一个分片的不同副本。


```

复制表有如下特点：

- create语句来创建副本表时，只会在当前的机器上创建一个副本。因此，如果要创建一个三副本的复制表，那么要在三个不同的机器上执行三次create命令。
- drop语句和create类似，只会删除当前机器上的副本
- Zookeeper 如果故障，那么会导致所有副本表变成只读状态
- 复制是多主异步的，INSERT 语句（以及 ALTER ）可以发给任意可用的服务器。数据会先插入到执行该语句的服务器上，然后被复制到其他服务器。
- 默认写一个副本即写入成功，但是可以配置为insert_quorum 模式
- 如果写入相同的数据块，那么写入会被去重。
- 对于轻微的数据不一致，clickhouse会借助ZK自动修复。但是如果某个副本出现数据损坏，或者非常严重的不一致，那么需要用户手工介入来进行修复。

总体来说，ClickHouse的复制机制虽然比较灵活，但是个人认为有以下缺陷：

- 使用起来比较繁琐，无法向一般表一样建表删表。
- 官方文档没有说明使用什么机制保证副本间的一致性，从描述中看来应该是一个**最终一致**的系统
- 数据不一致时，Failover的机制不够自动化

## 其他表引擎

ClickHouse还支持内存表，Log表等表引擎。但是按照官方文档的叙述，这些表并不适合应用在生产环境中。



# 安装部署

ClickHouse可以使用RPM包部署（[Repo](https://packagecloud.io/altinity/clickhouse)），可以使用VPS[搭建私库](https://packagecloud.io/Altinity/clickhouse/mirror)。

```shell

# ClickHouse 要求内核支持SSE 4.2指令集，可以使用下面的命令检查

grep -q sse4_2 /proc/cpuinfo && echo "SSE 4.2 supported" || echo "SSE 4.2 not supported"

# 通过yum安装

yum -y install clickhouse-client clickhouse-server

```

安装完成之后，/etc/clickhouse-server目录下包含配置文件：config.xml、users.xml，前者是全局配置文件，后者是用户权限配置文件。同时要注意，config.xml中定义了clickhouse的数据目录，启动时需要将own改为clickhouse:clickhouse！！

```shell

service clickhouse-server start/stop/status

```

启动服务之后可以使用clickhouse-client连接服务！

## 配置文件示例

以下配置创建一个三节点的ClickHouse集群，集群中数据互为备份


# 总结

个人认为很适合IData当前的场景，可以使用ClickHouse替换掉Impala、hive、HBase这三个组件，并且还可充当时序数据库使用。在优化IData的表设计之后，应该能够提升SQL的查询性能！！！

优点：

1. 性能：9分。

    - 对于单表操作来说，性能基本是傲视群雄的，从收集到压测信息看是Impala、Greenplume等MPP架构的3~5倍。
    - join性能表现不如单表性能抢眼，但是从别人的测试结果上看下限依然强于SparkSQL，上限可能不会超过Impala
    - ClickHouse能在单机部署的情况下载就展示出非常好的性能，非常适合小集群

2. 可维护性：9分。
   
   - RPM包安装，就一个配置文件，安装部署和mysql一样
   - 有中文社区，官方的中文文档比较完善，基本看看就能上手

3. 社区生态：7分。

    - JDBC、ODBC、UI、SDK接口之类该有的都有

缺点：
 
1. 分片、副本功能很灵活，但是比较繁琐。ClickHouse的集群方式实际上是在多个实例上套用一个分库分表工具，并没有像Hadoop生态中的大部分工具那样自动Rep、Shard
1. SQL语法不完善，没有做到100%兼容标准SQL
2. 适合批量查询、批量修改，单条查询/修改的能力很差
   


# 其他知识

## 稀疏索引

稀疏索引,其相对的概念是**稠密索引**，它们都属于DB的**聚集索引**。

- 聚集索引定义：在一个文件中可以有多个索引，分别基于不同的搜索码。如果记录按照某个指定的顺序排序，那么该搜素码对应的索引就是聚集索引。

- 稠密索引定义: 在稠密索引中，每个搜索码值都对应一个索引值（记录）。

- 稀疏索引定义：在稀疏索引中，只为某些记录建立索引项。

稀疏索引的优势在于索引占用的空间小，并且插入和删除所需的性能开销同样也小。但是定位单条记录的能力弱。

# 参考

[中文介绍] (https://clickhouse.yandex/docs/zh/)

[GDPR-通用数据保护条例](https://gdpr-info.eu/)

[ClickHouse 中文论坛](http://www.clickhouse.com.cn/)

[ClickHouse 表引擎介绍](http://www.clickhouse.com.cn/topic/5c74dc8169c415035e68d4e1)

