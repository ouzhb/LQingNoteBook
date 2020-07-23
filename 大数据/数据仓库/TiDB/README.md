---
title: 数据库调研笔记 -- TiDB
categories: "笔记"
date: 2019-06-03
comments: true
toc: true
tags:
	- NewSQL
	- TiDB
	- 数据库
---


NewSQL 调研笔记


<!--more-->


# TiDB

## 特性

1. 兼容 MySQL，安装完成后，可以使用mysql客户端以及JDBC直连。
2. 支持水平扩展，包括:计算节点和存储节点，同时提供强一致性、高可用性（从官方的说明介绍中，TiDB实际上是一个CA型数据库）。
3. 云原生 SQL DB，能够容易的部署到K8S上，并且提供相应的Kubernetes Operator套件。
4. 支持 TiSpark 工具，通过这个工具可以解决LDAP问题。

## 整体架构

三个核心组件：TiDB Server、PD Server、TiKV Server

![TiDB架构](https://pingcap.com/images/docs-cn/tidb-architecture.png)


|组件|作用能力|
|---|---|
|TiDB Server |负责接收用户请求，并进行聚合计算，无状态，可以横向水平扩展。|
|PD Server|管理模块，存储集群的元数据、执行TiKV的负载均衡、管理事务ID。|
|TiKV Server|以Region为单位提供数据的KV存储，并通过Raft协议进行Region的复制，保持数据的强一致性和容灾。|
|TiSpark|Spark SQL 对接 TiDB 的插件，提供LDAP场景能力|


## 安装部署

在生产环境中部署，官方提供[Ansible安装包](https://pingcap.com/docs-cn/dev/how-to/deploy/orchestrated/ansible/)，安装过程比较友好。

个人部署了一把，非常的顺利，调整好系统的内核参数后，几乎是一键部署！

官方文档推荐生产TiDB需要部署9个实例（2计算+3管理+3存储+1监控，共占110核，220G内存，12SSD+5SAS）。

TiDB对PD、TiKV的磁盘性能要求比较高，甚至在安装包中了fio检查，当磁盘性能不达标时终止安装（当然用户可以调整这个指标）。

官方推荐SSD指标，如下：

|指标|官方推荐|
|---|---|
|随机读iops|40000|
|随机写iops|10000|
|混合随机读写iops|10000|

上面的指标，普通SATA接口的SSD不进行RAID很难达到。 PCI-E 接口的SSD应该可以达到上面的性能。

## MySQL兼容性

TiDB 支持 MySQL 传输协议，当前Tidb支持MySQL 5.7版本，可以使用MySQL客户端直连，并且支持mysql的备份恢复工具。

TiDB是虽然号称和MySQL完全兼容，但是依旧有部分特性缺失，如：存储过程与函数、视图、触发器、事件、自定义函数、外键约束、全文函数与索引、空间函数与索引、非 utf8 字符集 ......

TiDB MySQL的兼容性差异，可以看pingcap的[官方文档](https://pingcap.com/docs-cn/dev/reference/mysql-compatibility/)，其中有详细说明。

## TiSpark

TiSpark 是 pingCAP 基于 Spark Catalyst 的Tidb扩展数据引擎，能够直接读取 Tikv 上的数据的OLAP插件。

在一个6节点的SSD环境上测试，有如下结果：

1. TiSpark执行复杂查询（TPC-DS）的性能相比直接运行JDBC查询有20%~30%提升。
2. TiSpark在小数据集（亿以内）的表现不如Impala，稍微优于SparkSQL。但是TiSpark目前只能进行查询，不能进行插入。但是目前使用JDBC大批量插入TiDB的效率远逊于Hive等SQL on Hadoop！

## BenchMark

pingCAP公司官网有发布了一个[TPC-H 50G 性能](https://pingcap.com/docs-cn/benchmark/tpch/),报告主要展示了TiDB 1.0到2.0的性能提升，但是从结果上来看TiDB 2.0的性能相比Greenplum等MPP引擎来说依然没有优势。

TiDB的官方仓库提供了 [Bench Market](https://github.com/pingcap/tidb-bench) 测试工具，包含OLTP和OLAP两种类型的测试。其中，OLAP测试包括TPC-DS、TPC-H两种测试集，以及聚合性能测试，以及SSB压测工具。

测试结果参考：[TPC-DS基准测试.xlsx](https://github.com/LinQing2017/DevOpsTools/tree/master/bench_test/case/TPC-DS基准测试.xlsx)


# 总结

1. 易用：8分。

   - 通过官方Ansible工具一键部署，无需额外开发。
   - 能够兼容MYSQL绝大多数特性、允许MySql工具直连。
   - 可以使用TiSpark和Spark无缝集成。

2. 性能：6分。

    - 并不是专门为OLAP场景设计的DB，对偏重OLAP的项目来说，无论写入或者查询都不是太擅长。
    - TiSpark不支持批量导入，而使用JDBC导入数据的性能太差。
  
3. 可用性：6分。

   - 极其耗费磁盘性能，上生产需要使用专用SSD！
   - TiSpark 和 CDH Spark 似乎有一些兼容性问题！
   - pingCAP提供的官方文档虽然比较全面，但是玩家似乎还比较少，遇到故障时，FAQ资料很少，因此故障运维难度不小。


总体来说，目前TiDB和无线的大数据场景并不契合。


# 其他知识

## 共识算法：Raft

 解决简化的拜占庭将军问题：假设将军中没有叛军，信使的信息可靠但有可能被暗杀的情况下，将军们如何达成一致性决定？

 同类型算法包括：Paxos

### Raft的一致性方案

核心思想：**先在所有将军中选出一个大将军，所有的决定由大将军来做。**

选举方式：每个将军持有一个随机时间的计时器，倒计时最先结束的将军发起投票推举自己为大将军。当获得半数以上投票时，选举结束，否者重复选举。

Raft 算法中保持一致性的几种场景：

1. Leader不停的向Follower发送心跳来保证自己存活。
2. 多个Leader的场景：Raft中每次选举有一个递增的ID来标记，非最新选举的Leader会自动降级为Follower。
3. 选举时出现多个Candidate，如果多个Candidate同票，这种情况下选举会因为超时而失败。下一轮选举会重新投票。

## CAP原理

|特性|解释|
|---|---|
|Consistency|“一致性”，对于每一次读操作，要么都能够读到最新写入的数据，要么错误。|
|Availability|“可用性”，对于每一次请求，都能够得到一个及时的、非错的响应。|
|Partition tolerance|"分区容错"，即系统出现网络分区后，必须是可恢复的！<br> 只要是分布式DB，这一条都必须满足。|

所有的分布式系统都属于CP或者AP系统，而mysql等传统DB属于CA！

通常CP系统出现网络故障的话，数据同步时间可能无限延长，此时系统会停止对外提供服务，来保证数据一致性，而AP系统在分区场景下依旧提供服务，但是用户可能发现系统数据存在不一致的情况。

当前一些分布式式系统可以通过一些配置，使系统在提供CP、AP两种不同的特性（比如MongoDB、Kafka）。




# 参考

[TiDB 官网](https://pingcap.com/docs-cn/)

[Raft 算法原理](https://www.jianshu.com/p/8e4bbe7e276c)