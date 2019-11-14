---
title: HBase Tools 
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-04-08
comments: false
toc: true
tags:
	- HBase
---

HBase Tools 介绍 ~~~ 持续更新！！

<!--more-->

# Canary

## Read
Canary 工具可以对HBase集群的表进行"canary-test"。该检查支持一下三种模式：

- region mode： 从每个region的每个column-family上读取一行，如果都OK，那么测试通过。该模式是默认模式！
- regionserver mode： 从每个RegionServers的所有Region中随机选择一行读取，如果OK，那么测试通过。
- zookeeper mode： 读取HBase的Znode节点，如果读取正常，那么测试通过。

以下是调用示例：

```shell
# 所有table以region mode执行一次检查
hbase canary 
# 检查所有dim_开头的表
hbase canary -e dim_.*
# 检查table表
hbase canary dim_ac_info
# 以regionserver mode方式执行检查
hbase canary -regionserver
# 每隔5秒运行一次检查，并且即使检查发生错误，cannary进程也不会退出。超时时间为60s
hbase canary -daemon -interval 5 -f false -t 60000
# 每隔5秒运行一次检查，检查发生错误，cannary进程退出
hbase canary -daemon -interval 5 -treatFailureAsError
# 在Kerberos hbase环境中进行检查
hbase canary -Dhbase.client.kerberos.principal=hbase -Dhbase.client.keytab.file=/etc/krb5.keytab

```

## Write

默认情况写入检查时不开启的，通过-writeSniffing配置可以开启写入检查。

进行写入检查时，canary进程会写入一张指定的表，这张表由Canary进程创建，并且该表的分区分布在所有的region servers上。

```shell
# 打开写入检查，默认情况下，写入表为hbase:canary
hbase canary -writeSniffing
# 执行canary写入的表名，以及写入大小（默认为10b）
hbase canary -writeSniffing -writeTable ns:canary -Dhbase.canary.write.value.size=100
```

## Cloudera

cm页面集成了canary的配置项，默认情况下该配置不打开，并且不能调整Canary参数。

```shell
# Cloudera 运行Canary的命令如下：
org.apache.hadoop.hbase.tool.Canary -t 15000 -daemon -interval 6 -regionserver bdnode2
```

# RegionSplitter

手工Region预分区工具（[参考](http://hbase.apache.org/book.html#manual_region_splitting_decisions)），通过这个工具可以手工创建一张指定分区数目的新表，用户可以指定不同的Key策略，包括：HexStringSplit、DecimalStringSplit、UniformSplit


```shell
hbase regionsplitter  {table name} HexStringSplit   -c 10 -f cf1:cf2
```

在hbase shell 中可以使用 split 命令和 merge 命令进行强制region拆分、合并操作。

关于 Region 拆分合并的知识可以参考这篇[blog](https://www.cnblogs.com/hopeiscoming/p/10168932.html)

# Health Checker

HBase在[HBASE-7351](https://issues.apache.org/jira/browse/HBASE-7351)中提供了Health Checker机制，让HBASE服务通过sh调用的方式来确认master、regionserver等服务是否可用。

```shell

# health checker脚本涉及到以下几个配置项
 hbase.node.health.script.location
 hbase.node.health.script.timeout
 hbase.node.health.script.frequency Default is every 60seconds.
 hbase.node.health.failure.threshold Defaults to 3.
```

官方提供hbase-examples/src/main/sh/healthcheck/healthcheck.sh脚本作为设计checker脚本的参考。


# Driver

HBase 提供一些便利的Driver Class，这些Class可以通过bin/hbase命令来运行。

通过${HBASE_HOME}/bin/hbase org.apache.hadoop.hbase.mapreduce.UtilityName可以运行以下driver类

|UtilityName|说明|示例|
|----|----|----|
|LoadIncrementalHFiles|将本地HFILE文件导入到HBase|
|CopyTable|将一张表从一个HBase导出到另一个HBase|
|Export|将表导出到HDFS中|
|Import|将表导入到HBase中|
|ImportTsv|将CSV导入到HBase中|
|RowCounter|统计表的行数|
|CellCounter|统计表的cell数目|
|replication.VerifyReplication|比较两个HBase集群的表|

# hbck 和 HBCK2

hbck是一个完整性、一致性工具，在hbase-1.x 版本之前该工具可以诊断表的完整性、同时可以用于修复不一致表。hbase-2.x版本以后，该工具被HBCK2取代，只能只读运行，不提供修复功能。


使用以下命令可以进行HBase表的一致性检查：

```shell
# 全表检查
hbase hbck -details
# 检查指定表
hbase hbck Table1 Table2
```

官方文档强烈不建议使用hbck去修复hbase 2.x的不一致，原因是hbck会绕过Master直接访问HDFS上的HFile文件，可能造成更严重的不一致。

参考hbck的深入介绍：[hbck In Depth](https://hbase.apache.org/2.1/book.html#hbck.in.depth)

HBCK2 工具是一个独立于HBase的[项目](https://github.com/apache/hbase-operator-tools/tree/master/hbase-hbck2)，其jar包不随HBase一同发布需要用户从源码进行编译。

由于 CDH 6.1.0 是基于 HBase 2.1.0 开发，HBCK2 说明文档中提及该工具不支持 2.0.3 和 2.1.1 之前的HBase版本，HBase只在3.x的官方文档中提到了这个工具。所以该工具用在hbase 2.1.0-CDH 6.1.0 上也不一定靠谱。

HBCK2提供了以下功能：

- bypass：释放一个或者多个卡住的procedure
- assigns/unassigns：对应region，这两个操作在 hbase shell 中也可以运行。
- setTableState：设定表的状态，可以将表设定为ENABLED, DISABLED, DISABLING, ENABLING
- serverCrashProcedures：？？？？这个不知道干啥用的

参考这篇blog说明了HBCK2的一些常见用法( 基本上是HBCK2 说明文档的翻译)：[HBase 2.0之修复工具HBCK2运维指南](https://yq.aliyun.com/articles/683107)

# HFile Tool

使用HFile Tool可以查看HBase在HDFS上的hfile文件的内容

```shell
hbase hfile -v -f  hdfs://nameservice1/hbase/data/default/TestTable/d051c004ff7f7441eb7f89eed9136c57/info0/c3d62dcd7c3f4b8dacfc9688b850d723
```
# WAL Tools

主要是用来查看hbase的 WAL 日志，以及拆分WAL日志
```shell
hbase org.apache.hadoop.hbase.regionserver.wal.FSHLog --dump hdfs://nameservice1/hbase/MasterProcWALs/pv2-00000000000000000048.log
hbase org.apache.hadoop.hbase.regionserver.wal.FSHLog --split {wal_log path}
```

hbase wal 命令同样可以用来查看wal日志

# CopyTable

表备份工具，可以跨集群备份，也可以同一个集群备份。

# HashTable/SyncTable

表的同步工具，两张表可以运行在不同的集群上。使用这个工具可以进行表的增量备份。

# Export

支持Mapreduce-based和Endpoint-based两种方式进行Export。

```shell
# Mapreduce-based Export
bin/hbase org.apache.hadoop.hbase.mapreduce.Export <tablename> <outputdir> [<versions> [<starttime> [<endtime>]]]
# Endpoint-based Export
bin/hbase org.apache.hadoop.hbase.coprocessor.Export <tablename> <outputdir> [<versions> [<starttime> [<endtime>]]]
```


# ImportTsv、CompleteBulkLoad、Import

这三个都是批量导入工具。


# 参考

[官方文档](http://hbase.apache.org/book.html#ops_mgt)
