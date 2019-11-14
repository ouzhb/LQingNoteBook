---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-11-06
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 收集统计信息

```sql

-- ANALYZE无参数数时收集整个数据库的统计信息；
-- 可以收集单个表，或表的单个列的统计信息；
-- 不收集外表的统计信息；
-- 通常建议每天运行一次 VACUUM 和 ANALYZE；
-- ANALYZE 在表中产生 UPDATE EXCLUSIVE 锁，因此一些查询语句可能为产生冲突；

ANALYZE [VERBOSE] [ROOTPARTITION [ALL] ] 
   [table [ (column [, ...] ) ]];

-- 收集分区表的根表信息，该命令不会收集普通表的信息
ANALYZE ROOTPARTITION ALL;

-- 审定test表的note列不收集统计信息；
alter table test alter note SET STATISTICS 0;
```

相关配置：

 - optimizer_analyze_root_partition：为on，ANALYZE命令同样会收集分区表的统计信息
 - default_statistics_target：ANALYZE命令进行随机抽样时的采样系数，值越大采样越准确，所花的时间更长。一些特定的列可以单独设定这个值（ALTER TABLE ... ALTER COLUMN ... SET STATISTICS）
 - gp_enable_relsize_collection：没有统计值时，使用表的大小进行估计
 - gp_autostats_mode：NONE（不收集）、ON_CHANGE（变化收集）、no_no_stats（建表时收集一次）
 - gp_autostats_on_change_threshold：自动收集阈值，默认是20亿


# 分析执行计划

```sql
EXPLAIN [ANALYZE] [VERBOSE] statement
```

## EXPLAIN

EXPLAIN语句不会实际执行语句，只是根据当前收集的统计信息生成**执行计划树**来评估SQL运行成本。

计划树中每一个节点代表SQL需要进行的操作，并包含以下信息：

- cost：当前节点以及子节点需要读取的磁盘页，格式为：cost=xxx..xxx(第一行输出时..输出完成时)
- rows: 表示该节点需要读取的行数
- width：平均每行的字节数

## EXPLAIN ANALYZE

EXPLAIN ANALYZE 会实际执行SQL语句并且提供一些额外的统计信息：

- actual time：实际执行时间，单位是ms，格式和cost相同
- rows：实际返回的行数
- loops：？？？
- 每个Slice使用的内存情况（应该包括：work_mem和statement_mem的内存使用情况，**测试中只要statement_mem够大就不会发生磁盘IO能一定程度提高性能**）


## 执行计划中的重要关键字

- 数据扫描（Scan）：

	- Seq Scan：顺序扫描，有时候可能带有Dynamic前缀，表示分区顺序扫描
	- Shared Scan：扫描shared_buffer中的某个slice
	- Index Scan：索引扫描
	- 其他扫描子句：Bitmap Heap Scan、Tid Scan、Subquery Scan、Function Scan

- 数据移动（Motion）：

	- Gather Motion(N:1)：在master上聚合
	- Broadcast Motion(N:N)：所有Segment上广播
	- Redistribute Motion(N:N)：重分布，常见关联、Group by、开窗函数中发生。
	
		- 重分布除了IO开销之外，还会带来数据不均衡的问题！！
		- union合并表时，去重会导致重分布，并且此时以整行（所有列）进行重分布，因此慎用union（整行重分布 --> 排序 --> 去重 --> 插入结果集），另外union all虽然和并时不涉及去重，但是在写入结果集时任然会引发重分布，需要注意。

	

- Slice：将SQL拆分多个切片，Montion操作都会产生一个切片，通常Montion操作后会表名其切片号，以及涉及的segment数目。

- 数据聚合：

	- HashAggregate：基于Group By字段的hash值维护内存hash表，hash表的长度正比于聚合字段的distinct值，对n个聚合字段Greenplum需要维护n个hash表。
	- GroupAggregate：基于聚合字段排序后，对数据进行一次全扫描从而得到聚合结果。
	- 建议：GroupAggregate的性能相比HashAggregate较为稳定，当聚合函数的种类较多并且聚合键的重复性较差时会使HashAggregate使用的内存急剧上升，此时应该选择GroupAggregate方式聚合。

- 关联：涉及到广播和重分布

	- Hash join：通过内存中的Hash表来实现关联
	- NestLoop：效率最低，执行笛卡尔积时使用该方式
	- Merge Join：两表按照关联键排序，之后通过归并排序的方式关联（性能不如hash join）

- 开窗函数：

	- 当开窗函数的分布键不是表的分布键时，会引起表**多次**的重分布。
	- 如果开窗函数没有partition字段，只有Order字段那么为了维护一个全局序列，所有数据必须汇聚到Master上进行排序操作，此时Master会成为系统瓶颈。

有些参数可以控制优化器的执行计划,参考[enable_xxx配置](https://gpdb.docs.pivotal.io/6-0/ref_guide/config_params/guc-list.html)!


### 优化器开销的计算

优化器通过开销的计算结果选择SQL的执行步骤，其Cost值的计算方式是可以用参数控制的。

通常以抓取顺序页的开销作为基准单位(seq_page_cost取值为1)，以下是不同开销的默认值：

- seq_page_cost：磁盘顺序读的开销
- random_page_cost：磁盘随机读取的开销
- cpu_tuple_cost：处理一行数据的开销
- cpu_index_tuple_cost：索引扫描每个索引行的开销
- cpu_operator_cost：一次查询中执行一个操作符或者函数的开销
- gp_motion_cost_per_row：motion操作的开销
- effective_cache_size

Greenplum优化器会根据pg_class表中的relname、relpages、reltuples的值每种运行方式的cost成本，之后选择cost最小值做为执行方案。

调整经验：

- 如果内存充足random_page_cost可以适当降低；
- seq_page_cost和 random_page_cost同时降低时，会使CPU开销上升；

# Join的广播和重分布

Join通常涉及单库关联、以及跨库关联：

- 单库关联：关联键和分布键一致，此时没有数据重分布
- 跨库关联：关联键和分布键不一致，数据重新分布，装换为单库关联

|表名|字段|分布键|数据量|
|----|----|----|----|
|A|id，id2|id|M|
|B|id，id2|id|N|

以下是A、B表进行内连接时的场景，左连接和其原理类似（PS：左连接时一般不广播左表）。

遇到全连接时，Greenplum中使用Merge Join方式实现（即排序方式实现Join），全连接通常进行重分布。

```sql
-- 由于A，B表的分布键均是id，且此关联的关联键也是id，此时A，B中id取值相同的行在同一个pg库中，可以直接关联
select * from A,B where A.id=B.id

-- 表A的关联键是分布键，但是表B的关联键不是分布键
-- 方式一：将表B按照id2字段重分布到每一个节点上 ———— 重分布（处理数据量是N）；
-- 方式二：将表A广播到每个节点中 ———— 广播（处理数据量是M*节点数）；
select * from A,B where A.id=B.id2

-- 表A、B的关联键都不是分布键
-- 方式一：将表A和表B都按照id2字段，将数据重分布到每个节点，代价是M+N
-- 方式二：将小表广播
select * from A,B where A.id1=B.id2

```

PS：**Greenplum判断表的大小是通过统计信息决定的，因此如果统计信息不准确可能会使重分布策略选择错误**。
