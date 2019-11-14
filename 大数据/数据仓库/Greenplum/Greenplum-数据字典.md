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

《Greenplum企业应用实战》一书第四章读书笔记。

# 数据字典

这里的数据字典指的是 Greenplum 中元数据信息，包括：pg_ 和 gp_ 开头的一些系统表。

## oid

oid 是 PG/GP 中用来表示对象（包括：表、函数、操作符等等）的全局递增 id （是32位数字），GP 中绝大多数数据字典通过oid相互关联。

表名、函数名、操作符名可以和 oid 相互转换：

```sql

-- 类似的还有
-- regclass : pg_class中的oid关联
-- regproc/regprocedure : pg_proc 中的oid关联
-- regoper/regoperator：pg_operator中的oid关联

-- pg_class表的oid是1259
select 1259::regclass ; 
select oid,relname from pg_class where oid='pg_class'::regclass;

```

## 常用数据字典

不同Greenplum版本数据字典有不同程度的变化，具体表结构可以参考[Pivotal官网](https://gpdb.docs.pivotal.io/6-1/ref_guide/system_catalogs/gp_segment_configuration.html)，或者参考[Postgresql手册——系统表](https://www.php.cn/manual/view/20933.html#)。

- gp_segment_configuration：集群配置信息、状态表。
- pg_class：保存所有表、视图、序列、索引的元数据信息，所有DDL、DML需要和该表发生关系。

	- relname：表名
	- relnamespace：表空间oid和pg_namespace关联
	- relpages：磁盘页大小，一般默认32kb
	- reltuples：表的行数
	- relstorage：表类型，如堆表、appendonly表、视图、外部表

- pg_attribute：字段信息表，所有表的字段类型、名称等信息被保存在该表中

	- attrelid：所属的表，关联pg_class.oid
	- attname: 字段名称
	- atttypid：字段类型，关联pg_type.oid
	- attstattarget：和ANALYZE相关的配置，0表示该字段不收集统计信息，负值表示默认统计，正值表示收集是创建的柱状图数目。
	- pg_attribute可以展示表的隐藏字段：
		
		- gp_segment_id：隐藏字段，表示改行记录的存储位置
		- tableoid
		- oid
		- cmax、xmax、cmin、xmin等

- gp_distribution_policy：保存表的分布键

	- localoid：关联表的oid
	- distkey: int2vector类型的数组和pg_attribute的attnum绑定

- pg_statistic：保存表的统计信息，以列为单位。

	- stanullfrac：该字段中为NULL记录的比率
	- stawidth：非NULL记录的平均存储宽度，以字节计
	- stadistinct：重复率，这个值有两种统计方式，1. 大于零的数值，此时该值是确切的非重复记录数；2. 小于0的值，此时表示重复率（如-0.5表示，一个记录平均出现2次）

- pg_partition：分区信息表

	- oid：唯一标识分区表的oid和pg_class.oid不同
	- parrelid：关联 pg_class.oid 
	- tablename/partitiontablename：表名和分区表名
	- parkind：分区模式
	- parlevel：表示分区的层级
	- paratts / parnatts：分区键信息

- pg_cast：定义了Greenplum的类型转化关系

	- castsource：源类型，关联pg_type.oid
	- casttarget：目标类型，关联pg_type.oid
	- castfunc：转换函数，如果是0表示二进制兼容，关联pg_prov.oid

- pg_locks：保存了数据库的锁信息（可以查询这张表的视图gp_toolkit.gp_locks_on_relation）
- pg_stat_activity：当前正在运行的SQL

## gp_toolkit

gp_toolkit是Greenplum为了方便查询数据字典定义了许多视图，用户通过该工具箱可以快速查询想要的信息。

关于gp_toolkit的详细内容可以参考[gp_toolkit管理方案](ocs-cn.github.io/docs/ref_guide/gp_toolkit.html#topic2)。

可以关注以下几张表：

- gp_bloat_diag：识别需要VACUUM或者VACUUM FULL命令回收已经删除的或者废弃的行占据的空间的表
- gp_stats_missing：统计信息缺失的表（不含分区表的父表，或者一些空表）
- gp_locks_on_relation：当前表上持有的锁
- gp_locks_on_resqueue：当前资源队列持有的锁
- gp_log_*：关于日志信息的外部表
- gp_param_setting：配置信息表
- gp_size_of_*：检查数据库对象的大小和磁盘空间