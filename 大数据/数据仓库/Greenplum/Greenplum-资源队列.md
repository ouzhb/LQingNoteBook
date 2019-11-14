---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-17
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 资源队列

**资源队列**是Greenplum的**默认资源管理方式**，包括以下特点：

- 可以定义多个不同的资源队列，pg_default是默认队列；
- Role需要和一个队列绑定，不明确绑定时和pg_default绑定；
- 队列定义了并发数、内存、CPU等资源，ROLE的SQL消耗队列中的资源；
- 拥有SUPERUSER属性的角色将会不受资源队列的限制，查询立即执行（经过测试：发现并发数限制对SUPERUSER不起作用）；
- resource_select_only=on时，INSERT、UPDATE、DELETE不会受到资源队列影响；
- 在执行EXPLAIN ANALYZE命令期间的SQL不受资源队列影响；

资源队列包含以下特性：

- MEMORY_LIMIT：每个Segment中所有查询所使用的的内存的量；
- ACTIVE_STATEMENTS：该队列的并发查询限制；
- PRIORITY：队列的CPU优先级，包括LOW、MEDIUM、HIGH、MAX级别，默认为MEDIUM，级别越高CPU使用越优先；
- MAX_COST：优化器评估上限，当优化器对SQL的消耗评估大于这个值时，SQL被队列拒绝。

**默认队列pg_default**的配置为：ACTIVE_STATEMENTS=20、PRIORITY=MEDIUM、没有MEMORY_LIMIT和MAX_COST。

内存管理相关注意：

- MEMORY_LIMIT不设定时，一个资源队列的可用内存大小是statement_mem*ACTIVE_STATEMENTS（statement_mem指定当前会话分配到内存）

- 设定MEMORY_LIMIT时，并行度受到当前使用的内存影响；

- 设定MEMORY_LIMIT时，每个会话分配的内存为MEMORY_LIMIT/ACTIVE_STATEMENTS（不指定statement_mem时）；

- statement_mem可以覆盖会话的内存分配，取值范围是min(MEMORY_LIMIT, max_statement_mem)，命令为为set statement_mem='128MB'；

- 配置文件级别也有statement_mem配置，值为125MB（是否生效？？）；

- 队列一旦分配出内存，直到查询结束才回收这一部分配额；

- **gp_vmem_protect_limit**决定了，单个Segment中所有队列的可用总内存；

PRIORITY管理相关注意：

- SQL按照其资源队列的优先权共享可用的CPU资源;
- SQL的复杂度不影响CPU的分配;
- 有新的SQL开始运行时，CPU份额将会被重新计算；

## 配置资源队列

相关配置参数，包括以下：

- 用于资源队列的一般配置：

	- max_resource_queues 
	- max_resource_portals_per_transaction 
	- resource_select_only 
	- resource_cleanup_gangs_on_wait 
	- stats_queue_level 

- 内存利用有关配置：

	- gp_resqueue_memory_policy 
	- statement_mem
	- max_statement_mem 
	- gp_vmem_protect_limit 
	- gp_vmem_idle_resource_timeout（大并发时调整）
	- gp_vmem_protect_segworker_cache_limit （大并发时调整）
	- shared_buffers: 共享内存缓冲区大小，至少为128MB并且至少为16MB以max_connections。

- CPU优先级配置：

	- gp_resqueue_memory_policy 
	- gp_resqueue_priority_sweeper_interval 
	- gp_resqueue_priority_cpucores_per_segment：每个Segment实例分配的CPU核数。Master和Segment的默认值是4，一般需要将HOST的所有CPU都利用上。

## SQL命令

```sql
-- 创建资源队列
CREATE RESOURCE QUEUE adhoc WITH (ACTIVE_STATEMENTS=3);
CREATE RESOURCE QUEUE myqueue WITH (ACTIVE_STATEMENTS=20, 
MEMORY_LIMIT='2000MB');
-- 设定优先级
ALTER RESOURCE QUEUE adhoc WITH (PRIORITY=LOW);
-- 设定并发数
 ALTER RESOURCE QUEUE reporting WITH (ACTIVE_STATEMENTS=20);
-- 关联队列和role
ALTER ROLE name RESOURCE QUEUE queue_name;
CREATE ROLE name WITH LOGIN RESOURCE QUEUE queue_name;
-- 移除资源队列
ALTER ROLE role_name RESOURCE QUEUE none;
-- 删除资源队列
DROP RESOURCE QUEUE name;
-- 查看ROLE绑定的资源队列
SELECT rolname, rsqname FROM pg_roles, 
          gp_toolkit.gp_resqueue_status 
   WHERE pg_roles.rolresqueue=gp_toolkit.gp_resqueue_status.queueid;

```

## 查看队列中的语句和资源队列状态

- gp_toolkit.gp_resqueue_status可以查看队列资源的使用情况；
- stats_queue_level = on可以收集统计信息和性能，通过pg_stat_resqueues可以查看收集到的信息；
- gp_toolkit.gp_locks_on_resqueue可以查看等待的SQL；

- 参看当前活跃或者等待的SQL，如果需要结束这些SQL执行pg_cancel_backend(31905)；

```sql
SELECT pg_stat_activity.pid,rolname, rsqname,granted, datname,query
FROM 
   pg_roles, gp_toolkit.gp_resqueue_status, pg_locks, pg_stat_activity 
WHERE pg_roles.rolresqueue=pg_locks.objid 
   AND pg_locks.objid=gp_toolkit.gp_resqueue_status.queueid
   AND pg_stat_activity.pid=pg_locks.pid
   AND pg_stat_activity.usename=pg_roles.rolname;
```
- gp_toolkit.gp_resq_priority_statement可以查看SQL优先级，超级用户可以修改某个SQL的优先级（gp_adjust_priority函数）

## 内存配置对资源队列影响：

主要关注：vm.overcommit_ratio、gp_vmem_protect_limit、shared_buffers

- 主机内存，Segment主机的可用内存，主要由**vm.overcommit_ratio**配置控制（此处讨论的情况是Segment主机单独部署的情形）

```shell
# 通常配置95即可，若果是资源组模式可以配置50
vm.overcommit_ratio = 95
```

- Segment实例内存，每个Segment的可用内存由**gp_vmem_protect_limit**控制：

```shell
# gp_vmem_rq是GP使用的内存，计算公式为（0.95*RAM - 7.5GB）/1.7
# mirror不计入活跃内存
gp_vmem_protect_limit = gp_vmem_rq / 最大活跃Segment数目
```

- shared_buffers: 共享内存缓冲区大小，至少为128MB并且至少为16MB以max_connections。

# 其他知识

## 1. 基线硬件性能

gpcheckperf 可以进行：

- 磁盘I/O测试（dd测试）：默认情况下，在会在磁盘目录下读写2倍内存大小的文件
- 内存带宽测试（流) ：使用STREAM基准程序来测量可持续的内存带宽（以MB/s为单位），该测试不涉及CPU计算性能。
- 网络性能测试（gpnetbench*）：当前主机发送5秒钟的数据流到测试中包含的每台远程主机。数据被并行传输到每台远程主机，支持串行（一台一台通信）、并行、全矩阵测试。

测试命令：[参考](https://gp-docs-cn.github.io/docs/utility_guide/admin_utilities/gpcheckperf.html)

## 2. CPU带宽与内存带宽的计算

内存带宽：

```
	内存带宽=内存（等效）频率（内存工作频率X倍频，DDR内存为2，DDR2内存为4，DDR3内存为8）X位宽/8。
```

CPU的工作频率涉及**主频、外频、倍频**三个概念

	- 主频：CPU自身的工作频率
	- 外频：电脑主板提供的系统总线频率，外频是其他组件和CPU通信的基准（lscpu |grep "CPU MHz"，外频可能因为一些其他原因不断变化）
	- 倍频：主频/外频

CPU的带宽一般指：CPU与北桥数据交换的速度，也叫作**前端总线FSB**。早期，FSB和外频一致的，使用QDR技术后，前端总线的频率成为外频的两倍或者是四倍。

```
Intel处理器前端总线（FSB）= 处理器前端总线频率（MHz，处理器外频X4）X位宽（Bit）/8

```

参考:[PC总线带宽与内存带宽的计算](https://www.cnblogs.com/l1pe1/archive/2012/02/02/2335557.html)

