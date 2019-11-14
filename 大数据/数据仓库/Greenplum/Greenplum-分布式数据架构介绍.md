---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-08
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 并行计算

并行计算可以划分为以下分类：

- 时间并行计算：流水线技术
- 空间并行计算：使用多个处理器执行并发计算

    - 单指令流多数据流架构（SIMD）
    - 多指令流多数据流架构（MIMD）

        - PVP：并行向量处理机
        - SMP：对称多处理机，CPU对称工作，无主从关系，并共享内存（一致存储器访问结构，UMA）。这种架构极易产生内存瓶颈，扩展性很差。
        - MPP：大规模并行处理机，由多个SMP服务器通过网络连接构成。工作过程完全没有内存共享，因此需要数据重分配机制进行一些并行工作！
        - COW：工作站集群
        - DSM（NUMA）：分布式共享存储处理机，具有多个CPU模块,每个CPU模块由多个CPU组成，并且具有独立的内存，I/O槽口。CPU模块间通过交互模块连接，本质上每一个CPU可以访问全局内存（本模块内的 + 通过交互模块访问其他模块的内存）。由于访问不同内存的性能有所差别，因此称为非一致存储架构，对其扩展并非是线性扩展。

并行数据库的三种基本架构：

- 共享内存结构（Shard-Memory）：多个具备独立磁盘的CPU + 高性能共享内存，实例通过网络和共享内存连接。
- 共享磁盘结构（Shard-Disk）：多个具备独立内存的CPU + 多个磁盘存储，每个处理器通过网络可以读写全部磁盘（典型代表是Oracle集群）。
- 无共享资源结构（Shared-Nothing）：多个具备独立内存、磁盘的CPU（典型代表：Teradata、Vertica、Greenplum、Aster Data、IBM DB2 和 Mysql集群）



# 两阶段提交

该协议一般包含两类节点，协调者（coordinator）和事务参与者（participants，cohorts或workers）：

- 协调者：根据所有事务参与者向事务参与者发出 Commit 或者 RollBack 指令 
- 事务参与者：记录写前日志并持久存储，并向协调者发出同意（事务参与者本地作业执行成功）或取消（本地作业执行故障）

缺点：

- 所有参与节点都是事务阻塞型的，导致Greenplum的整体性能取决于最慢的Segment。
- 协调者（Master）存在单点故障问题，尤其第二阶段发生故障时，会阻塞事务。
- 数据不一致，。在二阶段提交的阶段二中，当协调者向参与者发送commit请求之后，发生了局部网络异常或者在发送commit请求过程中协调者发生了故障，导致只有一部分参与者接受到了commit请求。

2PC的改进算法是3PC：

- CanCommit阶段：协调者发送commit请求
- PreCommit阶段：

    - A：协调者获得全部Yes的反馈，发送PreCommit请求，并进入Prepared阶段。接收到PreCommit请求后事务参与者，写编辑日志并返回ACK响应。
    - B：协调者获得No的反馈或者超时，那么就中断事务，发送中断请求。Coordinator向所有Cohort发送abort请求。

- DoCommit阶段：发送doCommit请求，事务参与者确切提交后发送ACK响应。
