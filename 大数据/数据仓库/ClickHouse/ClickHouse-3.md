---
title: 数据库调研笔记 -- ClickHouse之配置项
categories: "笔记"
date: 2019-06-21
comments: true
toc: true
tags:
	- 数据库
	- ClickHouse
---


数据库调研笔记 -- ClickHouse


<!--more-->

# 服务器配置

## CPU

- 强烈建议使用内核的 SSE 4.2 功能，08年之后的处理器全部支持该功能。官方建议，CPU核数对于性能的影响大于主频，如：16 cores 2600 MHz的CPU性能强于8 cores 3600 MHz。

- 开启BIOS里的Hyper-threading配置，能够优化部分查询的性能

- 开启BIOS里的Turbo Boost功能

- 在配置 CPU Scaling 为 performance 模式

```shell
# 确认sse 4.2配置
grep -q sse4_2 /proc/cpuinfo && echo "SSE 4.2 supported" || echo "SSE 4.2 not supported"
# 配置CPU Scaling模式
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## 内存

- 当查询的数据量小于200GB（压缩后）时，建议客户端内存和数据量配置一致，这样能够达到最佳性能；
- 当查询的数据量较大时，建议配置128GB以上的客户端内存；
- 关闭页面缓存
- 关闭Huge Pages
```shell
echo 'never' | sudo tee /sys/kernel/mm/transparent_hugepage/enabled
```

## 磁盘

- 能用SSD，建议用SSD
- 建议使用 RAID-10, RAID-5, RAID-6 ，RAID-50
- 推荐使用EXT4文件系统，挂载时添加noatime, nobarrier参数

## Zookeeper

不要使用ZK的默认配置，默认配置下zk不会自动清理编辑日志文件，建议使用官方推荐的配置：https://clickhouse.yandex/docs/zh/operations/tips/#zookeeper。

```properties
# ZK的autopurge配置
autopurge.snapRetainCount=10
autopurge.purgeInterval=1
```

# 客户端配置

客户端配置一般分为两类：

- max_xxx_xxx : 表示限制使用的资源数目，配置为 0 时表示不限制！
- overflow_mode ： 表示超出资源限制时，CK执行的操作，配置throw表示退出，break表示返回部分查询结果。

## 性能相关的配置

参考[官方文档](https://clickhouse.yandex/docs/zh/operations/settings/query_complexity/),目前看来下面三个最有用：

- max_memory_usage ：单个server执行**单个查询**使用的最大内存，默认10G

- distributed_product_mode：子查询中包含，分布式表时CK的查询行为。[参考](https://clickhouse.yandex/docs/zh/operations/settings/settings/#distributed-product-mode)

- enable_optimize_predicate_expression: 谓词下推功能，默认配置为0，即关闭

- max_threads： 单个查询使用的最大线程数目，默认是2

- max_block_size ： 单个block中数据的行数，默认是65,536

- max_insert_block_size： 单次插入的块大小，建议略大于max_block_size

