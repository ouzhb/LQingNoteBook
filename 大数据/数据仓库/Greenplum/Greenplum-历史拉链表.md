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

《Greenplum企业应用实战》一书第三章读书笔记。

# 历史拉链表

历史拉链表主要用于：记录一个事务从开始一直到当前状态的所有变化信息。相比于定时快照，历史拉链表结构可以避免数据海量存储，是处理缓慢变化数据的常见方式。

记录member事实表的变换情况，可以通过以下四张表实现历史拉链表：

- member_fatdt0：事实表，添加 dw_end_date 和 dw_beg_date 字段分别表示记录的失效时间和生效时间，使用 dw_end_date 分区，最后一个分区（取值为无穷大）是最新数据，其余分区是失效数据。
- member_delta：当天数据库的变更，action字段表示数据操作类型（I,U,D）。
- member_tmp0：刷新过程的临时表，有二个分区记录历史数据（当天失效数据）和当前数据，结构和member_fatdt0一样。
- member_tmp1：刷新过程的临时表，用来交换分区，结构和member_fatdt0一样。

更新过程：

- 当天任意数据变更插入到 member_delta 中，但次日凌晨将 member_delta 合并到 member_fatdt0 中
- member_delta 和 member_fatdt0 的合并参考下面的步骤：

    - member_fatdt0 和 member_delta 最后一个分区使用 member_id 进行左外连接，关联上了说明数据发生变更，关联不上说明没有发生变更。
        
        - 关联上的数据修改 dw_end_date 插入 member_tmp0 的历史分区
        - 关联不上的数据插入 member_tmp0 的当前分区（即今天没有发生变更的数据）

    - 将 member_delta 中 action 类型为（I,U）的插入到 member_tmp0 当前数据分区（dw_end_date = 无穷大， dw_beg_date = 当天时间）
    - 将 member_fatdt0 的当天分区和 member_tmp0 历史数据分区交换
    - 将 member_fatdt0 的最后一个分区和 member_tmp0 当前分区交换

查询 member_fatdt0 时通过 dw_end_date 和 dw_beg_date 可以回溯到任意一天的状态。

```sql
select * from public.member_fatdt0 where dw_beg_date <= date'2011-12-01' and dw_end_date >= date'2011-12-01' order by member_id;
```

