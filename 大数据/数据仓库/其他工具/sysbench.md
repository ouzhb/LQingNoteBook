---
title: 数据库调研笔记 -- Sysbench
categories: "笔记"
date: 2019-09-30
comments: true
toc: true
tags:
	- Sysbench
    - 数据库
---

Sysbench 性能测试工具

<!--more-->

# Sysbench

Sysbench是基于LuaJIT的可编写脚本的多线程基准测试工具，提供系统软硬件层面的性能基准测试，包括;

- 数据库基准测试
- 文件系统基准测试
- CPU性能基准测试
- 内存性能基准测试
- 线程调度基准测试
- POSIX信号量基准测试

## 安装部署

```shell
curl -s https://packagecloud.io/install/repositories/akopytov/sysbench/script.rpm.sh | sudo bash
sudo yum -y install sysbench
```


# 性能测试

## Postgresql


测试命令：

```
sysbench oltp_insert.lua --time=60 --percentile=99 --pgsql-host=172.24.9.11 --pgsql-port=9999 --pgsql-user=root --pgsql-password=rjbigdata --pgsql-db=sysbench --threads=20 --table-size=10000000 --db-driver=pgsql --tables=20 prepare

sysbench oltp_insert.lua --time=60 --percentile=99 --pgsql-host=172.24.9.11 --pgsql-port=9999 --pgsql-user=root --pgsql-password=rjbigdata --pgsql-db=sysbench --threads=20 --table-size=6000000 --db-driver=pgsql --tables=20 run

sysbench oltp_read_only.lua --time=60 --percentile=99 --pgsql-host=172.24.9.11 --pgsql-port=9999 --pgsql-user=root --pgsql-password=rjbigdata --pgsql-db=sysbench --threads=20 --table-size=6000000 --db-driver=pgsql --tables=20 run

sysbench oltp_read_write.lua --time=60 --percentile=99 --pgsql-host=172.24.9.11 --pgsql-port=9999 --pgsql-user=root --pgsql-password=rjbigdata --pgsql-db=sysbench --threads=20 --table-size=100000 --db-driver=pgsql --tables=20 run

sysbench oltp_insert.lua --pgsql-host=172.24.9.11 --pgsql-port=9999 --pgsql-user=root --pgsql-password=rjbigdata --pgsql-db=sysbench --threads=20  --db-driver=pgsql --tables=20 cleanup

```


硬件配置： 128GB + 32CPU + SSD + 万兆网络

pg配置： shared_buffers = 32GB（huge_pages = on ），其他默认

测试条件：

    - 预写20张表，每张1000w记录

|Case|insert|read|混合|配置|
|----|----|----|----|----|
|1|20线程：24885</br>32线程：36893</br>64线程：58137</br>96线程：66747|20线程：105781</br>96线程：230311|20线程：54485</br>96线程：58404|单机PG |
|2|20线程：7249</br>32线程：8752</br>64线程：9991</br>96线程：10545|20线程：35531</br>96线程：72351| 略：|2PG+pgpool |


# 参考

[官方GitHub](https://github.com/akopytov/sysbench)