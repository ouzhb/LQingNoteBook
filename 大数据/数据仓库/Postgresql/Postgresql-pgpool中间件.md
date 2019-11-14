---
title: 数据库调研笔记 -- PostgreSQL
categories: "笔记"
date: 2019-09-29
comments: true
toc: true
tags:
	- PostgreSQL
    - 数据库
    - pgpool-II
---


PostgreSQL 调研笔记


<!--more-->

# pgpool-II

提供以下功能：

- 连接池
- 复制
- 负载均衡（读性能应该和 PostgreSQL 服务器的数量成正比，适用于大用户并发查询的场景）

## 安装
```shell
yum -y install http://www.pgpool.net/yum/rpms/4.1/redhat/rhel-7-x86_64/pgpool-II-release-4.1-1.noarch.rpm
yum -y install pgpool-II-pg11-debuginfo pgpool-II-pg11 pgpool-II-pg11-devel pgpool-II-pg11-extensions

systemctl enable pgpool.service
systemctl start pgpool.service 

```

## 配置

### HBA配置

如果开启pgpool.conf中的 enable_pool_hba = on，需要额外配置 pool_hba.conf 文件，同时还要将用户密码的MD5写入pool_passwd文件中(pgpool.conf文件中会配置该文件的文件名pool_passwd = 'pool_passwd')。

```
# 实际测试中，pool_passwd文件中的密码要和pg中的用户密码一致
pg_md5 -m -u root rjbigdata

# root用户允许使用MD5方式连接
host    all             root            all                     md5

```

### 后端配置

```
# 以下是2节点后端配置
backend_hostname0 = 'node11'
backend_port0 = 5432
backend_weight0 = 1
backend_data_directory0 = '/data/data_ssd/pd_data/'
backend_flag0 = 'ALLOW_TO_FAILOVER'
backend_application_name0 = 'server0'

backend_hostname1 = 'node12'
backend_port1 = 5432
backend_weight1 = 1
backend_data_directory1 = '/data/data_ssd/pd_data/'
backend_flag1 = 'ALLOW_TO_FAILOVER'
backend_application_name1 = 'server1'
```




# 参考

[中文手册](https://www.pgpool.net/docs/pgpool-II-3.5.4/doc/pgpool-zh_cn.html#Whatis)