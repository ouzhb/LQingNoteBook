---
title: Impala 安装部署
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-02-25
comments: false
toc: true
tags:
	- Cloudera
	- Impala
---

关于Impala安装部署的一些说明。

<!--more-->

# 安装方案

Impala作为Cloudera主导开发的一个开源SQL工具，在github上有Apache/impala,Cloudera/impala两个代码仓库。从提交的活跃度上看，Cloudera依然是Impala的主导者。


当前,Impala支持运行在有以下这几个系统上:

1. [CDH](https://www.cloudera.com/documentation/enterprise/latest/topics/impala.html)
2. [MapR](https://mapr.com/docs/archive/mapr50/Installing-Impala-on-MapR_26982569.html)
3. [AWS S3](https://impala.apache.org/docs/build/html/topics/impala_s3.html)

比较令人震惊的是：官方没有任何声明 Impala 能够运行在 Apache Hadoop 上，虽然谁都知道肯定没有问题。Cloudera 至今没有提供在 Apache Hadoop 版本的安装包，也就是说Impala on Apache Hadoop 官方无法保证兼容性完全没有问题。

目前，Impala有以下几种部署方案：

1. 通过Cloudera Manager的Parcels包安装；
2. 通过Cloudera 提供的RPM包安装；
3. 通过代码编译安装；

通过上面三种方案，安装的Impala连接Hive、Hadoop、HBase的Client均是CDH版本。

# Requirements

官网给出了[Requirements](https://impala.apache.org/docs/build/html/topics/impala_prereqs.html#prereqs),包括以下几点：

- Hive metastore service 依赖
- 指明依赖Oracle JDK，其他版本的JDK可能引发ISSUSE
- 2.2以上的版本需要运行在 SSSE3 指令集的CPU上
- 官方推荐单点Impalad内存128GB以上（呵呵）
- 官方不推荐使用root运行Impala，原因是会影响性能

