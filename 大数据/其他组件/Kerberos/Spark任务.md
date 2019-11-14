---
title: 在Kerberos环境中运行Spark On Yarn
categories: "大数据" 
date: 2019-03-04
comments: false
toc: true
tags:
	- Yarn
	- Spark
	- 安全
---

Kerberos接入的场景下，提交spark任务。

<!--more-->

# Yarn

当Hadoop接入Kerberos之后，CDH对向Yarn的提交任务的用户进行了限制：

1. 不能是root用户（参考：hadoop-yarn-project\hadoop-yarn\hadoop-yarn-server\hadoop-yarn-server-nodemanager\src\main\native\container-executor\impl\container-executor.c）
2. 用户的id必须大于min.user.id
3. 用户不能再黑名单（banned.users）中

如果用户需要使用小于min.user.id的账号提交yarn任务（比如impala、hue、hive等账号），那么需要将用户加入到allowed.system.users配置中

# Spark

对Spark任务来说，在yarn上运行时可以自动获取tgt，并自动刷新票据。

通过以下配置可以调整Kerberos登录参数：

|参数名称|说明|备注|
|---|---|---|
|spark.yarn.keytab|使用的keytab文件|可以在spark-submit中通过--principal指定|
|spark.yarn.principal|kerberos指定的用户名|可以在spark-submit中通过--keytab指定|
|spark.yarn.access.hadoopFileSystems|需要访问Hadoop，可以指定多个|默认是none|
|spark.yarn.kerberos.relogin.period|检查票据失效的周期|默认是1m|
|spark.security.credentials.${service}.enabled|访问Hive、HBase等服务时是否通过Kerberos用户访问|默认是true|

# 参考

[security kerberos](https://spark.apache.org/docs/latest/security.html#kerberos)
[running on yarn](https://spark.apache.org/docs/latest/running-on-yarn.html#kerberos)