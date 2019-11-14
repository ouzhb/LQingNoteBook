---
title: Impala 安装部署
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-03-11
comments: false
toc: true
tags:
	- Cloudera
	- Impala
---

关于 Impala 认证、鉴权的一些相关说明

<!--more-->

# 概述

Cloudera 在CDH的用户文档中有相关Impala安全方案，包括：认证、授权、审计。从官方说明来看，授权功能需要对接Cloudera Sentry，而审计功能需要对接付费产品Cloudera Navigator。

官方文档地址：

[Enabling Kerberos Authentication for Impala](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/impala_kerberos.html#kerberos)

[Enabling Sentry Authorization for Impala](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/impala_authorization.html#authorization)

[Auditing Impala Operations](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/impala_auditing.html#auditing)

# 认证

## 配置Kerberos

通过Cloudera Manager导入Kerberos时，集群已经同步配置了Impala的相关配置。

Kerberos涉及到的配置，只包括下面三个：

```shell
# Cloudera 将下面的配置自动更新到impala_conf的state_store_flags、impalad_flags、catalogserver_flags中

-kerberos_reinit_interval=60
-principal=impala/{ 服务运行节点 }@IDATA.RUIJIE.COM
-keytab_file=/var/run/cloudera-scm-agent/process/{ service—_process_dir }/impala.keytab

```

当为Impalad配置proxy时，需要需要额外生成VIP使用的principal账号，并且需要在impalad的启动配置项里指定vip的principal账号

```
# 通过Cloudera页面配置Impala Daemons Load Balancer后，Cloudera在启动服务时会为用户自动创建vip节点使用的principal账号
	
--principal=impala/cdh.vip@IDATA.RUIJIE.COM	
--be_principal=impala/{ 服务运行节点 }@IDATA.RUIJIE.COM

```


# 参考

[Impala Security Overview](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/impala_security.html)


