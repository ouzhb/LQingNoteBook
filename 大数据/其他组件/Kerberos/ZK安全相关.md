---
title: Zookeeper 安全认证
categories: "大数据" 
date: 2019-03-01
comments: false
toc: true
tags:
	- Zookeeper
	- 安全
---

简要说明Zookeeper对接Kerberos的过程，以及在Zookeeper中使用ACL功能的方案。

<!--more-->

# Kerberos认证

Zookeeper 支持SASL框架，能够支持Kerberos、DIGEST-MD5等认证机制。


下面简要说明，Cloudera给出Zoookeeper集成Kerberos的配置方法：

## 修改zoo.cfg
安装完成KDC后，修改 zoo.cfg 配置文件,修改添加以下内容：

```properties
# 移除Principal中的Realm和host信息
kerberos.removeHostFromPrincipal=true
kerberos.removeRealmFromPrincipal=true

# SASL插件
authProvider.1=org.apache.zookeeper.server.auth.SASLAuthenticationProvider		

# Zookeeper 节点使用 Principal _HOST表示zk server所在的节点主机名
quorum.auth.kerberos.servicePrincipal=zookeeper/_HOST
# 配置Zk几点之间使用SASL认证
quorum.auth.learnerRequireSasl=true		
quorum.auth.serverRequireSasl=true
```
## 创建Kerberos账号

为每个zookeeper节点创建对应的krb账号，账号名为zookeeper/_HOST。 导出keytab文件到各个节点目录。

## 创建jaas.conf文件

在ZK的配置文件目录创建，jaas.conf文件。在启动Zookeeper进程时指定环境变量，JVMFLAGS="-Djava.security.auth.login.config=/etc/zookeeper/conf/jaas.conf"。 


```
Server {
  com.sun.security.auth.module.Krb5LoginModule required	
  useKeyTab=true	
  keyTab="zookeeper.keytab"	 #按照实际情况修改
  storeKey=true	
  useTicketCache=false	
  principal="zookeeper/bdnode1@IDATA.RUIJIE.COM";  #按照实际情况修改
};
	
QuorumServer {
  com.sun.security.auth.module.Krb5LoginModule required
  useKeyTab=true
  keyTab="zookeeper.keytab"
  storeKey=true
  useTicketCache=false
  principal="zookeeper/bdnode1@IDATA.RUIJIE.COM";
};
				
QuorumLearner {	
  com.sun.security.auth.module.Krb5LoginModule required	
  useKeyTab=true	
  keyTab="zookeeper.keytab"	
  storeKey=true	
  useTicketCache=false	
  principal="zookeeper/bdnode1@IDATA.RUIJIE.COM";
};
```

# Client连接ZK

客户端连接Zookeeper时，需要在/etc/zookeeper/conf目录下创建以下两个文件：

```
# 创建jaas.conf

Client {
  com.sun.security.auth.module.Krb5LoginModule required
  ticketCache="/tmp/krb5cc_0"
  useTicketCache=true;
};

# 创建java.env

export JVMFLAGS="-Djava.security.auth.login.config=/etc/zookeeper/conf/jaas.conf"

```

# ZooKeeper Access Control

Zookeeper官方文档有针对ACLs章节的相关讨论，下面简单终结。

Zookeeper 的权限包括：CREATE、READ、WRITE、DELETE、ADMIN。

Zookeeper 的身份的认证方式：

|认证方式|说明|
|---|---|
|world|表示所有客户端都能够通过认证|
|auth|表示任何经过身份验证的用户，当使用Kerberos时表示对应的principal账户通过认证|
|digest|用户名/密码方式通过认证|
|host|客户端主机名通过认证|
|ip|客户端IP通过认证|

常用的ACLs命令
```shell
# 获取当前Node的权限信息
getAcl path

# 指定当前Node的权限，使用Kerberos时，参考下面的命令，表示给当前用户所有权限
setAcl /test auth::cdrwa

# 给任意用户只读权限
setAcl /test world:anyone:r

```



# 参考
[Hardening Apache ZooKeeper Security: SASL Quorum Peer Mutual Authentication and Authorization](https://blog.cloudera.com/blog/2017/01/hardening-apache-zookeeper-security-sasl-quorum-peer-mutual-authentication-and-authorization/)

[ZooKeeper and SASL](https://cwiki.apache.org/confluence/display/ZOOKEEPER/ZooKeeper+and+SASL)

[ZooKeeper Authentication](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/cdh_sg_zookeeper_security.html)

[jaas.conf参数说明](https://docs.oracle.com/javase/7/docs/jre/api/security/jaas/spec/com/sun/security/auth/module/Krb5LoginModule.html)


[ZooKeeperAccessControl](https://zookeeper.apache.org/doc/r3.1.2/zookeeperProgrammers.html#sc_ZooKeeperAccessControl)


