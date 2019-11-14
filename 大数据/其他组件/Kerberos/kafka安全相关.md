---
title: Kafka 安全认证
categories: "大数据" 
date: 2019-03-12
comments: false
toc: true
tags:
	- Kafka
	- 安全
---

简要说明Zookeeper对接Kerberos的过程，以及在Zookeeper中使用ACL功能的方案。

<!--more-->

# 认证

在 Kafka 0.9.0 以后的版本，添加了安全相关的功能，包括以下：

- 支持SSL和SASL两种认证方式，其中SASL框架支持 GSSAPI 、PLAIN 、SCRAM-SHA-256 、OAUTHBEARER 等多种认证引擎。认证发生在：client-broker，broker之间，broker-zookeeper。
- 支持使用SSL进行数据传输加密。
- 支持client对topic的读写授权。
- 支持授权服务可插拔，并且支持与外部授权服务（比如sentry）的集成。

## 对接Kerberos

### 服务端配置

Cloudera导入Kerberos以后，CM自动为Kafka创建了principal账号，并且生成了jaas.conf。

```
KafkaServer {
   com.sun.security.auth.module.Krb5LoginModule required
   doNotPrompt=true
   useKeyTab=true
   storeKey=true
   keyTab="{ process-config }/kafka.keytab"
   principal="kafka/{ broker-host }@IDATA.RUIJIE.COM";
};


Client {
   com.sun.security.auth.module.Krb5LoginModule required
   useKeyTab=true
   storeKey=true
   keyTab="{ process-config }/kafka.keytab"
   principal="kafka/{ broker-host }@IDATA.RUIJIE.COM";
};
```

上述配置中，包括Client和KafkaServer两个context：

- KafkaServer：主要用于broker之间，以及client和broker之间的认证；
- Client：主要用于broker和zookeeper之间的认证；
  
安装Kafka的官方文档，Kafka在进行SASL时使用context名称为KafkaServer，在这个配置下可以定义多个不同的listenerName、同一个listenerName下可以定义多个不同的认证机制（PS:这个部分的内容[官网](http://kafka.apache.org/documentation/#security_jaas_broker)描述的不是太清晰，没怎么看懂！）。

官方文档中，kafka.properties涉及到的配置项包括：

```

listeners=SASL_PLAINTEXT://host.name:port           # broker使用的listeners名称
security.inter.broker.protocol=SASL_PLAINTEXT       # broker之间使用的认证协议
sasl.kerberos.service.name=kafka            # broker使用的kerberos Principal名称

sasl.mechanism.inter.broker.protocol=GSSAPI # 默认配置就是GSSAPI
sasl.enabled.mechanisms=GSSAPI              # 默认配置就是GSSAPI

```
涉及到的JVM环境变量包括：

```

-Djava.security.krb5.conf=/etc/kafka/krb5.conf
-Djava.security.auth.login.config=/etc/kafka/kafka_server_jaas.conf

```


### 客户端配置

客户端可以使用两种方式配置jaas的参数：

- 通过配置文件jaas.conf指定认证信息，最后通过KAFKA_OPTS="${KAFKA_OPTS} -Djava.security.auth.login.config=jaas.conf" 指定jaas.conf参数。

```
KafkaClient {
    com.sun.security.auth.module.Krb5LoginModule required
    useKeyTab=true
    storeKey=true
    keyTab="/etc/security/keytabs/kafka_client.keytab"
    principal="kafka-client-1@EXAMPLE.COM";
};
```

- 通过sasl.jaas.config在producer/consumer的properties配置文件指定，该方式优先级高于使用jaas.conf指定，通过这种方式可以在同一个JVM中指定多个sasl配置；

```
sasl.jaas.config=com.sun.security.auth.module.Krb5LoginModule required \
    useKeyTab=true \
    storeKey=true  \
    keyTab="/etc/security/keytabs/kafka_client.keytab" \
    principal="kafka-client-1@EXAMPLE.COM";
```

其他client配置包括：

```
security.protocol=SASL_PLAINTEXT
sasl.mechanism=GSSAPI
sasl.kerberos.service.name=kafka
```

## 其他关于认证的配置

### 启用多种认证配置

Kafka支持同时启用多种认证配置，下面的示例中jaas.conf同时指定了Kerberos和秘钥方式的认证：

```
        KafkaServer {
            com.sun.security.auth.module.Krb5LoginModule required
            useKeyTab=true
            storeKey=true
            keyTab="/etc/security/keytabs/kafka_server.keytab"
            principal="kafka/kafka1.hostname.com@EXAMPLE.COM";

            org.apache.kafka.common.security.plain.PlainLoginModule required
            username="admin"
            password="admin-secret"
            user_admin="admin-secret"
            user_alice="alice-secret";
        };
```
用户同时在kafka.properties中指定：

- sasl.enabled.mechanisms=GSSAPI,PLAIN           # 启动kerberos和密码两种认证方式
- security.inter.broker.protocol=SASL_PLAINTEXT 
- sasl.mechanism.inter.broker.protocol=GSSAPI    # broker内部指定使用Kerberos认证

PS:Kafka甚至支持client连接同一个集群的不同Broker时，不同Broker采用不同的认证方式！

# 授权

## 授权配置

Kafka 授权模块是通过插件方式实现，支持为以下ACL方式：

```
Principal P is [Allowed/Denied] Operation O From Host H on any Resource R matching ResourcePattern RP。

Principal：表示用户
Operation：表示用户操作，包括：Read, Write, Create, Delete, Alter, Describe, ClusterAction, All
Host： 表示发起操作的主机
ResourcePattern： 表示一组正则匹配的资源
```

ACL涉及到的kafka.properties配置包括：

```
authorizer.class.name=kafka.security.auth.SimpleAclAuthorizer # 指定认证插件
allow.everyone.if.no.acl.found=false        # 指定到任意未关联ACL的topic，是否允许任意人访问
super.users=User:idata;User:kafka           # 指定超级用户
sasl.kerberos.principal.to.local.rules      # 指定Kerberos的principal匹配规则，使用默认配置即可

```

## ACLs命令

通过Kafka自带的kafka-acls.sh，可以进行权限配置。


# CDH中启用Kerberos认证

CDH中开启Kerberos时，CM中已经自动生成了JAAS配。kafka在Zookeeper上的元数据依然是全局访问，用户需要进行手工迁移。

Kafka需要进行以下设置，在CDH上开启Kerberos和ACL认证：

0. CM上导入Kerberos完成；
1. kafka.properties安全阀追加以下配置：
```
zookeeper.set.acl=true
authorizer.class.name=kafka.security.auth.SimpleAclAuthorizer   #启用sentry时，该配置走的是org.apache.sentry.kafka.authorizer.SentryKafkaAuthorizer
super.users=User:idata;User:kafka               # 该配置在CM中有独立配置，但是在启用Sentry时才生效的。需要直接通过安全阀配置。
```
2. 创建client.properties 配置文件，该配置文件主要是给测试工具使用的。代码中应该直接将配置项嵌入代码中
```
security.protocol=SASL_PLAINTEXT
sasl.mechanism=GSSAPI
sasl.kerberos.service.name=kafka
sasl.jaas.config=com.sun.security.auth.module.Krb5LoginModule required useTicketCache=true;  #表示使用手工kinit的方式获取权限，实际在代码中应该通过keytab的方式
```

4. 使用kafka账号将元数据迁移为安全模式，迁移后只有通过kafka账号能够创建、删除topic。其他账号均不允许！

```
kinit -kt /opt/idata_security/ktable/kafka.keytab kafka@IDATA.RUIJIE.COM
export KAFKA_OPTS="${KAFKA_OPTS} -Djava.security.auth.login.config=/opt/idata_security/kafka_jaas.conf"
/opt/cloudera/parcels/CDH/lib/kafka/bin/zookeeper-security-migration.sh --zookeeper.acl=secure --zookeeper.connect=bdnode1:2181
zookeeper-client -server bdnode1:2181,bdnode2:2181,bdnode3:2181 setAcl / world:anyone:cdrwa
```

# 参考
[ Kafka Security ](http://kafka.apache.org/documentation/#security)
[ Command Line Interface ](http://kafka.apache.org/documentation/#security_authz_cli)

