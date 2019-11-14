---
title: Hive 安全认证相关
categories: "大数据" 
date: 2019-03-05
comments: false
toc: true
tags:
	- Hive
	- Cloudera
	- 安全
---

Hive的Kerberos接入方案，以及ACLs！

<!--more-->

# 认证

通过 Cloudera 接入Kerberos之后，Hive 默认已经完成了用户认证的配置。无论使用 hive-cli 或 beeline ，在用户连接之前均需要手工执行kinit登录一个principal。

默认情况下，通过以下方式连接：

```shell
kinit -kt /opt/idata_security/ktable/idata.keytab idata
# 通过hive-cli
hive
# 通过beeline，principal 指的是用户启动HS2的principal，而非实际用户
beeline -u "jdbc:hive2://bdnode3:10000/;principal=hive/_HOST@IDATA.RUIJIE.COM"

# 如果需要模拟任意用户，那么可以使用HDFS操作用户（组）的票据
kinit -kt /opt/idata_security/ktable/hdfs.keytab hdfs
beeline -u "jdbc:hive2://bdnode3:10000/;principal=hive/_HOST@IDATA.RUIJIE.COM;hive.server2.proxy.user={{任意用户}}"
```

## HiveServer2的认证方式

HiveServer2的认证方式通过 hive.server2.authentication 配置，默认情况下为None，即不进行认证。

HS2支持：NONE (uses plain SASL), NOSASL, KERBEROS, LDAP, PAM, CUSTOM 。配置为KERBEROS时需要指定下面两个配置：

- hive.server2.authentication.kerberos.principal
- hive.server2.authentication.kerberos.keytab

配认证之后，需要将HS2的hive.server2.enable.doAs打开！并且将hadoop.proxyuser.hive.hosts/groups配置为*！

## MetaStore的认证方式

参考下面的配置：

```xml
  <property>
    <name>hive.metastore.sasl.enabled</name>
    <value>true</value>
  </property>
  <property>
    <name>hive.metastore.kerberos.principal</name>
    <value>hive/_HOST@IDATA.RUIJIE.COM</value>
  </property>
  <property>
    <name>hive.metastore.kerberos.keytab.file</name>
    <value>hive.keytab</value>
  </property>
```

## HIVE访问HBase外表

配置HBase的security之后，如果需要由Hive访问外表，那么需要添加以下环境变量（客户端以及hs2的hive-env.sh）

```shell
HIVE_OPTS="-hiveconf hbase.security.authentication=kerberos -hiveconf hbase.master.kerberos.principal=hbase/_HOST@IDATA.RUIJIE.COM -hiveconf hbase.regionserver.kerberos.principal=hbase/_HOST@IDATA.RUIJIE.COM -hiveconf hbase.zookeeper.quorum=bdnode1,bdnode2,bdnode3" 
```

并且需要将上面的三个配置加入到HS2的白名单中：

```
hbase\.security\.authentication|hbase\.master\.kerberos\.principal|hbase\.regionserver\.kerberos\.principal|hbase\.zookeeper\.quorum
```


# 授权

默认情况下Cloudera只会完成 Hive 的认证配置，不会进行授权相关配置。

当用户操作Hive中的数据时，需要元数据和数据文件的相关权限。默认情况下：

- metastore没有ACLs控制，用户可以任意通过drop、alter等命令修改元数据；

- 数据文件处在hadoop的权限控制之下，hive client用户只能修改自己在HDFS被授权的部分；

由于操作元数据是没有ACLs控制的，因此可以认为默认情况下所有HIVE都是有最大权限的！！！

HIVE创建DB、表、分区、执行load/insert等命令时，在HDFS中创建的文件（夹）遵循hdfs的权限集成规则：

- user identity 为客户端的用户ID
- group identity 继承父目录的group信息
- 权限信息集成HDFS的umask配置

通过hive.warehouse.subdir.inherit.perms配置可以改变上面的集成策略：

- DB目录的权限继承warehouse的权限
- Table目录的权限继承DB目录的权限（或者warehouse的权限）
- 外部表的权限继承父目录的权限
- Partition目录的权限继承表目录的权限
- table文件的权限继承父文件夹的权限

在CDH6.1中。Cloudera将/user/hive/warehouse的权限设置为777t，并且hive.warehouse.subdir.inherit.perms配置为True。

当hive.warehouse.subdir.inherit.perms打开时，权限继承可能失败，但是这不会让HIVE操作失败，文件会回退到HDFS的默认继承规则。


## 用户类型

安全模式下，Hive的授权主要有以下两种场景：

1. 直接操作HDFS类型，包括：同通过 HCatalog API 操作的各类组价，如 Apache Pig、MR、Impala、Spark SQL等等。这类用户需要同时满足HDFS和metastore的用户认证
2. Hive作为SQL引擎的类型：主要指通过HiveServer2进行JDBC、ODBC操作的用户

## 授权方式

Hive的授权方式当前包括以下几种：

|认证方式|说明|
|---|---|
|[Storage Based Authorization in the Metastore Serve](https://cwiki.apache.org/confluence/display/Hive/Storage+Based+Authorization+in+the+Metastore+Server)|这种方式基于HDFS的文件权限来控制用户的行为，适合直接操作HDFS文件的用户。<br> 如果用户通过HiveServer2来访问Hive数据，那么需要配置hive.server2.enable.doAs为true。<br>这种方式可以提供Database、Table、Partition级别的控制,并且能够对所有的用户起作用。<br>该模式下，在Hive中进行授权的是metastore服务。|
|[SQL Standards Based Authorization in HiveServer2](https://cwiki.apache.org/confluence/display/Hive/SQL+Standard+Based+Hive+Authorization)|主要针对使用HiveServer2场景的权限控制，能够提供row、column的权限控制。<br> 对直接访问HDFS数据的用户，这种安全方式没有作用。<br> 这种模式中进行授权的是HiveServer2|
|Ranger & Sentry|Ranger和Sentry是两款安全相关的插件，提供动态row、column的acls，以及审计等功能。Ranger是基于策略的管理，而Sentry是基于传统的RBAC管理|
|[Legacy Mode]((https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=45876173))|hive 2.0.0之前的默认授权方式，当前已经被废弃。官方文档中介绍这是一种不完善的认证机制？|

### 基于文件系统的授权

这是一个简陋授权系统，主要包括：

- 基于HDFS的权限控制对数据库、表、分区进行授权
- metastore基于HDFS的文件（夹）权限，对元数据进行保护

涉及的配置项为：

|配置项|值|
|---|---|
|hive.metastore.pre.event.listeners|org.apache.hadoop.hive.ql.security.authorization.AuthorizationPreEventListener|
|hive.security.metastore.authorization.manager|org.apache.hadoop.hive.ql.security.authorization.StorageBasedAuthorizationProvider|
|hive.security.metastore.authenticator.manager|org.apache.hadoop.hive.ql.security.HadoopDefaultMetastoreAuthenticator|
|hive.security.metastore.authorization.auth.reads|true|


下面的例子简要说明文件系统授权的Demo：

```shell
# 通过idata用户创建一张外部表

create EXTERNAL table idata_acls ( id int, name string)
row format delimited
fields terminated by ','
LOCATION '/tmp/idata_acls';

# 上传表数据到hdfs的/tmp/idata_acls目录，修改test.txt文件权限
hdfs dfs -put test.txt /tmp/idata_acls

# 修改文件的所有组为idata:idata, 权限为600。 、
# 这时候idata用户可以正常删表、读表，使用root用户登录，执行select、drop表等操作会发生错误
hdfs dfs -chmod 600 /tmp/idata_acls/test.txt
hdfs dfs -chown idata:idata /tmp/idata_acls/test.txt
```

这种ACLs机制，有以下特点：

1. 可以保护metastore中的元数据不被误删，在无ACLs控制时任意用户都能删除metastore的元数据；
2. 需要手工调用HDFS命令，来控制DB、Table、Partition的文件夹，实现ACLs；
3. 配合HDFS的ACLs功能，能够一定程度的提高灵活性（dfs.namenode.acls.enabled配置为true）；
4. 对于Spark SQL、Impala等服务，只能通过这种方式控制权限；
5. hdfs的超级用户，有所有表权限；
6. 除了MetaStore，HCatalog同样支持该方式（可以参考应用中的文档）

### 标准SQL授权

标准SQL授权主要针对使用HS2的用户进行授权，提供细粒度的权限控制，HiveServer2在编译过程中，进行权限识别。对 Hive Cli、Spark SQL、Impala等直接通过元数据操作 HDFS 文件的client，这种授权模式不起作用。

HS2的标准SQL授权可以和metastore文件系统的授权同时使用。通常情况下 Hive 的安全控制遵循以下方式：

- 通过Kerberos提供权限认证，任意用户需要获得TGT才可使用；
- metastore 使用文件系统的授权，Impala、Spark SQL、MR 集群内部用户各自作为特权用户管理各自账户下的数据；
- HS2提供标准SQL授权，对集群外部用户实行严格的分级授权，用户不应该使用 Hive-Cli 操作HIVE；


#### 权限模型

HS2使用标准的RBAC授权，通过Role和User绑定进行授权。

通常情况下，一个用户可以关联多个Role。但是用户在执行hive查询时，只拥有当前用户的的权限。用户使用"show current roles;" 命令可以查询当前的role，通过"set role"命令可以切换到其他用户。

Hive中包括以下两个特殊Role：public 和 admin

- 所有用户都属于public角色，如果需要给所有用户赋权，那么可以将权限和public角色绑定
- 通常情况下database的管理员和admin绑定，admin用户可以创建/删除role，默认情况下用户需要通过"set role admin;"获取admin权限

在HIVE中，用户名和组名有以下特定：

- role不是大小写敏感的，user是大小写敏感的；
- 角色/用户名，可以包含任意Unicode字符（前提是使用 hive.support.quoted.identifiers 配置的转义符转义）

Hive包括以下Privileges，用户相关操作需要的权限可以参考官网的[Privileges Required for Hive Operations](https://cwiki.apache.org/confluence/display/Hive/SQL+Standard+Based+Hive+Authorization)：

|Privileges|说明|
|---|---|
|SELECT |对象的读权限|
|INSERT |对象的追加写权限|
|UPDATE |对象的update权限|
|DELETE |对象的删除权限|
|ALL PRIVILEGES |上述所有权限集合|

涉及的对象包括：

- 表
- 视图

1. table、view 的创建者默认拥有所有权限
2. 权限模型不涉及Database的权限管理，只有部分操作时会考虑DB的所有权。通过 alter database 命令可以将数据库的owner指定为用户或者组

#### 相关SQL命令

ROLE相关命令
```SQL

--- admin 相关命令
CREATE ROLE role_name;
DROP ROLE role_name;
SHOW ROLES;

--- 关联roles到用户（或者其他role）,使用WITH ADMIN OPTION配置时，用户可以将获得role转售
GRANT role_name [, role_name] ...
TO USER linqing, ROLE idata ...
[ WITH ADMIN OPTION ];

--- 收回roles，ADMIN OPTION FOR 表示回收有用户的role转售权限
REVOKE [ADMIN OPTION FOR] role_name [, role_name] ...
FROM USER linqing, ROLE idata ... ;

--- 查询用户/Role当前关联的role
SHOW ROLE GRANT (USER|ROLE) principal_name;

--- 查询所有关联到role的用户/role
SHOW PRINCIPALS role_name;

--- 普通用户相关命令
SHOW CURRENT ROLES;
SET ROLE (role_name|ALL|NONE);

```

对象相关命令
```SQL
--- 给予表/视图的相关权限
GRANT
    priv_type [, priv_type ] ...
    ON table/view table_or_view_name
    TO principal_specification [, principal_specification] ...
    [WITH GRANT OPTION];

--- 从用户上回收表/视图的相关权限
REVOKE [GRANT OPTION FOR]
    priv_type [, priv_type ] ...
    ON table/view table_or_view_name
    FROM principal_specification [, principal_specification] ... ;


--- 展示当前用户在某张表（或者所有表）上的权限
SHOW GRANT [principal_specification] ON (ALL | [TABLE] table_or_view_name);

/*
principal_specification
  : USER user
  | ROLE role
  
priv_type
  : INSERT | SELECT | UPDATE | DELETE | ALL
*/


```

#### 限制

HS2配置标准SQL授权之后，部分命令在beeline中被限制使用：

- dfs、add、delete、compile、reset 命令在权限模式下被禁用，通过配置 hive.security.command.whitelist 可以解除禁用；
- set 命令能够设定的参数会被部分限制，参考配置hive.security.authorization.sqlstd.confwhitelist 、hive.security.authorization.sqlstd.confwhitelist.append、hive.conf.restricted.list ；
- 默认情况下允许执行所有内建udf，配置hive.server2.builtin.udf.whitelist、hive.server2.builtin.udf.blacklist可以限制用户可以执行的udf；
- 只有管理员角色，才能添加、删除函数或者宏。通过hive.users.in.admin.role可以指定管理员用户；
- 无法使用 transform 函数

#### 配置项

全局配置项：

|配置项|值|说明|
|---|---|---|
|hive.server2.enable.doAs|false|禁止使用代理，所有操作通过HS2完成。<br> 但是在HS2中是以实际的Client用户ID进行Check的！！|
|hive.users.in.admin.role|hive,idata|需要配置为admin角色的用户|
|hive.security.<br>metastore.authorization.manager|org.apache.hadoop.<br>hive.ql.security.authorization.<br>MetaStoreAuthzAPIAuthorizerEmbedOnly|添加该配置后，所有访问远程metastore的authorization api 调用会被拒绝。<br>此时，HS2使用一个内嵌的metastore工作，拥有authorization api的调用权限|
|hive.security.<br>authorization.manager|org.apache.hadoop.<br>hive.ql.security.authorization.<br>plugin.sqlstd.<br>SQLStdConfOnlyAuthorizerFactory|配置所有table、view的创建者有所有的权限|

HS2服务端配置项：

|配置项|值|
|---|---|
|hive.security.authorization.manager|org.apache.hadoop.hive.ql.security.authorization.plugin.sqlstd.SQLStdHiveAuthorizerFactory|
|hive.security.authorization.enabled|true|
|hive.security.authenticator.manager|org.apache.hadoop.<br>hive.ql.security.<br>SessionStateUserAuthenticator|
|hive.metastore.uris|' '|

#### 在CDH上配置SQLstd授权

由于CDH魔改了HIVE的几个配置导致开启授权后，有两个配置hive.query.redaction.rules和hive.exec.query.redactor.hooks无法用set命令修改，当用户使用beeline连接HS2失败。

出现下面的异常：

```
19/03/07 19:21:38 [main]: WARN jdbc.HiveConnection: Failed to connect to bdnode3:10000
Error: Could not open client transport with JDBC Uri: jdbc:hive2://bdnode3:10000/;principal=hive/_HOST@IDATA.RUIJIE.COM: Failed to open new session: java.lang.IllegalArgumentException: Cannot modify hive.query.redaction.rules at runtime. It is not in list of params that are allowed to be modified at runtime (state=08S01,code=0)
Beeline version 2.1.1-cdh6.1.0 by Apache Hive
```

在HS2日志也会出现，下面的异常：

```
 Cannot modify hive.exec.query.redactor.hooks at runtime. It is not in list of params that are allowed to be modified at runtime
```

可以将hive.query.redaction.rules和hive.exec.query.redactor.hooks添加到白名单，解决该问题。在HS2的hive-site.xml配置下面的参数开启sqlstd授权。

```xml
<!-- 注意：hive.server2.enable.doAs 需要配置为false -->

<property>
    <name>hive.users.in.admin.role</name>
    <value>idata</value>
</property>
<property>
    <name>hive.security.authorization.manager</name>
    <value>org.apache.hadoop.hive.ql.security.authorization.plugin.sqlstd.SQLStdHiveAuthorizerFactory</value>
</property>
<property>
    <name>hive.security.authenticator.manager</name>
    <value>org.apache.hadoop.hive.ql.security.SessionStateUserAuthenticator</value>
</property>
<property>
    <name>hive.security.authorization.enabled</name>
    <value>true</value>
</property>
<property>
    <name>hive.security.authorization.sqlstd.confwhitelist.append</name>
    <value>hive\.exec\.query\.redactor\.hooks|hive\.query\.redaction\.rules</value>
</property>
```




# 参考

[LanguageManual+Authorization](https://cwiki.apache.org/confluence/display/Hive/LanguageManual+Authorization)

[HCatalog Authorization](https://cwiki.apache.org/confluence/display/Hive/HCatalog+Authorization)

[Setting Up HiveServer2](https://cwiki.apache.org/confluence/display/Hive/Setting+Up+HiveServer2)

[Using Hive to Run Queries on a Secure HBase Server](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/cdh_sg_hive_query_secure_hbase.html)

[HiveServer2 Security Configuration](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/cdh_sg_hiveserver2_security.html#topic_9_1)

[Hive Metastore Server Security Configuration](https://www.cloudera.com/documentation/enterprise/5-16-x/topics/cdh_sg_hive_metastore_security.html#topic_9_2)

[Permission Inheritance in+Hive](https://cwiki.apache.org/confluence/display/Hive/Permission+Inheritance+in+Hive)

[Configuration Properties](https://cwiki.apache.org/confluence/display/Hive/Configuration+Properties)