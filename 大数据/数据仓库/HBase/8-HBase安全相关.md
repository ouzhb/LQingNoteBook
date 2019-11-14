---
title: HBase 安全相关配置
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-02-25
comments: false
toc: true
tags:
	- HBase
---

HBase 安全相关配置。

<!--more-->

# Kerberos认证

HBase配置Kerberos时，需要Hadoop和Zookeeper同时配置Kerberos接入。

## Server的hbase-site.xml配置

```xml
<!-- 默认情况下是simple，表示不使用认证 -->
<property>
  <name>hbase.security.authentication</name>
  <value>kerberos</value>
</property>

<!-- 开启Kerberos的安全授权，通常接入Kerberos以后授权必须一起打开 -->
<property>
  <name>hbase.security.authorization</name>
  <value>true</value>
</property>

<!-- Master、regionserver、thrift使用的principal账号 -->
<property>	
  <name>hbase.master.kerberos.principal</name>
  <value>hbase/_HOST@IDATA.RUIJIE.COM</value>
</property>
<property>
  <name>hbase.regionserver.kerberos.principal</name>
  <value>hbase/_HOST@IDATA.RUIJIE.COM</value>
</property>
<property>
  <name>hbase.thrift.kerberos.principal</name>
  <value>hbase/_HOST@IDATA.RUIJIE.COM</value>
</property>

<!-- Master、regionserver、thrift不同组件使用的keytable文件 -->
<property>
  <name>hbase.master.keytab.file</name>
  <value>hbase.keytab</value>
</property>
<property>
  <name>hbase.regionserver.keytab.file</name>
  <value>hbase.keytab</value>
</property>
<property>
  <name>hbase.thrift.keytab.file</name>
  <value>/etc/hbase/conf/hbase.keytab</value>
</property>


<!-- 配置相关的coprocessor -->
<property>
  <name>hbase.coprocessor.master.classes</name>
  <value>org.apache.hadoop.hbase.security.access.AccessControllerorg.apache.hadoop.hbase.security.visibility.VisibilityController</value>
</property>

<property>
  <name>hbase.coprocessor.region.classes</name>
  <value>org.apache.hadoop.hbase.security.access.AccessController,org.apache.hadoop.hbase.security.visibility.VisibilityController,org.apache.hadoop.hbase.security.token.TokenProvider,org.apache.hadoop.hbase.security.access.SecureBulkLoadEndpoint</value>
</property>

```

## 配置jaas.conf

在Apache hbase版本的配置文件中，可以在hbase-env.sh文件中指定jaas.conf文件的地址，配置该文件主要是为了让HBase通过特权账户访问Zookeeper。

```shell
export HBASE_OPTS="-Djava.security.auth.login.config=$HBASE_CONFIG_DIR/jaas.conf"

```

## Client相关配置

当HBase配置Kerberos认证之后，Client需要以下额外配置：

```xml

<property>
  <name>hbase.security.authentication</name>
  <value>kerberos</value>
</property>

<!-- 以下两个参数需要2.2.0以上版本才可生效，CDH6.1.0集成的2.1.0刚好差一个版本 -->
<property>
  <name>hbase.client.keytab.file</name>
  <value>/local/path/to/client/keytab</value>
</property>

<property>
  <name>hbase.client.keytab.principal</name>
  <value>foo@EXAMPLE.COM</value>
</property>

```

## Thrift、REST 访问HBase

略

# 数据安全

对接Kerberos后，可以针对用户需要，为HBase开启不同的数据安全策略（必须配置）。

HBase提供以下安全策略：

- 基于 RBAC 的访问控制，包括：命名空间、表、列族、列的ACL
- 基于可见性标签的 Cell 访问控制
- HFiles 和 WAL 的数据透明加密

## Tag机制

HBase的部分安全功能需要 HFile V3 格式的支持。

```xml
<property>
  <name>hfile.format.version</name>
  <value>3</value>
</property>
```

Tag 是 HFile v3 的新特性。 Tag是cell的metadata资源，被直接存储在HFiles中，一个 Cell 可以包括一个或者多个 tag 。HBase的cell级别ACL，以及visibility labels 需要通过 Tag 能力支持。

Tag可以在列族级别指定压缩的格式（当WAL使用加密时，不支持对Tag进行压缩）。用户读取Cell时，Coprocessors 会在RPC传输层分离Tag，因此对用户来说Tag是不可见的。

## ACLs

HBase 提供用户/用户组的权限认证功能。

由于HBase本身不保存user-groups映射，所有的映射关系来自Hadoop，通常情况下在HBase的RBAC体系中，Group被作为Role。

### ACLs相关配置
HBase 的ACL控制通过 Coprocessors 实现，大部分ACL逻辑实现在org.apache.hadoop.hbase.security.access.AccessController类中。

配置HBase 的ACL包括以下配置：

```xml

<property>
  <name>hbase.security.authorization</name>
  <value>true</value>
</property>
<property>
  <name>hbase.coprocessor.region.classes</name>
  <value>org.apache.hadoop.hbase.security.access.AccessController, org.apache.hadoop.hbase.security.token.TokenProvider</value>
</property>
<property>
  <name>hbase.coprocessor.master.classes</name>
  <value>org.apache.hadoop.hbase.security.access.AccessController</value>
</property>
<property>
  <name>hbase.coprocessor.regionserver.classes</name>
  <value>org.apache.hadoop.hbase.security.access.AccessController</value>
</property>

<!-- 启用X权限的相关配置 -->
<property>
  <name>hbase.security.exec.permission.checks</name>
  <value>true</value>
</property>

```

### 权限类型
HBase提供的权限包括：

|权限|说明|备注|
|---|---|---|
|Read|读权限||
|Write|写权限||
|Execute|执行权限，即执行相关的Coprocessors的权限||
|Create|创建表、删除表的权限||
|Admin|管理员权限，包括：表的balance操作、region assin、grant、revoke等操作|由于Admin权限能够给自己赋权，因此该权限是最高权限|

### 权限范围
包括以下权限范围：

|Scope|说明|备注|
|---|---|---|
|Superuser |HBase中的超级用户，默认情况下，启动用户hbase是超级用户。HBase中可以配置多个超级用户。超级用户有一切权限|hbase.superuser配置项可以指定超级用户|
|global|整个HBase空间的权限范围||
|namespaces|命名空间范围||
|tables|表级权限范围||
|ColumnFamily |列族级权限范围||

### 常用权限命令

```SQL
grant 'user', 'RWXCA', 'TABLE', 'CF', 'CQ'

-- 为admin用户组提供global权限
grant '@admin', 'RWXCA'

-- 为用户提供命名空间权限
grant 'root','RWXCA','@my-NS'

-- 为用户提供表级权限
grant 'root','RW','my-table'

-- 为用户提供列族级权限
grant 'root', 'RW', 'my-table', 'cf1'

-- 为用户提供列权限
grant 'root', 'RW', 'my-table', 'cf2','A'

```

```SQL
/***
需要注意的是revoke操作会一次回收，用户（组）在某个scope内的所有权限。当前好像没有办法只回收部分权限
***/
revoke 'user', '@<namespace>', 'table','cf','qualifier'

```

```SQL
-- 打印某个范围内的所有权限信息
user_permission '<scope>'

/***
在HBase中所有权限信息保存在，hbase:acl表中，可以通过超级用户查看
***/
scan 'hbase:acl'
```

### Cell/Row级别的权限

Cell/Row级别的权限控制，需要而外配置以下参数：

```xml

<!-- Cell权限需要启动下面两个配置 -->
<property>
  <name>hfile.format.version</name>
  <value>3</value>
</property>

<property>
  <name>hbase.security.access.early_out</name>
  <value>false</value>
</property>

```

通过以下命令提供Cell权限：

```SQL
grant 'table', {'user'=>'RW', ... }, { 筛选条件 }

-- 提供root、admin用户在linqing:test，表中的相关行的权限
grant 'linqing:test',{'root'=>'RW','admin'=>'W'},{FILTER => "(PrefixFilter('roww'))"}

-- 收回root用户在linqing:test，表中的相关行的权限
grant 'linqing:test',{'root'=>''},{FILTER => "(PrefixFilter('roww'))"}
```

PS: 关于筛选条件可以参考[官网](https://hbase.apache.org/book.html#individualfiltersyntax)


### 可视化标签

Cell权限控制是通过可视化标签实现的。 通常情况下可视化标签这个feature，可以作为HBase的视图特性！

可视化标签只需要VisibilityController类就可以运行！不需要配置AccessController

通过标签，可以实现以下效果：

在A用户有整张表的读权限是，使某些Cell对A来说不可见。

```SQL
-- 创建一个标签
add_labels 'linqing'
-- 将表的cf1，打上可见性标签
set_visibility 'linqing:test','linqing',{COLUMN =>'cf1'}

/****

这时对任意一个，没有标签授权的用户，linqing:test:cf1 都不可见

****/

-- 将标签分配给用户，linqing:test:cf1 可见
set_auths 'root','linqing'
-- 清除用户对应标签
clear_auths 'root','linqing'
-- 查看当前用户的标签
get_auths 'root'
-- 展示指定可是化标签的Cell，当cell没有标签时也可以展示
scan 'linqing:test',{ AUTHORIZATIONS =>['linqing']}
```

PS：HBase可以自定义标签算法，[参考](https://hbase.apache.org/book.html#_implementing_your_own_visibility_label_algorithm)


# 参考

[How-to: Enable User Authentication and Authorization in Apache HBase](https://blog.cloudera.com/blog/2012/09/understanding-user-authentication-and-authorization-in-apache-hbase/)

[权限矩阵](https://hbase.apache.org/book.html#appendix_acl_matrix)

[Apache HBase Cell Level Security, Part 1](https://developer.ibm.com/hadoop/2015/11/10/apache-hbase-cell-level-security-part-1/)

[Apache HBase Cell Level Security, Part 2](https://developer.ibm.com/hadoop/2015/11/25/apache-hbase-cell-level-security-part-2/)