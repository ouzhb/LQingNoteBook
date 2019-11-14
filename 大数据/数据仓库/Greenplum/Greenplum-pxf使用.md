---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-11-07
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->


# PXF

PXF 是 Greenplum 自带的外表连接插件，可以用来连接包括：Hive、HDFS、HBase、S3、JDBC 等外部数据源。 

PXF 运行在每个 Segment 主机上，可以直接连接外部数据源获取数据，具有：谓词下推、列式查询等特性。

PXF 基于JAVA，使用时需要在所有Greenplum集群上安装jdk，其安装目录位于$GPHOME/pxf。


## 配置文件

### Conf配置目录

当用户初始化PXF时，初始化进程会在 $PXF_CONF 目录下创建用户配置目录。

用户配置目录以 /usr/local/greenplum-db/pxf 下的文件为模板，用户应该在 $PXF_CONF 目录中修改配置文件。

用户需要关注 $PXF_CONF/conf 目录中的三个配置文件：

- pxf-env.sh：定义 PXF 使用的环境变量
- pxf-log4j.properties：日志文件
- pxf-profiles.xml：自定义profiles

```
profiles 配置是用来定义：数据格式、依赖jar等内容的，可以认为一个profiles 是 pxf 和外部数据源之间的连接器。

通常一个数据源有多种连接器，PXF 已经预定义了一些 profiles 配置（参考$GPHOME/pxf/conf/pxf-profiles-default.xml文件）

Hadoop作为外部数据源时，数据格式和数据源（HIVE、HBASE、HDFS）对应的 profiles 参考：

[Connectors, Data Formats, and Profiles](https://gpdb.docs.pivotal.io/6-1/pxf/access_hdfs.html)

```

### 数据源配置

PXF 中将外部数据源定义为Server，用户在创建外部表时可以指定不同的Server，当不指定Server时从默认数据源（default server）拉取数据。

Server的配置存放在 $PXF_CONF/servers/{server_name}/ 目录中，每一个Server表示一种外部数据源。

Server的配置文件取决于类型，可能包含多种配置文件。PXF 还允许不同Greenplum用户连接数据源时，使用不同的配置用户可以在 $PXF_CONF/servers/{server_name}/ 中创建 {greenplum_user_name}-user.xml 文件，根据需要编辑该用户使用参数（比如，**身份信息**和**其他配置**）。

```xml

<!-- 使用JDBC作为外部存储时，在用户配置中定义JDBC的用户信息 -->

<configuration>
    <property>
        <name>jdbc.user</name>
        <value>pguser1</value>
    </property>
    <property>
        <name>jdbc.password</name>
        <value>changeme</value>
    </property>
</configuration>
```



## 建表语句

```sql

-- 建表语句
-- pxf协议中，包含以下参数：
--      PROFILE         ：指定了配置文件地址
--      SERVER          ：配置文件中定义的数据源配置
--      path-to-data    ：数据地址，HDFS上的地址或者HIVE上的表名

CREATE [WRITABLE] EXTERNAL TABLE <table_name>
        ( <column_name> <data_type> [, ...] | LIKE <other_table> )
LOCATION('pxf://<path-to-data>?PROFILE=<profile_name>[&SERVER=<server_name>][&<custom-option>=<value>[...]]')
FORMAT '[TEXT|CSV|CUSTOM]' (<formatting-properties>);

```

## QuickStart

以下Demo展示了Greenplum访问HDFS上的Text文本数据：


```shell

# 所有节点指定PXF配置目录
echo "export PXF_CONF=/home/gpadmin/conf/pxf_conf" >> ~/.bashrc 
source ~/.bashrc

# 在整个集群中初始化pxf服务，pxf会在 $PXF_CONF 指定的路径下生成一系列配置文件
$GPHOME/pxf/bin/pxf cluster init

# Copy Hadoop环境的以下文件到$PXF_CONF/servers/default目录
#   core-site.xml
#   hdfs-site.xml
#   mapred-site.xml
#   yarn-site.xml

# 当Hadoop使用Kerberos认证时，执行以下操作
# 
#   1. 安装kerberos客户端，yum -y install  krb5-libs krb5-workstation
#   2. 配置krb5.conf
#   3. copy keytab文件到所有segment机器
#   4. 修改$PXF_CONF/conf/pxf-env.sh中的以下几个配置项：
#           PXF_KEYTAB：            使用的keytab
#           PXF_PRINCIPAL：         认证的账号
#           PXF_USER_IMPERSONATION：建议改为false

# 执行以下命令同步配置到所有segment节点
#   
$GPHOME/pxf/bin/pxf cluster sync
$GPHOME/pxf/bin/pxf cluster start
```

```sql
-- HDFS 上文本内容如下，文件位于/tmp/pxf_hdfs_simple.txt目录
--
-- Prague,Jan,101,4875.33
-- Rome,Mar,87,1557.39
-- Bangalore,May,317,8936.99
-- Beijing,Jul,411,11600.67

-- 创建Greenplum用户用来访问外部表，这里建议和PXF_PRINCIPAL中定义一致，否则需要配置 PXF_USER_IMPERSONATION
CREATE ROLE idata LOGIN REPLICATION CREATEDB CREATEEXTTABLE PASSWORD 'idata';

-- 创建db，并且注册pxf插件
CREATE DATABASE idata;
CREATE EXTENSION pxf;

-- 为用户赋pxf协议权限
GRANT INSERT ON PROTOCOL pxf TO idata; 
GRANT SELECT ON PROTOCOL pxf TO idata; 

-- 建表语句
CREATE EXTERNAL TABLE pxf_hdfs_textsimple(location text, month text, num_orders int, total_sales float8) LOCATION ('pxf://tmp/pxf_examples/pxf_hdfs_simple.txt?&PROFILE=hdfs:text')FORMAT 'TEXT' (delimiter=E',');

```

**需要注意：当使用Kerberos认证的Hadoop时，只能使用default作为server配置**




# 参考

[Introduction to PXF](https://gpdb.docs.pivotal.io/6-1/pxf/intro_pxf.html)

[Accessing Hadoop with PXF](https://gpdb.docs.pivotal.io/6-1/pxf/access_hdfs.html)：读写不同格式的HDFS文件、读写hive表、HBase表

[Configuring PXF for Secure HDFS](https://gpdb.docs.pivotal.io/6-0/pxf/pxf_kerbhdfs.html)

[Troubleshooting](https://gpdb.docs.pivotal.io/6-1/pxf/troubleshooting_pxf.html)