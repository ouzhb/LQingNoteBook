---
title: 数据库调研笔记 -- MyCat
categories: "笔记"
date: 2019-11-04
comments: true
toc: true
tags:
	- 数据库
	- MyCat
---


数据库调研笔记 -- MyCat


<!--more-->

# 介绍

MyCat基于Ali的开源组件cobar演变而来，是一款知名度较高的SQL中间件。

当前[MyCATApache/Mycat-Server](https://github.com/MyCATApache/Mycat-Server)项目有7.2K Star，89个贡献者。

当前的稳定版本是[MyCat 1.6.5](http://dl.mycat.io/1.6.5/)，另外社区还有[Mycat2](https://github.com/MyCATApache/Mycat2/blob/master/doc/00-mycat-readme.md)项目，但是当前还没有正式发行版。

目前，MySQL后端支持以下数据库： Mysql、SQL Server、Oracle、 DB2、PostgreSQL、部分NoSQL

PS："MyCat目前在Github上有655个open状态的issue，绝大数issue没有人回应。"

## 安装

第一次安装使用MySQL 8.0作为后端，但是安装完成后发现存在兼容性问题：Mysql 8.0自带的mysql客户端无法正常连接mycat，但是使用低版本的mysql客户端可以正常连接（这个问题Github上有[issues](https://github.com/MyCATApache/Mycat-Server/issues/1842)，但是没有修复）。

mysql 8.0 安装过程如下：

```shell

# 安装mysql 8.0
wget -i -c  https://repo.mysql.com//mysql80-community-release-el7-1.noarch.rpm
yum -y install mysql80-community-release-el7-1.noarch.rpm
yum -y install mysql-community-server

# 安装mysql5.7
#wget -i -c https://dev.mysql.com/get/Downloads/MySQL-5.7/mysql-5.7.28-1.el7.x86_64.rpm-bundle.tar
#tar -xvf mysql-5.7.28-1.el7.x86_64.rpm-bundle.tar
#yum -y localinstall mysql-community-*5.7.28-1*.rpm

echo "lower_case_table_names=1" >> /etc/my.cnf
echo "default-authentication-plugin=mysql_native_password" >>  /etc/my.cnf  # 安装低版本mysql时无需配置
systemctl enable mysqld.service && systemctl start mysqld.service

grep "password" /var/log/mysqld.log
mysql -uroot -p

# 连接mysql修改默认密码，以及认证加密方式，并创建数据库

ALTER USER 'root'@'localhost' IDENTIFIED BY 'Ruijie@123';
ALTER USER 'root'@'%' IDENTIFIED BY 'Ruijie@123' PASSWORD EXPIRE NEVER;       # 使用低版本mysql时无需执行
ALTER USER 'root'@'%' IDENTIFIED WITH mysql_native_password BY 'Ruijie@123';  # 使用低版本mysql时无需执行

use mysql;
#GRANT ALL PRIVILEGES ON *.* TO 'root'@'%' IDENTIFIED BY 'Ruijie@123';         
UPDATE USER SET HOST = '%' where USER = 'root'; 
flush privileges;
CREATE DATABASE mycat_db;                   

```

部署mycat

```shell

groupadd -g 550 mycat
useradd -g 550 -u 550 -m -d /home/mycat -s /bin/bash mycat
echo "ruijie" | passwd --stdin mycat

wget http://dl.mycat.io/1.6.7.3/20190927161129/Mycat-server-1.6.7.3-release-20190927161129-linux.tar.gz
tar -xzvf Mycat-server-1.6.7.3-release-20190927161129-linux.tar.gz -C /usr/local
chown -R mycat:mycat /usr/local/mycat

echo "export MYCAT_HOME=/usr/local/mycat" > /etc/profile.d/mycat.sh
echo "export PATH=$PATH:$MYCAT_HOME/bin" >> /etc/profile.d/mycat.sh
source /etc/profile

# 如果后端数据库使用的是mysql 8.0需要替换lib目录下的驱动文件

```

安装过程中，注意schema.xml和server.xml文件：

- schema.xml定义mycat的逻辑表和逻辑库信息，以及后端mysql信息；
- server.xml文件定义连接信息和用户信息，安装包中默认提供二个用户 root/123456 和 user/user；

```xml

<!-- 
    PS：该配置文件中mycat规划了2个shard，小于默认的三个，需要修改autopartition-long.txt，将最后一行配置删掉
-->

<?xml version="1.0"?>
<!DOCTYPE mycat:schema SYSTEM "schema.dtd">
<mycat:schema xmlns:mycat="http://io.mycat/">

	<schema name="TESTDB" checkSQLschema="true" sqlMaxLimit="100">
		<table name="travelrecord" dataNode="dn1,dn2" rule="auto-sharding-long" />
	</schema>
	<dataNode name="dn1" dataHost="mycat71" database="mycat_db" />
	<dataNode name="dn2" dataHost="mycat72" database="mycat_db" />
	<dataHost name="mycat71" maxCon="1000" minCon="10" balance="0" writeType="0" dbType="mysql" dbDriver="native" switchType="1"  slaveThreshold="100">
		<heartbeat>select user()</heartbeat>
		<writeHost host="mycat71" url="mycat71:3306" user="root" password="Ruijie@123">
		</writeHost>
	</dataHost>
	
	<dataHost name="mycat72" maxCon="1000" minCon="10" balance="0" writeType="0" dbType="mysql" dbDriver="native" switchType="1"  slaveThreshold="100">
		<heartbeat>select user()</heartbeat>
		<writeHost host="mycat72" url="mycat72:3306" user="root" password="Ruijie@123">
		</writeHost>
	</dataHost>
</mycat:schema>

```

安装完成后执行mycat start启动mycat服务。

# 配置信息

Mycat的配置信息主要位于schema.xml、server.xml、rule.xml三个文件中。

[MyCat权威指南](chrome-extension://ikhdkkncnoglghljlkmcimlnlhkeamad/pdf-viewer/web/viewer.html?file=http%3A%2F%2Fwww.mycat.io%2Fdocument%2Fmycat-definitive-guide.pdf)第七章详细说明了这三个配置文件。

## schema.xml文件

### 1. schema标签

该标签用来定义mycat中的逻辑数据库，对于一个mycat实例可以配置多个逻辑数据库。

schema字段包含以下属性：

- name：逻辑库名称
- dataNode：将逻辑库绑定到后端一个具体的数据库，此时没有配置分片的表落在该数据库中
- checkSQLschema：将发送到后端DB的schema关键字去除
- sqlMaxLimit：自动加上limit语句

### 2. table标签

该标签是schema的子标签，所有需要拆分的表在该标签中定义，形成一张拆分逻辑表。

- name：逻辑表名
- dataNode：逻辑表分片到的实际数据库，可以定义多个，或者通配符定义
- rule：逻辑表遵循的分片规则，规则在rule.xml中定义（tableRule标签中的name属性一一对应）
- ruleRequired：该属性用于指定表是否绑定分片规则，如果配置为true
- primaryKey：该逻辑表对应真实表的主键。使用非主键分片时，指定实际主键会使Mycat缓存查询结果，提高性能
- type：表属性，包括全局表、普通表两种
- autoIncrement：指定这个表有使用自增长主键
- subTables：启用分表属性（指对大表宽表进行垂直拆分），分表放在一个数据库中，并且不支持各种条件的join
- needAddLimit：自动的在每个语句后面加上limit 限制（默认为true）

### 3.childTable标签

childTable用于定义E-R分片的子表

### 4. dataNode/dataHost标签

dataNode 标签定义了MyCat中的数据节点。一个dataNode 标签就是一个独立的数据分片，对应一个后端数据库。

dataHost 标签定义了一个数据库的具体链接方式，读写分离配置、和心跳语句。dataHost 标签比较重要的参数包括：

- maxCon/minCon：连接池配置
- balance ：读操作负载均衡配置，当前支持单点读、读写分离、双主双从、随机分发
- writeType ：写操作的服务负载均衡
- switchType：主备切换配置
- dbType：数据库类型，目前实际上mycat只支持mysql的二进制协议，但是这里可以填写MongoDB、Oracle、Spark等类型的数据库，但本质上都是通过jdbc连接的后端。
- dbDriver：驱动类型支持natvie、jdbc两种
- heartbeat 标签：心跳语句
- writeHost标签、readHost标签

## server.xml文件

server.xml几乎保存了所有mycat需要的系统配置信息。其在代码内直接的映射类为SystemConfig类。其中最为重要的标签是：

- user标签：定义mycat的连接用户和权限。
- system标签：定义mycat的系统配置，如字符集、sql解析器、线程数据等等

## rule.xml文件

该文件定义了一些分片策略，在schema.xml文件中指定表分片时可以指定这些分片策略。

```xml

<tableRule name="rule1">    <!--表规则名称-->
    <rule>
        <columns>id</columns>   <!--片键名称-->
        <algorithm>func1</algorithm> <!--路由算法名称-->
    </rule>
</tableRule>

<!--分片算法通过java类实现，配置指定在文件中-->
<function name="hash-int"class="io.mycat.route.function.PartitionByFileMap">
    <property name="mapFile">partition-hash-int.txt</property>
</function>

```

常用分片算法：参考[MyCat权威指南](chrome-extension://ikhdkkncnoglghljlkmcimlnlhkeamad/pdf-viewer/web/viewer.html?file=http%3A%2F%2Fwww.mycat.io%2Fdocument%2Fmycat-definitive-guide.pdf)第10.5章节！

# Mycat的join操作

Mycat支持以下几种场景的Join：

## 全局表

全局表在所有节点保存完整副本，具备以下特性：

- 全局表的插入、更新操作会实时在所有节点上执行，保持各个分片的数据一致性
- 全局表的查询操作，只从一个节点获取
- 全局表可以跟任何一个表进行JOIN操作
- Mycat通过在MYSQL中额外添加内部列进行全局一致性检查

    - 检查全局表是否存在内部列
    - 检查全局表的记录总数
    - 检查全局表的时间戳最大值

- 插入全局表时必须带列名插入


## E-R Join

E-R Join是在schema.xml文件中预先定义两张的关联key，使分片时关联键相同的数据分布在同一个数据库中，从而保证不会发生跨跨库join。

```xml

<!-- customer 和 orders 的关联关系为 orders.customer_id=customer.id -->
<table name="customer" dataNode="dn1,dn2" rule="sharding-by-intfile">
    <childTable name="orders"  joinKey="customer_id" parentKey="id"/>
</table>

```

## 其他Join方式

- Share Join：ShareJoin是一个简单的跨分片Join,基于HBT的方式实现，目前支持两个表join，官方文档称只在开发版中有该功能。
- Catlet Join：通过自定义接口实现Join操作。
- Spark/Storm对join扩展：还没有实现！！

# 全局序列号

Mycat中全局序列号主要用来为表提供自增组件，用户可以通过：本地文件、数据库配置、时间戳配置、以及分布式ZK ID生成器。



# 总结

经过几天的测试，对 MyCat 使用总结以下几点：

优势：

- 使用较为简便，主要因为是，原理简单（就是SQL变换），并且是国产软件有很多中文资料；
- 预留了很多API进行自定义开发；
- 纯java实现，有进行源码级别Debug、自定义开发的可能；

劣势：

- 并非真正意义上的分布式数据库，相比Greenplum、TiDB这种真正的分布式方案架构上有先天缺陷（个人认为，Mycat的发展方向路子走偏了）
- 有很多功能性的限制，以下是官方权威指南中提到的功能限制（不包括网络上其他人发现的）


    - 跨库Join 功能太弱，只能支持全局表Join或者片内join（Share join支持2个表的跨库join，但是性能堪忧），考虑以下场景MyCat很难胜任：

```sql

-- A表：id1 、id2
-- B表: 片键id
-- C表: 片键id
--
-- E-R join无法同时满足下面二个Join，只能选择将一张表设定成全局表：

select * from A , B where A.id1 = B.id;
select * from A , B where A.id2 = C.id;
```

    - Mycat的全局表的一致性检查机制比较简单，且出现不一致时没有自动恢复机制（个人认为：误操作、掉电等场景都有可能出现不一致，Mycat的一致性检查太low了）

    - 官方文档提到MyCat不支持复杂子查询，但没有给出具体说明（很迷！！）

    - 保证高可用需要用户自行设计组网方案：MyCat本身不提供后端数据库的HA能力，也不提供MyCat服务本身的HA功能。每个接入的MySQL都需要进行主从复制的配置，而MyCat本身使用HAProxy避免单点故障。（这一点实际上又引申出如何保障mysql高可用的问题，从IData的应用场景来看还是比较复杂的）

    - Mycat没有提供备份和恢复方案，只能使用mysql的备份工具备份每一个mysql数据库，并且为了保证数据一致性每一次备份需要业务停止写入；

    - MyCat使用配置文件定义分片策略，如果分片规则变更数据无法同步更新。换句话说，如果生产中发现之前定义的分片不均衡或者节点扩容，那么需要重写整张表。

    - 分布式事务能力较弱，官方文档提到：Mycat 目前没有出来跨分片的事务强一致性支持，目前单库内部可以保证事务的完整性，如果跨库事务，在执行的时候任何分片出错，可以保证所有分片回滚，但是一旦应用发起commit 指令，无法保证所有分片都成功考虑到某个分片挂的可能性不大所以称为弱xa。（看到一篇blog说MyCat基于 Mysql XA 接口实现分布式事务，但是 Mysql XA 接口本身用户不多，因此性能和可用性都可能存在问题！）
