# NoSQL选型需要考虑的点：
  - 数据模型：NoSql包括多种数据模型，如键/值对、半结构化列式存储、文档结构存储
  - 存储模型：内存存储、持久化存储
  - 一致性模型：严格一致，顺序一致、因果一致、最终一致、弱一致。参考CAP定理！！
  - 物理模型：单机 or 分布式
  - 读写性能：扫描？ or 随机读写？
  - 辅助索引：是否支持索引
  - 故障恢复：故障恢复的成本
  - 压缩：是否支持压缩、支持压缩的能力如何
  - 负载均衡：是否能够自动均衡
  - 原子操作的读-修改-写：大部分NoSQL不支持这种操作
  - 锁：是否支持锁，是否存在死锁问题？

# HBase

## column、column family、row

** HBase的数据存取模型为： （Table、RowKey、Family、Column、Timestamp）--> Value **

column是HBase的最基本单元，多个column组成row。每行有一个唯一的Row Key, 所有的行按照Row Key的排序顺序（字典序）存储。

多个column组成一个列族（column family），**一个列族的所有列存储在同一个底层的储存文中**。列族有以下特点：

1. 列族在创建表时就要定义好；
2. 列族的数量不能太多，但是column的数量几乎没有限制；
3. 列族名必须是可以打印的字节数组（family：qualifier）

每个column有不同的cell组成，每个Cell存储Timestamp以及不同版本的值。默认情况下，Timestamp由系统自动生成，但是用户也可手工指定。cell按照timestamp排序。

用户可以指定两种老化方式： 按照时间戳老化、按照保存的最大版本数老化。

## 自动化分区

1. 一张表初始化情况只有一个Region，当表增长到指定大小时，HBase中将其分裂成两个Region。
2. 一个Region服务器上可以有一张表的一个或者多个Region。
3. HBase的HFile文件保存在HDFS上，无需考虑副本、同步等因素。

## API

1. 支持单行事务，但是不支持跨行以及跨表事务；因此在HBase中，单元格的值可以用作计数器。

## 简单实现原理

### 集群组成

HBase 集群包括以下内容：

|角色|数目|作用|
|----|----|----|
|master|通过Zookeeper选取一个主Master|1. 负责跨Region服务器的全局region的负载均衡<br /> 2. 提供了元数据的管理操作，如创建表和列族<br /> 3. master不提供任何数据服务或者检索路径功能|
|region|多个|1. 负责为他们服务的Region提供读写请求 <br />2. 负责拆分region|

### HFile 文件

HBase 的数据以排序Key-Value的方式保存在 HFile 中， HFile 则以文件形式保存在 HDFS 中。

**查询**：

 HFile文件由64K大小的块组成，每个HFile文件尾部保存了块的索引信息。查询时通过块索引获取包括key的块，之后读取指定的块获取指定key的值，整个过程只发生的一次磁盘IO；

**插入**：

1. 数据优先进行WAL；
2. 数据写入内存的memstore；
3. 当内存中的memstore达到设定大小，写入HFile文件中（memstore中的kv已经是顺序的，写入到HFile的过程不需要重新排序）；
4. 当HFile不断增多时，会触发文件合并：

    - minor合并：只是将小文件合并成数量较小的大文件
    - major合并：将一个region中的一个列族的若干个HFile重写成一个新的HFile，并实际删除数据以及老化数据

HBase的实现涉及到了以下算法：
- LSM树：HBase用于HFile文件的合并
- 布隆过滤器：行键的查找和修改


# 安装部署

## Quick Start

HBase支持Standalone单机模式，将Master、Region、ZK运行在一个JVM进程中，并且以本地文件系统为存储。

部署步骤：

1. 安装JDK
2. 下载[安装包](https://hbase.apache.org/downloads.html)
3. 修改conf/hbase-site.xml，为以下内容

```xml
<configuration>
  <property>
    <name>hbase.rootdir</name>
    <value>file:///opt/hbase/data</value>
  </property>
  <property>
    <name>hbase.zookeeper.property.dataDir</name>
    <value>/opt/hbase/zookeeper</value>
  </property>
  <property>
    <name>hbase.unsafe.stream.capability.enforce</name>
    <value>false</value>
  </property>
</configuration>
```
4. 执行bin/start-hbase.sh 启动 HMaster 进程
5. 登入web：http://172.24.26.90:16010
6. 通过hbase客户端检查可用性

```shell
# 进入HBase命令行
./bin/hbase shell

# 创建表： create '表名', "列族1", "列族2"
create 'test', 'cf1','cf2'

# 打印表信息
list 'test'
describe 'test'

# 插入数据：put '表名', "row key","列族：列","值","ts1", {ATTRIBUTES=>{'mykey'=>'myvalue'}}
put 'test', 'row1', 'cf1:a', 'value1'

# 全表扫描
scan 'test'

# 取数据：get 't1', 'r1', {FORMATTER => 'toString'}
get 'test','row1'

# Disable a table.
disable 'test'

# 删除表
drop 'test'

```

## 伪分布式

HBase依然运行在单个节点上，但是不同角色运行在不同的JVM进程中。

hbase-site.xml中添加以下配置：

```xml
<property>
  <name>hbase.cluster.distributed</name>
  <value>true</value>
</property>

<property>
  <name>hbase.rootdir</name>
  <value>hdfs://hdfs-k8s/hbase</value>
</property>
</configuration>

```

之后执行bin/start-hbase.sh命令即可启动相关进程，还可以使用 local-master-backup.sh 和 local-regionservers.sh 启动备份的master以及多个regionservers。

## 完全分布式

参考以下组网方式

|Node Name|Master|ZooKeeper|RegionServer|
|-----|-----|-----|-----|
|node1|yes|yes|no|
|node2|backup|yes|yes|
|node3|no|yes|yes|

部署包括以下步骤：

1. 节点配置互信
2. 修改 node1 中的 conf/regionservers 添加 node2、node3
3. 在 node1 创建 conf/backup-masters 添加 node2
4. 添加zookeeper的配置
```xml
<property>
  <name>hbase.zookeeper.quorum</name>
  <value>node-a.example.com,node-b.example.com,node-c.example.com</value>
</property>
<property>
  <name>hbase.zookeeper.property.dataDir</name>
  <value>/usr/local/zookeeper</value>
</property>
```
5. 拷贝配置文件到node2和node3
6. start-hbase.sh

# 参考资料

[架构说明](http://hbase.apache.org/book.html#_architecture)

[配置文件说明](http://hbase.apache.org/book.html#_configuration_files)

[基础参数的配置](http://hbase.apache.org/book.html#basic.prerequisites) : JDK版本、DNS、NTP、文件句柄数以及进程数配置

[Hadoop 配置](http://hbase.apache.org/book.html#hadoop) ： 版本适配，建议使用hadoop 2.7.1+

[配置项说明](http://hbase.apache.org/book.html#config.files)