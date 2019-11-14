# HIVE

Apache Hive™有助于用户使用SQL读取，编写和管理驻留在分布式存储中（如HDFS）的大型数据集。可以将表结构投影到已存储的数据中，并且提供CLI和JDBC使用户便捷访问Hive的数据。

当前HIVE支持

# 架构

Hive的主要组成模块包括：

- UI 
- Driver ：Driver用于接收用户的查询请求。该模块实现了session handles的概念，并且提供JDBC/ODBC的API接口。
- Compiler：分析查询语句，并且从MetaStore获取table和partition的元数据信息，最终生成执行计划。
- MetaStore：保存table、partitions结构的组件。
- Execution Engine：执行Compiler生成的执行计划，实际上应该是调用spark、MR的接口。

![典型的查询执行流程](https://cwiki.apache.org/confluence/download/attachments/27362072/system_architecture.png?version=1&modificationDate=1414560669000&api=v2)

## 数据模型

Hive的数据模型，从大到小依次为：

- Tables： 表中的数据保存在HDFS的目录中，
- Partitions：每个表可以有一个或多个分区键（partition keys），用于确定数据的存储位置。分区键和普通键不能重复，创建分区的过程如下：

```shell
# 创建一张分区表，pt_d为表的分区键
create table t1(
    id      int
   ,name    string
   ,hobby   array<string>
   ,add     map<String,string>
)
partitioned by (pt_d string)
row format delimited
fields terminated by ','
collection items terminated by '-'
map keys terminated by ':'
;

# 在t1表中插入以下数据

1,xiaoming,book-TV-code,beijing:chaoyang-shagnhai:pudong
2,lilei,book-code,nanjing:jiangning-taiwan:taibei
3,lihua,music-book,heilongjiang:haerbin

# 执行load data
load data local inpath '/root/data.txt' overwrite into table t1 partition ( pt_d = '201701');

# 查看t1表的分区/数据
select * from t1;
show partitions t1;

# 删除指定分区
ALTER TABLE t1 DROP IF EXISTS PARTITION ( pt_d = '201701');

# HDFS上t1表的存储目录

/user/hive/warehouse/test.db/t1 目录下出现了分区文件夹：pt_d=201701

```
- Buckets：分区中的数据可以基于表中列的散列，被分成桶。每个存储桶都作为一个存储文件在分区目录中。

HIVE除了支持INT、FLOAT、STRING、DATE、BOOLEAN等基础类型，还支持Arrays和Maps。

此外，用户可以从任何基础类型，集合或其他用户定义的类型以编程方式组成自己的类型。

实现自定义类型时，用户需要实现HIVE的Serailization/Deserialization接口。

## Metastore

Metastore主要功能包括：

1. Data Abstraction：在用户创建Table时记录下表结构数据，而在执行查询时为Driver提供这些数据抽象；
2. Data Discovery： 指metastore可以为其他组件提供数据仓库中的元数据信息

MetaStore需要维护data和metadata的数据一致性。

MetaStore中包括以下数据：

1. Database： 表的命名空间信息
2. Tables：列信息（数据类型等信息）、所有者、storage（数据的存储位置、文件格式、bucketing信息）、SerDe（数据的序列化/反序列化类）等
3. Partition：每个Partition保存一个独立的columns、SerDe、storage信息

Metastore的数据存取基于[DataNucleus](http://www.datanucleus.org)（一种ORM接口），实际数据可以存放在DB或者文件中。

Metastore可以通过 remote 、local/embedded 两种方式部署：

- remote ： metastore 作为一个 Thrift 服务，可以为一些非Java client提供Hive的接口
- local/embedded ：Hibie通过内置的Derby提供metaserver，这种模式下只有一个process能够连接metastore 的database，只能用于测试环境。
- local： 

## Compiler

Compiler包括以下内容：

- Parser：将查询表示成一种树状结构（ parse tree）
- Semantic Analyser：将Parse Tree进一步转换成一种内部查询结构，此时column名验证、"*"符号扩展、类型验证、隐式转换等操作被执行。如果是分区表，此时Compiler会收集分区信息用于后续修剪分区。
- Logical Plan Generator：将上面的内部查询转换为，filter、join等操作，这些操作和MR任务对应。一般通过优化该步骤能够提高HIVE的查询性能。
- Query Plan Generator：将Logical Plan装换成MR任务。每个任务的边界是reduceSink operator 
  
## Optimizer

Optimizer在HIVE的历代版本中不断进化，优化手段包括：column修剪、map-side join、sort nature等手段。

# 参考

[Hive Design](https://cwiki.apache.org/confluence/display/Hive/Design)

[AdminManual Metastore Administration](https://cwiki.apache.org/confluence/display/Hive/AdminManual+Metastore+Administration#AdminManualMetastoreAdministration-RemoteMetastoreDatabase)