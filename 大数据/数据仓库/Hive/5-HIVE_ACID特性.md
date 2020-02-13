# 说明

Apache HIVE 在 [HIVE-5317](https://issues.apache.org/jira/browse/HIVE-5317) 中实现了较为完整ACID支持，能够实现标准的Insert、update、delete语法，该特性在 0.14.0 版本引入。但是有以下限制条件（参考[【wiki Hive Transactions】](https://cwiki.apache.org/confluence/display/Hive/Hive+Transactions)）：

- 执行ACID操作的表存储格式必须为：AcidOutputFormat 以及 ORCFileformat
- 需要将表设定为分桶表
- 表属性中必须指定 transactional 为 true
- 用于分桶或者分区的键不能UPATE
- 不支持对ACID使用 LOAD DATA 方式加载数据（早期版本不支持， [HIVE-16732](https://issues.apache.org/jira/browse/HIVE-16732)修复了这个问题，但是IData 平台依旧不支持）
- 外部表不支持ACID特性（参考[HIVE-13175](https://issues.apache.org/jira/browse/HIVE-13175)）
- 在 non-ACID session 中不能读写ACID表（ non-ACID session 表示启动HIVE进程时，没有指定相关的ACID配置）
- 不支持 *BEGIN*, *COMMIT*, *ROLLBACK* 等事务相关的关键字，并且所有ACID操作都是自动提交
- 不具备完整的SQL隔离特性，多个并发查询时可能会出现结果不一致



由于Hive底层HDFS不支持对文件进行修改或者提供文件锁，因此Hive中所有update、delete、insert等操作均通过delta文件的方式实现，每个操作均会生成一个新的HDFS文件，如下。

```
/user/hive/warehouse/t/base_0000022
/user/hive/warehouse/t/base_0000022/bucket_00000
/user/hive/warehouse/t/delta_0000023_0000023_0000
/user/hive/warehouse/t/delta_0000023_0000023_0000/bucket_00000
/user/hive/warehouse/t/delta_0000024_0000024_0000
/user/hive/warehouse/t/delta_0000024_0000024_0000/bucket_00000
```

但用户读取文件时，SDK可以对这些delta文件进行自动合并，最后呈现给用户一张完整的表。

**随着delta文件越来越多，HIVE后台会启动压缩进程压缩这些文件，用户无需（压缩进程的参数是可以配置的，参考[【wiki Hive Transactions】](https://cwiki.apache.org/confluence/display/Hive/Hive+Transactions)）**。



由于 Apache Hive 的实现，这个特性基本上只能用在批量更新的场景上，按照官方的Use Cases，这个特性可以适用以下几个场景：

- 每小时一次，维度表的批量更新（up to 500k rows）
- 每天一次，批量删除表的记录（up to 100k rows）
- 每小时一次，对事实表的一些批量操作







当前 IData 平台使用的HIVE 版本是 2.1.1-cdh6.1.0，经过测试可以支持这个特性：

```sql
CREATE TABLE IF NOT EXISTS test_table( 
    BUCKETS_KEY  STRING, 
    INFO STRING
) CLUSTERED BY (BUCKETS_KEY) INTO 7 BUCKETS 
STORED AS ORC 
TBLPROPERTIES('transactional'='true');

INSERT INTO TABLE test_table VALUES ("AAAA","123456"),("BBBB","123456");
INSERT INTO TABLE test_table VALUES ("FFFF","123456");
UPDATE test_table SET INFO = "88888" WHERE BUCKETS_KEY="BBBB";
DELETE FROM test_table WHERE INFO="88888";
```



# 其他内容

## 1. 涉及到配置项

- hive.support.concurrency = true
- hive.enforce.bucketing = true
- hive.exec.dynamic.partition.mode = nonstrict 
- hive.txn.manager = org.apache.hadoop.hive.ql.lockmgr.DbTxnManager
- hive.compactor.initiator.on = true
- hive.compactor.worker.threads = 1

## 2. HIVE的分桶表

HIVE 的分桶机制：**对分桶Key进行Hash取模后，将数据分别保存到不同的桶文件中**。

分桶是相对分区表进行更细粒度的存储空间划分：

- 分区表将相同分区键的行保存到相同的**分区目录**中
- 分桶表将分桶键Hash模值相同的行保存到相同的**分桶文件**中

## 3. Hive Streaming API

https://cwiki.apache.org/confluence/display/Hive/Hive+Transactions

