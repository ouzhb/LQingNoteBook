# 1. HBase是什么？有什么特点？

- Hbase是一款分布式**列簇数据库**，基于HDFS进行数据存储，适合存储半结构化或非结构化数据。

优点：

- 基于列簇进行存储，物理视图基于行存储和列存储之间，可以灵活切换
- 可以存储稀疏表
- 分布式、可扩展，支持TB级别数据量
- 支持Cell多版本，支持TTL自动老化

缺点：

- 不支持复杂聚合运算
- 没有二级索引
- 支持单行事务

# 2. HBase 逻辑视图

- HBase的数据存取模型为： Table：（RowKey、Family（Column ( Timestamp）)))--> Value
- 每行有一个 Row Key，数据根据 Row Key 排序
- 一行中多个列组成一个列族。
    - 列族在建表时定义好，不可以变动
    - 可以随时增减列族中的列数目
    - 一个列族的所有列存储在同一个底层的储存文中
- 每个column有不同的cell组成，每个Cell存储Timestamp以及不同版本的值。

# 3. HBase 物理视图

- Region 表示一张表的数据分片，开始时一张表对应一个Region。但表的数据超过一定阈值时，Region就会发生水平切分，分裂成两个Region
- HBase中同一个列族的列存储在相同的文件中称为一个Store，每个Region管理一个或者多个Store
- 每个Store由一个MemStore和一个或者多个HFile组成
- MemStore称为写缓存，用户数据写入时会首先写入到MemStore中。当MemStore超出阈值时，系统会异步将数据flush成一个HFile文件
- 当HFile过多时，HBase会进行Compact操作。
- HFlie中数据以Block的方式存储，同时Region进行分裂时的最小单位也是Block。
- HLog是MemStore的写时日志，用来保证Memstore掉电时，数据不会丢失（HLog文件保存在WAL目录中，当Memstor中的数据写成HFile时，过期的HLog文件被写入到oldWAL中）
- BlockCache是HBase中的读缓存结构，用户读取一行后根据LRU原则，相邻的行会被缓存到BlockCache中。当用户在此读取数据时，会在MemStore和BlockCache中查找数据，如果未命中再查找HFlie。

# 3. HBase 设计RowKey的原则

1. rowkey 长度原则，不宜太长， 原因是： HFile 中是按照 KeyValue 存储的，如果 rowkey 过长会极大影响 HFile 的存储效率。

2. rowkey 散列原则，避免Region出现热点。

3. rowkey 唯一原则：必须在设计上保证其唯一性，并将经常读取的数据存储到一块，将最近可能会被访问的数据放到一块。

# 4. 描述 HBase 中一个 Cell 的结构

Cell中包含：rowKey，列簇名，列命令，操作类型（Type），TimeStamp --> Value

# 5. HBase 中的 Compact 

HBase中包含Marjor、Minjor两种压缩过程：

- Minjor：当一个Region中某个列族的HFile超过设定数目时，会进行Minjor压缩。Minjor压缩主要为了减少HFile的文件数量，不会进行删除等操作。

- Marjor：HBase周期或者手工触发Marjor压缩，此时Region下的所有HFile文件会被重写，过程中会删除过期的row。Marjor压缩会产生巨大的负载。

# 6. HBase 宕机如何处理？

HMaster 会将该 HRegionServer 所负责的 region 转移到其他 HRegionServer 上，并且会对 HRegionServer 上存在 memstore 中还未持久化到磁盘中的数据进行恢复;
这个恢复的工作是由 WAL重播 来完成。

# 7. hbase如何获取region存储位置信息

HBase中所有Region的位置信息保存在hbase;meta表中，这张表只有一个Region，并且其位置信息保存在Zookeeper上。

当Client请求数据时，Master会将Region的信息返回，Client会将这些信息缓存到本地，防止频繁访问Master产生压力。

# 8. HBase优化方法

- 通过major压缩，提升表的locality，即使HFile和Region所在的RS位于同一个节点。

- 根据数据实际使用情况调整BlockCache、MemStore的内存占比

- 调整RS内存，以及GC方案

- 配置表开启布隆过滤器，提高读的性能

- 一些不重要的业务，可以调整Wal方案，甚至关闭HLog

- 使用Snappy压缩

- 对一些大批量的ETL任务，可以通过BulkLoad直接将数据在HDFS上写成HFile文件，在将HFile文件导入到HFlie中。

# 9. 为什么不建议在 HBase 中使用过多的列族

- 因为HBase以列族作为存储单位保存HFile，过多列族会导致每次Memstore flush时在HDFS上产生大量小文件

- 过多的列族导致，列族之间数据量不均匀，当Region分裂时，数据量较少的列族会跟着分裂，进一步产生大量小文件

# 10. 简单说明RIT问题

RIT问题是指，HBase上Region的状态不一致，包括：

- Zookeeper上Region的状态：RS和Master通信的状态，实际上是最新的状态

- HBase Master内存中Region中状态：WebUI上看到的状态

- 以及hbase;meta中Region的状态：记录Region最终被分配到哪一个RS，是最终状态

当Region split、Merge，上线、下线时，如果ZK、HDFS或者HBase Master发生故障，都会导致RIT。当发生RIT时，在HBase 2中由于引入了新的Produce V2机制，很多RIT可以自愈。否则可以使用HBCK工具介入，进行修复。

# 11.简单叙述Region Split、Merge过程

当Region中某个HStore超过了指定大小时，RS会通知Master进行Split，此时RS会以HFile文件中心Block的Start Rowkey为分界，形成两个新的子Region。当子region生成后，Master将原有的父region下线，并将子region上线。

Split过程是轻量的操作，不会产生大量IO，子Region通过引用的方式，复用父Region的HFlie文件。

Megre操作和Split过程相反，一般当执行删除或者数据Major压缩后，会产生空白Region、小Region，此时用户可以使用命令合并相邻的Region，从而提高HBase的内存利用率。





