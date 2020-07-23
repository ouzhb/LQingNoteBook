task:

1. 熟悉一个基础的监控方案：
	
	- prometheus + granfna
	- 将监控页面和Kubernetes集成
	- 告警推送方案：公众号、企业微信、邮件
	
2. CI/CD流程概念熟悉

	- Jekins，harbor，nexus仓库，gitlab
	- gradle插件学习
	- helm包语法学习
	
3. Kubernetes相关知识

	- 基础核心概念
	- 网络:~~~~
	- 存储：ceph、iscsi-target、Local-Disk~~~
	- rancher、k3s、
	
4. 大数据组件
	
	- Cloudera
	- Hive
	- HBase
	- Kafka
	- Spark
		- Shuffle 的含义解释
	- Hadoop

5. wis 的集群规模
    
    - Cloudera 节点：12台服务器 160GB 40/64 vCore
    - Kafka 数据量： 40～60M/s 每天原始数据4 TiB左右
    - 流处理估算：20s～1min 一个批次，每个批示7～20w条数据
    - 数据流向： Kafka --> spark streaming --> mongodb、codis、hdfs、CK
    
6. 数据相关知识

    - mysql
        - 关于存储引擎的一些基础知识
            - InnoDB索引的实现方式：主键聚集索引（叶子节点包含改行数据）+ 多个辅助索引（叶子包含到聚集索引的bookmark）
            
        - mysql 相关的一些性能调优案例
        - gerala 集群
        - mysql的一些分库分表中间件
        
    - NoSQL
        - mongodb 了解
        - ~~Clickhouse~~：
        ```
            Clickhouse 是基于 OLAP 场景的列式存储数据库，比较适合:
            - 少量并发查询
            - 不支持事务、二级索引等关系型数据库中的功能
            - 批量删除或者修改
            - 对于单表聚合操作性能非常优异，适合大宽表的场景
            
            Clickhouse中，分片、副本是通过不同的表引擎实现的主要用到的包括：
            - MergeTree表：基于合并树将以主键排序的数据顺序写入到后台文件中，并在必要的时候对文件进行合并。
            - Distributed引擎： 表分片的一个统一视图，通过这张表可以实现并行读写分片的功能。
            - 复制表：通过Zookeeper共享设备信息，从而实现数据复制（复制是多主异步的，INSERT 语句（以及 ALTER ）可以发给任意可用的服务器。数据会先插入到执行该语句的服务器上，然后被复制到其他服务器。）
      
            ClickHouse的环境信息：
            - 6 个节点，3分片/2副本
            - 表的大小： 相比mongodb，CK 磁盘使用率只有1/20。我们虽然在mongodb的表里开了snappy，但mongodb的索引非常占磁盘空间，而CK是采用稀疏索引（8192行进行一次索引）的机制，所以有非常明显的磁盘空间优势。
                dist_apSummaryInfo：87.3 亿 rows, 34.95 GB （原本mongodb 4.4亿/天，58GB/天）
                dist_areaHourStaTypeSyslog : 12.3 亿 rows, 11.74 GB （原本mongodb 2.2亿/天，36GB/天）
                dist_radiocfg : 279.7 亿 rows, 1.20 GB （原本mongodb 14亿/天，163GB/天）
                dist_staActiveStat :12.1 亿 rows, 27.08 GB （原本mongodb 1.2亿/天 ，25GB/天）
                dist_staPerfData : 77.8 亿 rows, 4.73 GB （原本mongodb 12亿/天，250GB/天）
                dist_staSyslog : 76.1 亿 rows, 15.22 GB （原本mongodb 15亿/天，300GB/天）
            - chproxy 进行负载均衡
            - 表按天分区、按天老化、数据保留15天
      
            从MongodbDB向ClickHouse迁移的原因：
            - 业务的核心数据库是Mongodb（28台服务器，4T SSD raid 1-0），主要数据存储方式是 Insert / Update 操作
            - mongodb在高并发写入情况下会出现写队列等待；队列堵塞后直接影响前端查询和后端数据入库。解决并发问题主要的途径就是扩容（降低每个monogdb实例的并发数和数据量）
            - 最后发现：可以将6张批处理的打表从Mongodb移出，这些表SQL简单，主要是批量操作
            - 寻找：写入数据量快，查询响应也能得秒级别，成本较低的数据库！
                
                - TiDB；
                - Greenplum；
                - CK；
      

        ```
        
        - Greenplum
        - HBase 
        - TiDB
        - redis/codis
        
    - 原理底层原理相关的知识
        - CAP原则：
        ```
        - Consistency： “一致性”，对于每一次读操作，要么都能够读到最新写入的数据，要么错误。
        - Availability：“可用性”，对于每一次请求，都能够得到一个及时的、非错的响应。
        - Partition tolerance： "分区容错"，即系统出现网络分区后，必须是可恢复的！<br> 只要是分布式DB，这一条都必须满足。
        
        所有的分布式系统都属于CP或者AP系统，而mysql等传统DB属于CA！
        通常CP系统出现网络故障的话，数据同步时间可能无限延长，此时系统会停止对外提供服务，来保证数据一致性，而AP系统在分区场景下依旧提供服务，但是用户可能发现系统数据存在不一致的情况。
        ```
        - Raft算法、Paxos算法
        - Join的广播和重分布
        - ACID
        ```
        - Atomicity：事务的操作结果要么全部执行要么全部不执行
        - Consistency：总是从一个一致的状态转换到另一个一致的状态
        - Durability：事务一旦被提交，它对数据库中数据的改变就是永久性的，接下来即使数据库发生故障也不应该对其有任何影响。
        - Isolation：事务的修改结果在什么时间能够被其他事务看到（GP使用快照进行隔离，MVCC引擎）
            - 脏读：事务之间能够看到未提交数据，导致事务回滚时其他事务读到脏数据
            - 不可重复读：事务能看到其他事务的已提交修改，这时会出现一个事务内两次读取数据不一致，这种模式下事务是不可重复读的。
            - 幻读：在两次读取时读取到的同一行数据是一致的，但是两次查询可能查到行数不一致（其他事务出现新的插入）
            - 序列化：以上情况都不会出现
        ```
        - 分片（shard）和分区（partition）
        ```
            - 分区（逻辑划分）的目的是根据分区键，将相邻的数据写到磁盘的相邻位置，从而使减小查询时的磁盘IO，通常比较常见的是按日期分区；
            - 分片的目的是根据片键，将表拆分成多块，分别存储在不同的机器上，从实现存储能力的扩展，以及并行读写的能力。
        ```
        
            