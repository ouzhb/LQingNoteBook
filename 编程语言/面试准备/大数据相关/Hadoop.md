# 1. 简单叙述HDFS的CheckPoint过程

NameNode通过Editlog和FSImage的方式来保证元数据的高可用。在HDFS上每一次创建、删除文件 ，NameNode都会将变更的信息在Editlog上进行顺序写，然后在修改NameNode的内存状态。

同时NameNode内部有一个线程，周期性的把内存状态导出到本地磁盘持久化成FSImage文件，对应已经持久化到FSImage文件的日志，属于过期的编辑日志可以删除。这整个过程称为CheckPoint。

当HFDS启动时，NN会将最新的FSImage加载到内存，并且从某个时间点Editlog回放元数据修改，直到内存状态和停机之前一致。由于NN在内存中维护每一个文件元数据信息，因此如果HDFS上小文件过多，容易造成内存压力，启动缓慢等问题。

# 2. 简单描述JournalNode的作用

JournalNode实际上是用来维护NN上Editlog一致性的Paxos组，每次NN写入编辑日志时需要半数以上JN写入成功才可以返回。

# 3. hflush和hsync的区别

hflush 刷新到DN，数据可能还没有落到硬盘。

hsync 刷新数据到DN，并且写入到磁盘。

# 4. 什么是HDFS上的Short Circuit Read

当Client和Block位于同一台机器时，数据读取不经过TCP协议，而是Client直接通过Block的文件句柄获取数据。


# 5. HDFS 小文件存储方案

引起的问题：
1.对NN产生内存压力：每个block、file在NN中占用150 bytes 内存，一般 1000w个小文件，每一个文件1个block，那么会消耗3GB内存。

2.显著影响Spark、MapReduce 等ETL任务从HDFS上读取数据的性能。

解决方案：

1. HAR files：hadoop archive命令进行归档

2. SequenceFile：存储二进制形式的[Key,Value]对而设计的一种平面文件(Flat File)

3. 存HBase
 
