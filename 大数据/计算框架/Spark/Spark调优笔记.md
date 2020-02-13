# 基础概念

每一个Spark任务，从不同的角度、从大到小涉及到以下概念：

- 按照占用的资源：Application --> Driver/若干Executor --> 每一个Executor 运行多个 Task（线程）
- 按照任务的逻辑：Application --> 若干连续执行的Job --> 每个Job划分为多个Stage

知识点：

- 从JVM角度每个Executor是一个独立进程，每个task是该进程的一个子线程。
- Job 中划分 Stage 以shuffle操作作为边界，而划分 Job 边界的是代码中的 action。
- 每个数据 partition 由一个 Task 处理，因此每个 Task 处理的数据量和 partition num 的大小成反比，控制 partition num 可以防止 Executor 发生 Oom。
- Task 的并行度由 slot 决定，slots = spark.num.executors * spark.executor.cores / spark.task.cpus。

# Shuffle 操作

Shuffle描述着数据从map task输出到reduce task输入的这段过程。在分布式情况下，每个 Reduce task 从不同 Map task 输出中拉取相同 Key 的 Records，这一过程会产生网络资源、内存、磁盘IO的消耗。

通常情况下，Shuffle 分成两个部分， 每个 Stage 从上一个 Stage Shuffle Read 得到数据，处理后进行 Shuffle Write 输出给下一个任务

- Map阶段的数据准备：Shuffle Write
- Reduce阶段的数据拷贝：Shuffle Read

在Spark的中负责shuffle的主要组件是ShuffleManager。

ShuffleManager随着Spark的发展有两种实现的方式：
- HashShuffleManager：Hash Shuffle
    - Spark 1.2以前默认配置
    - 会产生大量的中间磁盘文件，影响磁盘性能

- SortShuffleManager：Sort Shuffle
    - 从1.2 开始成为默认配置，并且在 Spark 2.0 时成为唯一的选择
    - 每个Task在进行shuffle后，将所有的临时文件合并(merge)成一个磁盘文件，因此每个Task就只有一个磁盘文件。Reduce Task 根据索引读取每个磁盘文件中的部分数据

参考资料：

 [Spark Shuffle的技术演进](https://www.jianshu.com/p/4c5c2e535da5)

 [Spark performance optimization: shuffle tuning](http://bigdatatn.blogspot.com/2017/05/spark-performance-optimization-shuffle.html)


# 内存管理

Spark中一个Executor对应一个JVM进程，Executor占用的内存分为两部分：ExecutorMemory和MemoryOverhead。ExecutorMemory是堆区内存，MemoryOverhead是Spark的堆外内存。

在Spark1.6以前的版本中，heap内存是静态管理的，而新版中内存使用动态管理方案进行管理。通过配置项spark.memory.useLegacyMode可以在两种方式中进行切换。


## 堆内存

动态配置的情况下，Heap内存包括以下几个部分：

- Reserved Memory：预留给系统使用，是固定不变的。默认300MB，并且这一部分是不可变的。
- User Memory：临时数据或者是自己维护的一些数据结构使用的内存空间， 默认大小：(Java Heap - Reserved Memory) x （1-spark.memory.fraction）（默认情况下1GB大小的Executor为289MB）。
- Spark Memory：系统框架运行时需要使用的空间，这是从两部份构成的分别是 Storage Memeory 和 Execution Memory。前者用来进行RDD缓存，后者用来Shuffle缓存。Storage 和 Execution (Shuffle) 采用了 Unified 的方式共同使用一个内存区域，默认情况下两者各站这一部分内存的50%，当一方内存不足时两者会相互占用对方内存，但是通常情况下Execution (Shuffle)的优先级更高！

## 堆外内存

Spark的堆外内存称为Memory-Overhead是JVM进程中除Java堆以外占用的空间大小，包括方法区（永久代）、Java虚拟机栈、本地方法栈、JVM进程本身所用的内存、直接内存（DirectMemory）等。通过spark.yarn.executor.memoryOverhead设置，单位MB。

默认情况下，spark.yarn.executor.memoryOverhead的大小按照下面的方式决定：

```
MEMORY_OVERHEAD_FACTOR = 0.07 
MEMORY_OVERHEAD_MIN = 384
min（MEMORY_OVERHEAD_FACTOR*spark.executor.memory，MEMORY_OVERHEAD_MIN）
```
默认情况下，spark单个Executor占用的内存资源为Spark堆外内存和Heap内存，当堆外内存超出限制时会产生OOM，使Yarn直接杀死容器，这时候没有任何异常。

[浅析 Spark Shuffle 内存使用](https://juejin.im/post/5cac122ef265da0356320f09)

# 分区

Spark任务的输入在绝大多数场景下存在分区的概念，默认情况下一个分区的数据在一个Task线程中执行。

可以使用以下两个参数，控制 Spark 任务的分区：

- spark.default.parallelism：控制RDD的分区数量
- spark.sql.shuffle.partitions：控制SQL的分区数量

需要注意，进行Shuffle时分区的数目是会发生变化的。

## Spark SQL 自动调整 Shuffle Partition

这个特性可以将Spark SQL执行Shuffle时，较小的连续分区进行合并，从而自适应 shuffle 时的分区数目。

该特性是 Intel 在[Intel-bigdata/spark-adaptive](https://github.com/Intel-bigdata/spark-adaptive)中设计，并且合入了Spark 2.3.1([SPARK-23128](https://issues.apache.org/jira/browse/SPARK-23128))。

spark-adaptive 针对 Spark 有下面三个优化：
- 在Spark SQL下，自适应 Shuffle Partition 的数量
- 动态调整执行计划（基于一些中间结果的数据量大小，动态的改变执行计划）
- 自动处理数据倾斜


参考: 

[Adaptive Execution 让 Spark SQL 更高效更智能](http://www.jasongj.com/spark/adaptive_execution/)

[Spark SQL在100TB上的自适应执行实践](https://mp.weixin.qq.com/s?__biz=MzA4Mzc0NjkwNA==&mid=2650784030&idx=1&sn=2c61e166b535199ee53e579a5092ff80&chksm=87faa829b08d213f55dab289bf5a12cfe376be0c944e03279a1c93e0f0d2164f1c6a6c7c880a&mpshare=1&scene=1&srcid=0111fEEzMCuhKozD4hsN4EE5&pass_ticket=WwOAQGxxBX9z63UyuFIXnWVm%2FSJhHkYwdsKplVDbaiA66ueqnDOtzgq86NgTgqvt#rd)


# Spark Web UI

Spark UI 的页面中，对检查任务的运行状态最有意义的是 Executors 和 Stages：

- Executors 可以查看以下内容

    - Task 运行情况，包括：正在运行数、失败数目、已经完成数目、以及 Task 的总数；
    - Task 累积运行时间，以及累积的GC时间
    - executor的Input（累积输入）、Shuffle Read（Reduce端读）、Shuffle Write（Map端写）
    - driver和executor的聚合日志

- Stages 可以查看以下内容

    - 每个Stages的持续时间（Duration）
    - Stages 的输入/ 输出数据大小（Input/Output）,这个指读写到Hadoop等外部存储的数据，以及从Spark Storage读到的数据
    - Shuffle Read / Shuffle Write 序列化后，shuffle读写的数据量
    - Stages 中每个 Task 的统计细节
        - Shuffle Read Size / Records : 序列化的shuffle读数据大小
        - Shuffle Write Size / Records : 序列化的shuffle写数据大小
        - Shuffle Spill (Memory) ： shuffle 过程中spill到disk的数据大小，这里指的是反序列化后数据大小
        - Shuffle Spill (Disk) ： shuffle 过程中spill到disk的数据大小，这里指的是序列化后数据大小

# Oom

处理Oom的一些手段：

- Driver Oom 时通常有以下可能：
    - Executor 返回的序列化结果集太小，而 spark.driver.maxResultSize（默认1g） 太小

- Executor Oom 时通常有以下可能：
    - 分区数量过小，那么单个Executor需要处理数据量会增多，使Executor的压力过大。
    - spark.yarn.executor.memoryOverhead太小，这时候堆外内存溢出，yarn会直接杀死容器，可能spark上可能看不到任何异常。
    - 由于Spark executor中多个Task并行时是共享内存的，因此减少slot可以改善Oom的情况；
    - 数据倾斜造成 OOM（主要原因是，堆内对象的分配和释放是由 JVM 管理的，而 Spark 是通过采样获取已经使用的内存情况，有可能因为采样不准确而不能及时 Spill，导致OOM）；

