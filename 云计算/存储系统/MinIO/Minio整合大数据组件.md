# 1. 对象存储

目前，所有主要的云服务商都提供对象存储服务，包括：AWS S3、Aliyun OSS、Azure Blob Storage等。这些存储服务具有：存储成本低、API接口友好、便于各种服务集成的优势。

当前Spark、Flink等计算框架都有直接读写对象存储服务的API，允许开发者在设计业务时使用这些对象存储服务替换掉HDFS，更方便的将应用部署Kubernetes等Paas平台或者云平台中。

在大数据应用中使用对象存储的优势：

- 更好的实现[存储和计算分离](https://juicefs.com/blog/cn/posts/why-disaggregated-compute-and-storage-is-future/)架构（当前趋势），便于企业在云上部署大数据应用。从而实现弹性扩容、按需管理资源。
- 相比HDFS于Minio是云原生服务，架构非常简单，安装运维都非常简单，而HDFS属于非常典型的有状态服务，几乎没有非常完善的方案可以云化HDFS。

当前 Minio 是使用最多的开源对象存储服务之一，提供完全兼容S3的API接口。通过Minio，用户可以快速部署“私有化的S3服务”，下面简单介绍Minio如何与Spark、Hive集成，在一定程度上替代HDFS，成为大数据平台的共享存储服务。



# 2. Spark with Minio

Spark 提供了用于连接不同FileSystem的Connector，可以通过 S3 Connectors 将 Spark 的输入、输出保存在Minio中。

以下是 spark-shell 中一个简单的Demo：

```scala
/**
spark-shell \
--conf spark.hadoop.fs.s3a.endpoint=http://172.24.33.77:9000 \
--conf spark.hadoop.fs.s3a.access.key=minioadmin \
--conf spark.hadoop.fs.s3a.secret.key=minioadmin \
--conf spark.hadoop.fs.s3a.path.style.access=true \
--conf spark.hadoop.fs.s3a.impl=org.apache.hadoop.fs.s3a.S3AFileSystem \
--conf spark.hadoop.mapreduce.fileoutputcommitter.algorithm.version=2 \
--conf spark.hadoop.mapreduce.fileoutputcommitter.cleanup-failures.ignored=true \
**/

// read
val b1 = sc.textFile("s3a://spark/test.json")
b1.collect().foreach(println)

// write
import spark.implicits._
val data = Array(1, 2, 3, 4, 5)
val distData = sc.parallelize(data)
distData.saveAsTextFile("s3a://spark-test/test-write")

// 关于Spark使用对象存储的一些调优参数，可以参考：https://spark.apache.org/docs/2.3.0/cloud-integration.html
```

相比 HDFS 基于对象存储时，有以下劣势：

- 由于对象存储，并非真正的文件系统，因此不支持一些类 Posix 系统的特性

  - 最终一致性：通常对象存储的一致性要求不如HDFS严格，采用最终一致性模型。可能导致：1. 新建文件不能立即可见；2. 删除更新等操作不会立即传播；3. 文件夹操作性能较差（和文件数目成正比），任意故障会导致文件夹操作出现中间状态；
  - 对象存储中并没有真正的文件夹概念，导致所有文件夹操作性能较差，并且不具备原子性；
  - 权限模型和HDFS差别较大

- 文件基于网络传输，存在性能损失（在万兆网络的私有Minio环境中，这一部分的性能损失实际上并不严重）。

# 3. Hive with Minio

Hive 支持在Minio（S3）上创建外部表，通常情况除了在Hadoop的配置文件中配置S3的连接信息以外，还需要对Hive的配置文件进行独立的设置，从而提高提高Hive的读写性能。

# 4. MinIO Spark Select

MinIO Spark-Select 是一组基于Spark的API，支持使用 Select API 仅从对象中检索所需的数据。在一些轻量的数据分析场合，使用 Spark + Minio 即可以实现一些分析任务。

当前，Spark Select 支持  CSV、JSON 、Parquet 三种数据格式。

# 总结

目前，大数据任务运行在云端的场景越来越多，各大云服务商都在极力推广自己的对象存储服务，S3、Minio等组件的社区非常活跃，版本更新相当之快。因此HDFS作为共享文件系统的场景可能会越来越少，甚至能会被完全取代。

但在本地化的大数据场景中，Minio 在性能和功能上相比HDFS依然有所欠缺，无法完全替代。



# 参考

[minio/spark-select](https://github.com/minio/spark-select)

[Netflix/genie](https://github.com/Netflix/genie)

[cookbook：apache-spark-with-minio](https://github.com/minio/cookbook/blob/master/docs/apache-spark-with-minio.md)

[Hadoop-AWS ](https://hadoop.apache.org/docs/current/hadoop-aws/tools/hadoop-aws/index.html)

