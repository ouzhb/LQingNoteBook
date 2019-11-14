

# Spark 问题记录

### JSON 解析性能低下

#### 问题描述

ProtoDhcpOperate、ProtoDnsOperate、ProtoHttpOperate、ProtoRadiusOperate这四个Class使用scala原生JSON库进行JSON解析，并且在driver进程中使用spring接口查询MongoDB。

上述操作中JSON解析花费了大量时间，导致spark任务卡死。

#### 解决方案

使用fastjson代替原生scala原生JSON库进行解析。参考以下代码：

```scala
import com.alibaba.fastjson.JSON
import com.mongodb.spark.config.ReadConfig
import org.apache.spark.{SparkConf, SparkContext}
import com.mongodb.spark._
object Main {
  def main(args: Array[String]): Unit = {
    val conf = new SparkConf().setAppName("NetSecuSumTask")
      .set("spark.ui.port", "4066")
      .setMaster("local[2]")
    val sc = new SparkContext(conf)
    val rows = sc.loadFromMongoDB(ReadConfig(Map[String, String]("uri" -> "mongodb://172.18.135.2:26000,172.18.135.3:26000,172.18.135.4:26000",
      "database" -> "ion",
      "collection" -> "dnsBase"))
    ).map(x => {
      x.remove("_id")
      val json: String = JSON.toJSONString(x, true)
      JSON.parseObject(json, classOf[DnsBase])
    })
    rows.foreach(println)
  }
}
```

#### 注意事项

```scala
com.ruijie.util.CommUtils#convertDocument2Json
# 上面这个接口好像有点问题，转换到的字符串fastjson可能解析不了；
```

### MongoDB 读

####  问题描述

当前assurance读取mongodb时主要使用的是以下两个接口：

```scala
com.ruijie.dao.mongodb.intf.BaseMongoDao#findAll // 情况1：全表查询
com.ruijie.dao.mongodb.intf.BaseMongoDao#find // 情况2： 带条件查询
```

上面两个接口都是spring-data-mongodb接口，没有分布式扩展能力。

读取MongoDB生成RDD的过程：

```
driver调用spring-mongodb -> driver 加载所有数据到内存 -> 调用parallelize 生成RDD
```

上述所有读写操作都发生在driver中，如果需要查询大表，只能加大driver的内存，非常容易造成OOM。

####  解决方案

**情况1**：建议使用**com.ruijie.dao.mongodb.intf.BaseMongoDao#loadForSpark**接口替代**findAll**接口。该接口直接返回**MongoRDD[Document]**，不会对driver产生内存压力！

**情况2**： 条件查询

| 场景1          | 方案                                                         | 优势                           | 劣势                                           |
| -------------- | ------------------------------------------------------------ | ------------------------------ | ---------------------------------------------- |
| 时间范围查询   | 以assurance-application-health任务从appUerInfoInfo表中查询10min的数据为例。可以将appUerInfoInfo表中的数据放到共享存储中，每隔10min放在一个目录中，需要时直接读取整个目录的数据。 | 能够保证性能                   | 对于复杂查询无能为例，代码改动比较大，灵活性差 |
| 非时间范围查询 | 替换其他数据库存储，比如es什么的                             | 支持更多操作，便于后续扩展功能 | 选用什么数据库、能够达到何种性能需要进行测试。 |

#### 注意事项

### MongoDB 写

#### 问题描述

当前assurance写mongodb时主要使用的是以下接口：

```shell
com.ruijie.dao.mongodb.intf.BaseMongoDao#bulkInsert
```

该接口是spring-data-mongodb接口，是单线程写的接口。从目前情况看，135.2单线程写入的性能非常差，每秒只有几百到一千条。 

#### 解决方案

建议向mongodb写入数据时，增加RDD的分区。目前代码中，RDD分区数=mongodb插入线程数。

插入的代码参考：

```scala
    appConvRdd.foreachPartition(iter =>{
      AppConvInfoDao.bulkInsert(iter)
    })
```

#### 注意事项

RDD分区数目按照以下方式确定：

1. RDD来自kafka：RDD分区数 = kafka分区数
2. RDD通过parallelize 或者makeRDD函数生成，按照以下规则：
   1. parallelize 、makeRDD指定了分区数，即为指定值；
   2. parallelize 、makeRDD没有指定分区值，参考spark.default.parallelism值为分区值（spark.default.parallelism的默认值和机器的core数目有关）

### 内存泄露

#### 问题描述

spark流处理任务，在数据量一致的情况下（空跑），流处理时间不断变长。

#### 解决方案

原因是在streaming中的foreachRDD调用persist时，不会自动释放内存。每个批处理结束以后，需要手工调用unperist释放。

#### 注意事项

### Kafka日志异常消费

#### 问题描述

```
18/12/27 13:52:00 ERROR JobScheduler: Error generating jobs for time 1545889920000 ms
java.lang.IllegalArgumentException: requirement failed: numRecords must not be negative
```

#### 解决方案

问题原因是spark任务消费topic时，发现current offset小于 lagerest offset 。

产生问题原因应该是有人清理了Kafka上对应topic的数据。

可以登录Zookeeper删除**/consumers/**对应目录下的offset（**rmr命令**），或者删除spark的Checkpoint。

#### 注意事项

wis和ion的任务全是使用direct方式消费kafka，但是wis会手工在zk上更新Offset，而ION不会。因此ION会出现kafka数据丢失的情况，而Wis可能出现重复消费的场景。

### local-cluster 模式内存指定

#### 问题描述

指定local-cluster 时无法通过local-cluster[N,vCore,Mem]指定executor内存。

#### 解决方案

通过spark.executor.memory可以直接指定，同理spark.driver.memory可以指定driver内存。

#### 注意事项

### JDK内存问题

#### 问题描述

手工部署Apache Spark on Yarn，出现yarn-cluster模式正常运行，但是yarn-client模式总是异常退出。

#### 解决方案

不知道啥原因，添加下面配置到yarn-site.xml后解决问题。

```xml
<property>
    <name>yarn.nodemanager.pmem-check-enabled</name>
    <value>false</value>
</property>

<property>
    <name>yarn.nodemanager.vmem-check-enabled</name>
    <value>false</value>
</property>
```