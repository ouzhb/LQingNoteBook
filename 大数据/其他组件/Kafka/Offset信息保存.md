---
title: Kafka 如何保存 Offset
categories: "大数据" 
date: 2019-04-1
comments: false
toc: true
tags:
	- Kafka
---

说明Kafka如何保存 Groups 的 Offset 信息。

<!--more-->

# Offset保存机制

老版本Kafka中，Group的Offset信息保存在Zookeeper的/consumers/{group}/offsets/{topic}/{partition}目录下。此时消费者一般是使用kafka.javaapi.consumer.ConsumerConnector进行消费的，用户在 param 中配置zookeeper.connect。这种情况下，如果 Kafka 集群规模庞大会给 Zookeeper 造成比较大读写负担。

新版Kafka中（具体怎么新不太清楚~~~），消费者如果使用 org.apache.kafka.clients.consumer.KafkaConsumer 消费数据，Offset信息会保存在一个 Kafka 自带的 topic（__consumer_offsets）中。这种方式下 Offset 信息序列化后保存在本地。

默认情况下，__consumer_offsets 有50个分区。Group 将 group.id 哈希取模后保存到 __consumer_offsets 的对应分区中。

通过以下命令可以查看__consumer_offsets：

```shell
# describe topic
kafka-topics --describe --zookeeper localhost:2181 --topic __consumer_offsets

# 消费topic
kafka-console-consumer --bootstrap-server bdnode1:9092,bdnode2:9092,bdnode3:9092 --topic __consumer_offsets --formatter "kafka.coordinator.group.GroupMetadataManager\$OffsetsMessageFormatter" --consumer-property exclude.internal.topics=false --from-beginning
```

通过 kafka-consumer-groups.sh 可以管理 Offset 信息：

```shell
# 查询指定Group的Offset信息
kafka-consumer-groups --bootstrap-server bdnode1:9092,bdnode2:9092,bdnode3:9092 --group KafkaConsumerOnSpark --describle 
# 重置Offset
kafka-consumer-groups --bootstrap-server bdnode1:9092,bdnode2:9092,bdnode3:9092 --group KafkaConsumerOnSpark --reset-offsets   --topic my_topic --to-earliest
# 删除Offset（注意删除前需要重置）
kafka-consumer-groups --bootstrap-server bdnode1:9092,bdnode2:9092,bdnode3:9092 --group KafkaConsumerOnSpark --delete

```
kafka-consumer-groups.sh 脚本的功能比较强大可以，按照需求修改Offset信息。

# Spark Streaming Kafka

Spark 项目提供了[0.8](https://spark.apache.org/docs/2.4.0/streaming-kafka-0-8-integration.html)和[0.10](https://spark.apache.org/docs/2.4.0/streaming-kafka-0-10-integration.html)两个版本的 Kafka 集成插件。

目前这个两个版本的代码在[Spark的源码](https://github.com/cloudera/spark/tree/cdh6.1.0/external)中都还在维护，但是0.8版本的许多接口已经不再维护。

两个版本提供的 Offset 更新接口也有所差异：

Spark 1.6.3 之前的版本中，0.8版本提供KafkaManager类，通过这个类可以将Offset信息更新到Zookeeper中，新版本该Class已经没有了~~！

Spark 2.0.0以上的版本中，1.0版本可以通过以下方式，更新Groups的Offset信息：

```scala
stream.foreachRDD { rdd =>
  val offsetRanges = rdd.asInstanceOf[HasOffsetRanges].offsetRanges

  // some time later, after outputs have completed
  stream.asInstanceOf[CanCommitOffsets].commitAsync(offsetRanges)
}
```