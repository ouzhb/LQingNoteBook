---
title: Waterdrop
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-07-01
comments: false
toc: true
tags:
	- ETL
	- 南北向接口工具
---

关于ETL工具，已经南北向接口工具的调研

<!--more-->

# Waterdrop

Waterdrop 是一款开源ETL工具，两个作者分别供职于新浪和一下科技。当前该项目在github上的有将近500个Star，并且在新浪、水滴筹、永辉等实际生产环境上有相关应用。

Waterdrop包括以下特点：

- 架构简单，其工作流只包含：Input、Filter、Output三个部分。每个部分都是基于 Spark 或者 Flink 现成代码段。部署Waterdrop应用时，通过配置文件自定义Pipline，并提交到Spark集群上运行。

- 包含Spark和Flink两个版本。

- **方便易用**，包含众多现成plugin，并可以根据实际业务进行定制。

## 核心逻辑

**Row**：Row 是Waterdrop逻辑意义上一条数据，是数据处理的基本单位。在Filter处理数据时，所有的数据都会被映射为Row。在代码实现中，数据实际上被组织成Dataset[Row]

**Field**：Field是Row中的一个字段，Row可以包含嵌套层级的字段，其中，raw_message 指从Input获得的原始数据。Row中最顶级的字段层级用root表示。



## 配置文件示例

在Waterdrop中数据的流向由配置文件决定。

Waterdrop只支持非常简单的pipline拓扑，其中filter只支持串行，Input到filter支持多个Input扇入，filter到Output支持多个Output扇出


```shell

# 该部分用于进行Spark参数的配置
spark {  
    ...
}

# 配置input插件
input {
    ...
}

# 串行配置多个filter插件
filter {
    ...
}

# 配置多个output插件
output {
    ...
}

```



# 插件

## Input

- 支持File、HDFS、S3上的文件流读取，支持Orc、Parquet、XML等格式；
- 支持 KafkaStream、ElasticSearch；
- 支持JDBC、Kudu、Mongdb、MySQL、Hive、Tidb等数据库表数据的抽取；
- 支持SocketStream；

## OutPut

- 支持Clickhouse、JDBC、Kudu、MongoDB、MySQL、Opentsdb、Tidb
- 支持File、HDFS、S3
- 支持Kafka、ES
- 支持Stdout


## Filter插件

- 解析固定格式的记录生成一张表
    - json
    - Grok
    - KV（解析URL中的KV参数）
    
- SQL类操作
    - join(inner join两张表)
    - **SQL（执行一句SQL）**
    - **Table：将文件映射成一张表**

- 对记录中字段的操作
    - Checksum（计算字段校验码）
    - Convert（字段类型转换）
    - Date（解析时间格式）
    - Lowercase/Uppercase（将指定字段内容全部转换为小/大写字母）
    - Replace（字段正则替换）
    - split（分割一个字段为多个字段）
    - Truncate（截断字段）
    - **Script操作**（基于QLExpress执行指定脚本逻辑。脚本接收一个指定JSONObject（可以认为是一条记录）, 完成自定义的处理逻辑，再返回一个新的event（记录）。
- 对记录的操作：
    - Add（添加字段）
    - Remove（删除字段）
    - Drop（删除行）
    - Rename（重命名字段）
    - Sample（数据采样）
    - Uuid：为每行记录添加一个全局唯一的UUID

- Repartition：调整数据分区数

- Watermark：Spark Structured Streaming Watermark



# 总结

- 功能过于简单，只能完成一些**数据导入导**出工作。

- 与CDH-6.1.0平台存在jar包冲突，需要重新编译才能在CDH-6.1.0上工作。

- 缺乏HA能力；

- 缺少监控功能（有监控组件，但是收费版本的功能）；


# 参考

[官方网站](https://interestinglab.github.io/waterdrop/#/zh-cn/)