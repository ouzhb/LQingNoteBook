# IoT 存储系统调研分析

在IoT系统中，设备通常只有有限的数据存储能力，因此 IoT 设备所获取的大部分数据一般需要使用[通信协议](https://www.ibm.com/developerworks/cn/iot/library/iot-lp101-connectivity-network-protocols/index.html)（进行通信上传到云端，由云端的IoT系统进行分析处理。

常见的IoT系统可能需要处理以下类型的数据：

- 离散传感器读数

- 半结构化数据，如：设备运行状况等，

- 图像或视频文件

上述这些数据的结构类型并不统一，需要存储在不同类型的存储系统 ，因此 **IoT 数据的存储系统**并不是一成不变的，需要结合实际情况选择合适数据存储方案。

以下存储系统可以作为IoT系统的数据存储：

- **时间序列数据库**：专为对基于时间的数据建立索引和进行查询而设计，用来非常适合用于存储传感器数据。
- **NoSQL数据库**：可以用于存储半结构化数据。相比于SQL，这些数据库有**高吞吐量**、**低延迟**、**可扩展**的优势。
  
- 文件存储：存储大容量的归档数据，或者图像或者文件数据

## 1、 时序数据库

以下是能从DB-Engine、以及Google中找到的有一定影响力的方案：

- InfluxDB（功能完善，性能卓越，坑最少，但缺少集群方案）
	- 性能优秀，被用于各种监控指标采集的场景，几乎可以认定是当前使用最多的TSDB，并且常年在DB-Engin的TSDB类评比中遥遥领先第二名；
	
	- InfluxDB本身的代码不开源，因此官方只在商业版中提供集群方案。当前网络上能够找到的集群方案包括：
	  
	  - [influxdb-relay](https://github.com/strike-team/influxdb-relay)：这是早先官方提供的高可用方案，已经有多年没有更新（该方案只是简单的进行数据双写，没有考虑分片、数据一致性等要求，属于非常简单的HA方案）。
	  - [influx-proxy](https://github.com/shell909090/influx-proxy)：饿了么的开源的方案，大概是在influxdb-relay的基础上进行了修改。但目前饿了么开源了自己的时序数据库，该项目也已经没有维护。
	  
	- 使用 InfluxDB 完整的生态栈包括（三个组件均是开源的）：
	  
    - Telegraf：开源时序数据收集器，将各种类型的数据转换为时序数据发送给InfluxDB
	- Chronograf：类似Granfna，Chronograf是Influxdata公司推出的图形化工具
	  
	  - Kapacitor：同样是Influxdata出品，是一个基于influxdb的事件处理以及告警引擎，能够根据建立的规则对异常时序数据进行报警，同时能够将这些告警推送给其他系统
	  
	- 当前阿里云提供托管高可用Influxdb
- [Prometheus](https://prometheus.io/)（性能OK，社区活跃，云原生，但是比较偏向监控场景！）
- Prometheus本身不只是一个数据库，而是一个完整的**监控框架**，拥有完善的告警、采集配套工具。目前，在一些Paas平台上Prometheus被广泛使用，社区非常活跃。
  - 主要工作在Pull模式下，可能不太适合一些高频数据采集场景
  - **原生不支持集群**，但是有非常多的第三方方案。总体来说相比于InfluxDB，Prometheus的集群方案可能更容易达成。
    - [Cortex](https://github.com/cortexproject/cortex)：为Prometheus提供可扩展的长期存储，并且是 CNCF 的孵化项目
    - [thanos](https://github.com/improbable-eng/thanos)：专门为Prometheus设计的集群方案，提供了全局查询、降准采样等能力、以及数据转存到对象存储的能力。
- [Graphite](https://db-engines.com/en/system/Graphite)（相比上面两个毫无优势，pass）

  - 同样是一个完整的监控框架，是历史比较悠久的方案，有一定的用户基数。整个方案包括以下模块：

    - Graphite-web：对标Grafana

    - Carbon：数据监听器，Carbon 守护程序会监听指标数据发送给后端存储

    - Whisper： 后端存储，本质上是时序文件块
  - 从网络上一些博文上看，由于性能、易用性、可维护性等原因，Graphite已经逐渐被其他开源工具替代。
- [Druid](https://druid.apache.org/docs/latest/querying/timeseriesquery.html)（如果数据规模非常大，Druid 是Influxdb的替代方案）
- Druid 是一个 LDAP 类型的数据库系统
  - Druid 的优势是聚合操作，同时提供非常高的实时响应能力，因此是适合存储时间序列数据库的
  - Druid 目前已经支持本地磁盘，NFS挂载磁盘，HDFS，S3等作为数据存储
  - 总归是LDAP类型的分析引擎，并发能力比较捉鸡！！
- [OpenTSDB](http://opentsdb.net/)（分布，性能OK，但架构比较笨重，个人认为没得选了才会选他）

  - 基于 HBase 时序数据库
  - Github上OpenTSDB的热度比较低，最新版本已经是2018年的版本了
- [TimescaleDB](https://docs.timescale.com/)（SQL支持完善，个人认为在一些数据量不大的场景可以使用）
- TimescaleDB是工作在Postgresql上的一个插件，提供了非常全面的SQL语法，那查询时序数据。
  - 存储架构是完全基于PostgresSQL的，因此可以使用PostgresSQL的Replice方案HA
  - 当前提供 [Hypertables](https://docs.timescale.com/clustering/getting-started/scaling-out) 方案实现分布式表功能，从而实现对表很向扩容。但是这个功能当前还是Beta版本
- [kairosdb/kairosdb](https://github.com/kairosdb/kairosdb)（pass，不考虑使用）

  - 类似OpenTSDB，是运行在Cassandra 上数据数据库插件，相比OpenTSDB社区更为活跃。但是相对了完善程度也不如OpenTsdb
- [Riak TS](https://docs.riak.com/riak/ts/latest/basho.com/products/riak-ts/)（数据规模不大时，可以作为一个备选方案）
- 比较新的一个开源时序数据库，按照官方网站的介绍是一个企业级的IoT数据库
  - 分为开源版和商业版，但开源就具有HA和集群功能，商业版主要卖技术支持和异地灾备服务
  - 支持SQL语法
  - AWS 提供托管RiakTS数据库（这一点说明这个东西可能不会太坑）
- [IoTDB](https://iotdb.apache.org/)（功能比较完善，但是新东西，基本没啥人用过，估计坑很多）

  - 清华大学开源时序数据库，当前已经是Apache 孵化项目
  - 官方介绍是一个轻量级的、高性能的时序数据（自己说性能超过Influxdb ）
  - 支持SQL、Spark、Hive集成的功能
  - 目前似乎还是一个单机数据库，但是有一个实验功能可以将本地文件备份到HDFS上
  - 集群分片功能，当前在开发中
- [CrateDB](https://github.com/crate/crate)（功能貌似很全面 ～～）
- 底层是基于ES的分布式
  - 支持类SQL的语法，能够作为关系型数据库、TSDB、文档数据来使用
  - 支持HA、分片的功能
- [M3DB](https://m3db.github.io/m3/)
- Uber开源的时序数据库
  - 具有分布式、可扩展能力
  - 可以和Prometheus一起使用，成为其后端存储方案
- [lindb](https://github.com/lindb/lindb)（一眼望去全是坑Pass!!）
- 饿了么最近才开源的分布式时序数据库（估计是19年才开源的），暂时还没有稳定版本发布。
  - 从Github上的主页来看，似乎饿了么推动这个项目的热情不高
  



**总结**：

1. 当前开源的时序数据库方案非常多（估计能超过30个），其中不少具备HA和高可用能力。

2. 由于我们实际上没有业务深度使用时序数据库，很难准确把握自己需要，从而选择一个比较好的方案。因此，我决定凭感觉找几个靠谱的调研、试用。

   - 数据规模不大，可能会作为一个组件部署在客户的环境中
     - CrateDB
     - Prometheus / M3DB 或者 Prometheus 的其他HA方案
     - Riak TS
     - TimescaleDB

   - 公有云场景中，数据规模非常巨大
     - Durid

3. 对一个TSDB进行试用包括以下工作

   - 比较详细的了解特性 - 输出文档
   - 安装部署测试环境 ，了解相关的Client SDK - 输出文档
   - 进行压力测试 - 输出测试报告



## 参考

[了解 IoT 数据](https://www.ibm.com/developerworks/cn/iot/library/iot-lp301-iot-manage-data/index.html)