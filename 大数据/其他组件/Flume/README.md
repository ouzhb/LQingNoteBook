# Apache Flume

Apache Flume是一个分布式，高可用的系统，用于有效地从不同的Source收集，聚合和移动大量日志数据到集中式数据存储。使用场景包括：数据聚合、RPC、网络流等场景。

 ![Flume](https://flume.apache.org/_images/UserGuide_image00.png)

Flume的主要组件成为Flume-agnet，由以下部分组成：

- Source：获取外部消息(Event)

- Channel:临时缓存为被消费的的Event，Channel可以是一个本地文件系统

- Sink：消费Channel的Event，将其转发到下一跳的Agent，或者将其存入外部存储（terminal repository）

其他子模块：
- Channel Selectors： Selectors本质上是一个Source的一个内部模块，用来决定Source写入Event的Channel。Flume原生提供了多个Channel Selector可以选择；
  
- Sink Processors：Sink Processors将多个sink聚合成一个Groups，在这个Groups中Sink通过配置Sink Processors可以实现负载均衡、Failover等功能。

- Event Serializers：Event序列化模块，主要使用场景是HDFS Sinks和File Sinks

- Flume Interceptors：Soure 支持配置多个串行的拦截器，用来过滤Event，以及对Event进行轻量级的修改。

Flume支持以下功能：
- 支持Agent级联，Agent内部Fan-In/Out配置；
- 通过properties、[Zookeeper](https://flume.apache.org/FlumeUserGuide.html#zookeeper-based-configuration)进行配置
  
## 安装部署

Flume只需要启动通过 flume-ng 命令启动相关Agent即完成了部署，官方提供了以下示例：通过一个单点的agent，监听指定端口信息，输出到flume日志。


```properties
# example.conf: A single-node Flume configuration

# 定义Agent关联的sink、source、channels
a1.sources = r1
a1.sinks = k1
a1.channels = c1

# 定义sink
a1.sources.r1.type = netcat
a1.sources.r1.bind = localhost
a1.sources.r1.port = 44444

# 定义source
a1.sinks.k1.type = logger

# 定义channels
a1.channels.c1.type = memory
a1.channels.c1.capacity = 1000
a1.channels.c1.transactionCapacity = 100

# 定义Agent内部的拓扑关系
a1.sources.r1.channels = c1
a1.sinks.k1.channel = c1
```

通过下面的命令启动Agent：
```shell
# --name 表示要启动的agent名称
# --conf 指向配置文件目录
# --conf-file 指向properties配置文件
# -Dflume.root.logger=INFO,console 则是注入到JVM中的环境变量

bin/flume-ng agent --conf conf --conf-file example.conf --name a1 -Dflume.root.logger=INFO,console
```

## 参考

[Flume Sources](https://flume.apache.org/FlumeUserGuide.html#flume-sources)

[Flume Sinks](https://flume.apache.org/FlumeUserGuide.html#flume-sinks)

[Flume Channel Selectors](https://flume.apache.org/FlumeUserGuide.html#flume-channel-selectors)

[Flume Sink Processors](https://flume.apache.org/FlumeUserGuide.html#flume-sink-processors)

[Flume Interceptors](https://flume.apache.org/FlumeUserGuide.html#event-serializers)

[美团的Flume日志收集系统 - 架构和设计](https://tech.meituan.com/2013/12/09/meituan-flume-log-system-architecture-and-design.html)

[美团的Flume日志收集系统 - 改进和优化](https://tech.meituan.com/2013/12/09/meituan-flume-log-system-optimization.html)
