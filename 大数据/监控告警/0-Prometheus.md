---
title: 数据库调研笔记 -- Prometheus
categories: "笔记"
date: 2019-07-16
comments: true
toc: true
tags:
	- 数据库
	- Prometheus
---


数据库调研笔记 -- Prometheus


<!--more-->

# Prometheus

Prometheus 是SoundCloud公司开源的一款监控框架，提供包括：告警工具、时序数据库、图表展示等功能，主要包括以下Feature：

- 支持PromQL来查询数据
- 支持多维时间序列数据存储
- 集群不依赖共享存储，单个服务器节点是自治的
- 支持Pull、Push（通过gateway组件）两种方式获取数据
- 支持动态发现的模式配置服务
- 高效：单一实例可以处理百万个监控指标、每秒处理数十万的数据点

Prometheus主要包括： 

- Prometheus server：提供TSD能力，以及Pull数据的能力，以及基于PromQL的查询
- client libraries：多种语言的客户端接口
- Push gateway：数据网关，服务将数据Push到gateway，Server从gateway pull数据
- exporters：Exporter将监控数据采集的端点通过HTTP服务的形式暴露给Prometheus Server。一些符本身及支持Prometheus，因此这些服务天生就是exporters，此外还有的exporters以插件的形式被安装到client端。
- alertmanager：告警模块

![](https://prometheus.io/assets/architecture.png)

# 概念

## Data Model

Prometheus的数据模型基于time series， 由一个Metric名称和若干labels构成，表示为:
```
<metric name>{<label name>=<label value>, ...}**
```
Prometheus 基于time series存储数据，并且可能会因查询而生成临时派生时间序列。


## Time-Series

Time-Series由若干样本组成，每个Sample包含以下三个部分：

- 指标（metric）：metric name + 描述当前样本特征的labelsets
- 时间戳（timestamp）：一个精确到毫秒的时间戳
- 样本值（value）：一个folat64的浮点型数据表示当前样本的值

以下几点可以注意：

- metric name实际上是一个特殊的label，名称为__name__，以“__”作为前缀的标签是系统内置标签
- 形式上所有metric格式为：<\metric name\>{\<label name\>=\<label value\>, ...}
- 一个Time-Series包含多个metric

## Metric Types

Prometheus当前只支持四种数据类型，并且在不同的API中稍有区别：

- Counter：只增不减的计数器，只能在重启时将其设置为0
- Gauge：常规测量值类型
- Histogram：直方图统计类型
- Summary：类似Histogram，区别[参考](https://prometheus.io/docs/practices/histograms/)

Histogram 和 Summary 本质上都是统计指标，不会保存原始数据。

### Histogram

Histogram 指标对每个采样点进行统计，并根据bucket将数据划分到不同区域。

假设bucket配置为[1,5,10], Prometheus观察到的数据被划分到[0,1],[1,5],[5,10]，[5,+inf] 四个不同的区间。

对每个Histogram 可以获取以下信息：

```
[basename]_bucket{le=“1”} # 观测小于1的次数
[basename]_bucket{le=“5”}   # 观测小于5的次数
[basename]_bucket{le=“10”}   # 观测小于10的次数
[basename]_count         # 所有观测值的个数
[basename]_sum   # 所有观测值之和
```

Prometheus 只会存储每个区间内观测值的个数，并且histogram_quantile使用分段线性近似的方式绘制分布曲线。

### summary

summary是精度更高的直方图测量，可以获取以下内容：

```
1. [basename]{quantile="φ"} (0 ≤ φ ≤ 1) # 获取百分之φ数据的最小值，基于第三方库perk
2. [basename]_sum   #指所有观察值的总和
3. [basename]_count #指已观察到的事件计数值
```

## JOBS AND INSTANCES

 Instances 表示能够pull到数据的一个endpoint通过 Host:Port 表示，多个类型一致 Instances 组成 job。

 在Prometheus拉取数据时，会自动为time series添加job和instance的名称到labels，如下：

```
job: api-server
	instance 1: 1.2.3.4:5670
	instance 2: 1.2.3.4:5671
	instance 3: 5.6.7.8:5670
	instance 4: 5.6.7.8:5671
```


# 安装

[官方](https://prometheus.io/docs/prometheus/latest/installation/)提供了包括：二进制安装包、Docker、Ansible等多种安装方式。

[Exporters](https://prometheus.io/docs/instrumenting/exporters/)

Prometheus 使用命令行和配置文件指定服务的配置项，其中：

- 命令行参数：指定数据目录、memory配置等服务配置，使用./prometheus -h可以查看相关配置。
- 配置文件：配置文件基于yaml格式，用于指定jobs、instances、rules，启动时通过--config.file指定配置文件，官方提供配置项有[详细说明](https://prometheus.io/docs/prometheus/latest/configuration/configuration/#configuration-file)，以及[模板](https://prometheus.io/docs/prometheus/latest/configuration/template_examples/)。

Prometheus支持动态刷新配置，用户可以通过向进程发送 SIGHUP 或者 HTTP POST 方式重载配置项。

```
# 执行配置文件动态刷新

kill -HUP {pid}
curl -X POST http://localhost:9090/-/reload
```

比较关键的启动配置包括：

- storage.tsdb.path：决定数据目录的存储位置
- config.file：读取的配置文件位置

## QuickStart

官方网站的QuickStart展示了使用Node Exporter采集主机数据的例子。

Node Exporter是Golang编写，主机系统测量值采集工具，不存在任何的第三方依赖，只需要[下载](https://github.com/prometheus/node_exporter)后即可直接运行。

主机启动Node Exporter命令后，会在 9100 端口暴露采集到的监控数据，每次metrics请求会返回多个指标，其格式如下：

```shell

#PS:每个指标的开头包含HELP和TYPE两行，HELP解释指标的内容、TYPE解释指标的类型
#PS:每个指标{}中的内容是lable信息，

# HELP node_cpu Seconds the cpus spent in each mode. 
# TYPE node_cpu counter
node_cpu{cpu="cpu0",mode="idle"} 362812.7890625
# HELP node_load1 1m load average.
# TYPE node_load1 gauge
node_load1 3.0703125
```

以下配置文件中，包含了两个job：prometheus是服务自身的监控，node是我们定义的主机运行情况监控，共包含三个instance。

重启prometheus后，在console页面执行up函数可以看到每个instance的运行情况（1表示正在运行）。

```yaml

global:
  scrape_interval:     15s # Set the scrape interval to every 15 seconds. Default is every 1 minute.
  evaluation_interval: 15s # Evaluate rules every 15 seconds. The default is every 1 minute.
  # scrape_timeout is set to the global default (10s).

# A scrape configuration containing exactly one endpoint to scrape:
# Here it's Prometheus itself.
scrape_configs:
  - job_name: 'prometheus'
    static_configs:
    - targets: ['localhost:9090']
  
  - job_name: 'node'
    scrape_interval: 1s
    static_configs:
      - targets: ['bdnode1:9100','bdnode2:9100','bdnode3:9100']

```


# PromQL

## 查询时间序列
查询时间序列的基本格式为：

```
metric_name{label_name OP label_value, ...}[duration] offset offset time
```

- OP可以支持完全匹配、正则匹配，允许以下符号：=、!=、=~、!~
- duration表示访问指定时间范围的数据，尺度为s、m、h、d、w、y
- offset表示从当前时间向前位移

## 聚合、操作符运算

Prometheus支持对序列进行聚合、以及运算符操作。

### 运算符操作

运算符操作包括：数学运算符，逻辑运算符，布尔运算符。

运算符的操作对象包括：序列、标量

- 数学运算符：+、-、*、/、%、^
- 布尔运算符：=、！=、>、<、>=、<= 将过滤掉不满足条件的采样，这些运算符用bool修饰之后，不会进行采样过滤，而是返回0或1
- 逻辑运算符：and、or、unless 根据情况产生两个时间序列的交集、并集、补集

需要注意的是：

- 操作符之间存在优先级
- 序列的运算遵循匹配规则：
	- 标签完全一致的元素之间进行计算，没找到匹配元素，则直接丢弃。
	- 通过on、ignoreing可以限定匹配的标签、或者忽略特定标签。
	- 当出现一对多、一对一匹配时，可以通过group运算符使唯一值被匹配多次。

### 聚合函数

基本用法：

```
<aggr-op>([parameter,] <vector expression>) [without|by (<label list>)]

# without：表示排除特定标签
# by：只考虑特定标签
```
支持的聚合函数包括：

- sum (求和)
- min (最小值)
- max (最大值)
- avg (平均值)
- stddev (标准差)
- stdvar (标准差异)
- count (计数)
- count_values (对value进行计数)
- bottomk (后n条时序)
- topk (前n条时序)
- quantile (分布统计)

### 其他内置函数

- increase(v range-vector)：返回区间的增长量
- rate(v range-vector)：返回区间的平均增长率
- irate(v range-vector)：返回区间的瞬时增长率
- predict_linear(v range-vector, t scalar)：基于简单线性回归，预测序列在t秒后的值
- histogram_quantile(0.5, http_request_duration_seconds_bucket)：计算histogram的分位数值
- label_replace：动态替换标签


## 例子

```shell
{instance="bdnode3:9100"}                           # 返回节点3的所有量测信息
node_hwmon_temp_celsius                             # 查询所有CPU核心的温度
node_hwmon_temp_celsius{chip="platform_coretemp_0"} #所有节点CPU0上的核心温度
node_hwmon_temp_celsius{chip="platform_coretemp_0",instance="bdnode3:9100"}[10s] offset 30s                  # 之前40s到30s的温度数据
avg(node_hwmon_temp_celsius{chip=~"platform_coretemp_[0-9]*" }) by (instance)         #查询每个节点CPU的平均温度
avg(node_hwmon_temp_celsius{chip=~"platform_coretemp_[0-9]*" }) by (instance,chip) #查询每个节点的所有CPU的平均温度
```

# 存储

### 本地存储
对于本地存储，Prometheus 2.x 采用自定义的存储格式将样本数据保存在本地磁盘当中。

Prometheus按照两个小时为一个时间窗口，将两小时内产生的数据存储在一个块(Block)中，每一个块中包含该时间窗口内的所有样本数据(chunks)，元数据文件(meta.json)以及索引文件(index)。

写入数据时，Prometheus先写内存，并且通过WAL进行重播进行数据会，API对数据删除同样通过tombstone进行标记删除。

Prometheus或周期合并时间窗内的数据，并删除垃圾数据。

### 远程存储Cortex

远程存储的目的是要使Prometheus**存储大量历史数据**、以及进行**灵活扩展**和**迁移**。

通过remote_write/remote_read，Prometheus可以将数据存放到其他[存储服务](https://prometheus.io/docs/prometheus/latest/storage/#remote-storage-integrations)中

远程存储是一种附加功能（可以和本地存储同时工作），部分存储服务Prometheus只支持写入，不支持读取，作为一种长期数据备份工具。

当前支持的远程存储包括：

|Storage|操作|备注|
|----|----|----|
|[Cortex](https://github.com/cortexproject/cortex)|读写|这个项目专门为Prometheus提供可扩展的长期存储，并且是 CNCF 的孵化项目github上有1500+的star|
|[M3DB](https://m3db.github.io/m3/)|读写|Uber开源的分布式时序数据库,github上有两千多个stars，支持PromQL和Prometheus本身有非常好的兼容性|
|[thanos](https://github.com/improbable-eng/thanos)|写入|专门为Prometheus设计的集群方案，提供了全局查询、降准采样等能力、以及数据转存到对象存储的能力。Github上有4000+star|
|[VictoriaMetrics](https://github.com/VictoriaMetrics/VictoriaMetrics)|写|开源分布式时序数据库，支持 PromQL 接口、号称相比TimeScale和Influxdb相比有20以上的性能|
|[Chronix](https://github.com/ChronixDB)|写入|基于Lucene的一款时序数据库，支持单机集群方式部署。单机模式基于纯Lucene实现可以嵌入到应用程序中，集群模式基于solr实现分布式能力。同时该项目有Spark接口。|
|[CrateDB](https://github.com/crate/crate)|读写|分布式数据库，主要的应用场景是IoT场景（智能工厂、智能驾驶等等），涵盖了一整套IoT场景的平台方案（平台不是开源的）。|
|[irondb](https://github.com/pruthvikar/irondb)|读写|一个连官网都没有、Github上只有五个Star的keyvalue数据库|
|[SignalFx](https://signalfx.github.io/)|写|SignalFx本身也是一个监控平台|
|Wavefront|写入|VMware的企业级监控平台|
|Splunk|读写|商业日志分析工具|
|AppOptics|写入|商业APM工具，用在应用性能监控场景|
|Gnocchi|写入|从OpenStack的孵化出的项目，本质上是一个中间件，用来将metric数据进行时序封装，并写入到后端存储中。|
|[graphite](https://graphiteapp.org/)|写入|时序数据库、图形渲染层。这个工具只被动收集数据，并且图形渲染能力较弱一般和Grafana配合使用。|

其他开源组件存储包括：Elasticsearch、InfluxDB、Kafka、OpenTSDB、TiKV、PostgreSQLTimescaleDB


# 集群模式

目前，Prometheus并没有一套非常完整的集群方案，仅能够通过联邦的方式解决部分问题。

当前可以采用的

- 基本HA方式：部署多套Prometheus，并且采集相同的Server实例。这种方式只能解决服务可用性的问题，无法解决一致性、故障恢复、动态扩展等问题，只适合：**小集群、不进行迁移**的场景。
- 基本HA方式 + 远程存储：Promthus Server将数据保存在远程服务上，并且部署多个实例，用NGINX路由请求。
- 基本HA + 远程存储 + 联邦集群
- 其他集群方案，如thanos

# 常用管理员命令

```shell

# 删除数据
curl -X POST \
  -g 'http://172.24.33.31:9090/api/v1/admin/tsdb/delete_series?match[]={job="node"}'
# 触发压缩操作
curl -XPOST http://localhost:9090/api/v1/admin/tsdb/clean_tombstones

# 重载配置文件
kill -HUP {pid}

```


# 参考

[官方文档](https://prometheus.io/docs/prometheus/latest/getting_started/)

[Prometheus-book](https://yunlzheng.gitbook.io/prometheus-book/)

[FUNCTIONS](https://prometheus.io/docs/prometheus/latest/querying/functions/)

[HTTP API](https://prometheus.io/docs/prometheus/latest/querying/api/)

