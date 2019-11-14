# Elasticsearch

## 基础概念

- ES是NRT存储平台，能够提供秒级延时的数据库操作。
- Cluster的名称必须是唯一的，在同一个环境中可以部署多个集群。
- Node同样需要一个唯一的UUID，可以在启动时自动分配，也可以由用户来指定。
- Index：Elasticsearch索引是一组具有共同特征的文档集合。每个索引(index)包含多个类型(type)，这些类型依次包含多个文档(document)，每个文档包含多个字段(Fields)。在ELK中，当logstash的JSON文档被发送给Elasticsearch时，它们被发送为默认的索引模式“logstash-%{+YYYY.mm.dd}”。它按日划分索引，以便在需要时可以方便地搜索和删除索引。这个模式可以在日志存储的输出插件中改变。
- type ：类型用于在索引中提供一个逻辑分区。它基本上表示一类类似类型的Documents。之前版本一个Index内可以存储不同type的Documents，但是现在的Index中只能保存同类型的Document，后续type的概念会被完全移除。
- Document：Elasticsearch文档是一个存储在索引中的JSON文档。
- Mapping：映射用于映射文档的每个field及其对应的数据类型，例如字符串、整数、浮点数、双精度数、日期等等。在索引创建过程中，elasticsearch会自动创建一个针对fields的映射，并且根据特定的需求类型，可以很容易地查询或修改这些映射。
- Field：文档内的一个基本单位,键值对形式
- Shards & Replicas：
  - 单个Index存储的数据可能超过了单台物理机器的限制，Shards表示Index数据的切分。创建Index时可以手工指定shards的数目。
  - 每个Shards本身都是一个功能齐全且独立的“Index”，可以托管在集群中的任何节点上。
  - es通过shards提供了横向扩展能力，以及并行的吞吐。 
  - 多个shards如何聚合数据，并传输给用户。这个操作对用户都是透明的。
  - replica 是shards的副本，用来进行容灾。
  - replica永远不会在与从中复制的原始/主分片相同的节点上分配。
  - replica除了提供容灾的能力外，同样提供并行吞吐的能力。
  - 提供了replica能力之后，es区分primary shards和replica shards。
  - shards、replica 在创建index后可以进行修改。
  - 单个shards中documents数目为2,147,483,519（20亿，Integer.MAX_VALUE - 128）

- Node 在Cluster中可以通过[HTTP](https://www.elastic.co/guide/en/elasticsearch/reference/5.5/modules-http.html)和[Transport](https://www.elastic.co/guide/en/elasticsearch/reference/5.5/modules-transport.html)进行通信。Transport主要用于集群内部的节点通信，HTTP主要用于外部的REST Client进行通信。
- Node了解所有其他Node的地址，并且能将客户端请求转发给其他的的Node。
- 单个Node中可以运行一下几种服务：
  
|Node类型|节点标志|说明|节点工作|
|-----|-----|-----|-----|
| Master-eligible|node.master=true|可以被选举为master的node。|Master节点负责**轻量级**群集范围的操作，例如创建或删除索引。Master同样需要具有data/目录的权限，用来存放cluster的元数据。|
| Data-node|node.data=true|存储数据，执行CRUD, search, and aggregations的节点。||
| Ingest node|node.ingest: true && search.remote.connect: false |用于[ingest pipline](https://www.elastic.co/guide/en/elasticsearch/reference/5.5/pipeline.html)的节点。|执行pipelines预处理的节点，由一个或多个ingest processors组成。| 
|Coordinating node||该类型的节点只能对请求进行rounte、处理请求的reduce phase、distribute bulk indexing|对于高负载的大型集群Coordinating node非常有用，但是过多协调节点会增加|
| Tribe-node||一种特殊类型的协调节点，可以连接到多个集群并在所有连接的集群中执行搜索和其他操作。|


- [X-Pack node settings](https://www.elastic.co/guide/en/elasticsearch/reference/5.5/modules-node.html#modules-node-xpack)  