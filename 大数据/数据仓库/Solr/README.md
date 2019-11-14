---
title: Solr 学习笔记 
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-04-11
comments: false
toc: true
tags:
	- Solr
---

Solr的学习笔记 

<!--more-->


# 安装部署

## 使用脚本安装Standalone模式

Solr安装包的bin目录下提供了install_solr_service.sh脚本，通过该脚本能够在节点上安装Solr服务。

```shell
./install_solr_service.sh solr-7.7.1.tgz

#############################################
#
#   1. 脚本将solr-7.7.1.tgz，默认解压到/opt目录，并且创建软连接/opt/solr。
#   2. 使用/var/solr作为solr的数据目录，该目录下包括solr数据、日志。
#   3. 安装完成后使用 /etc/init.d/solr 启停solr服务，该脚本使用solr用户启动服务（或者用service，使用systemctl似乎有一点问题）。
#   4. 默认使用8983作为Solr的端口
#   5. 默认情况下solr配置文件为 /etc/default/solr-demo.in.sh ，该文件中包含ZK、Hadoop等信息的定义。
#   6. /var/solr/data是默认情况下的主目录，该目录下包含：zoo.cfg和solr.xml。
#   7. /opt/solr/bin/oom_solr.sh 该脚本用来杀死oom状态的Solr进程。
#   8. solr的安装信息都固化在/etc/init.d/solr中，通过这个文件可以修改配置文件位置信息。
#
#############################################

```
## SolrCloud模式

当在/etc/default/solr-demo.in.sh 中配置 ZK_HOST 时，Solr通过Cloud的模式启动。

```shell
# 使用以下命令，可以打开一个交互式的shell，可以根据引导创建一个SolrCloud集群,所有Solr实例运行在一个物理机上
bin/solr -e cloud

# 下面的链接详细说明了如何使用solr命令创建一个solrcloud集群
## https://lucene.apache.org/solr/guide/7_0/getting-started-with-solrcloud.html#getting-started-with-solrcloud
```

# 简单使用

官方Quick Start展示了使用预置techproducts的一些简单用法：

Exercise 1

```shell
bin/post -c techproducts example/exampledocs/* # 使用post命令导入数据

# 搜索任意Document
http://localhost:8983/solr/techproducts/select?indent=on&q=*:*
# 单个短语搜索
http://localhost:8983/solr/techproducts/select?q=foundation
# 只返回id字段
http://localhost:8983/solr/techproducts/select?q=foundation&fl=id
# 搜索指定字段
http://localhost:8983/solr/techproducts/select?q=cat:electronics
# 搜索短语，这里+被转义为空格
http://localhost:8983/solr/techproducts/select?q=\"CAS+latency\"
# 联合搜索，%2B是+的转义，%20是空格的转义，等价于+electronics +music
http://localhost:8983/solr/techproducts/select?q=%2Belectronics%20%2Bmusic
http://localhost:8983/solr/techproducts/select?q=%2Belectronics+-music

```

Exercise 2
```shell
# 创建一个新collection，使用_default配置
bin/solr create -c films -s 2 -rf 2

####################
# 
# 默认情况下：
#   1. _default生成的是“managed schema”，用户需要使用 Schema API 修改schema的规则。
#   2. _default默认是schemaless（这个配置定义在solrconfig.xml中）。
#   3. 通常情况下不应该在生产环境中使用schemaless特性，而应该手工定义schema.xml文件。
#   4. 
####################

# 创建text_general类型的字段，stored为true表示这个字段可以用来进行查询

curl -X POST -H 'Content-type:application/json' --data-binary '{"add-field": {"name":"name", "type":"text_general", "multiValued":false, "stored":true}}' http://172.24.26.93:8983/solr/films/schema

####################
#
# CopyField：solr 有一个字段复制机制，可以将多个不同类型字段集中到一个字段。
#   
#    典型应用场景：
#       用户创建博客索引时，需要查询title、content。 此时可以定义一个新字段，将title和content复制到这个新字段，索引的时候，直接从这个新字段查#    询，这样就达到目地了。
#
####################

# 定义一个copyField，将所有字段的值复制到_text_字段
curl -X POST -H 'Content-type:application/json' --data-binary '{"add-copy-field" : {"source":"*","dest":"_text_"}}' http://172.24.26.93:8983/solr/films/schema

# 使用Post命令进行不同格式的数据导入

bin/post -c films example/films/films.csv -params "f.genre.split=true&f.directed_by.split=true&f.genre.separator=|&f.directed_by.separator=|"
bin/post -c films example/films/films.xml
bin/post -c films example/films/films.json


####################
#   Faceting：
#        允许将搜索结果排列成子集，为每个子集提供计数。Solr提供以下类型的Faceting：
            Field Facets：对某个Field进行分类统计
            Numeric Range Facets：统计某个Field的Range，如价格区间
            Pivot Facets：组合分类，如统计属于A、同时又属于B的记录
####################

# 以下命令对genre_str进行分类统计，并返回统计结果
curl "http://localhost:8983/solr/films/select?q=*:*&rows=0&facet=true&facet.field=genre_str"
# 以下命令对genre_str进行分类统计，但是只返回统计计数大于200的结果
curl "http://localhost:8983/solr/films/select?=&q=*:*&facet.field=genre_str&facet.mincount=200&facet=on&rows=0"
# 统计发行年代，范围是[NOW-20YEAR,NOW],每一类的间隔是1年
curl 'http://localhost:8983/solr/films/select?q=*:*&rows=0&facet=true&facet.range=initial_release_date&facet.range.start=NOW-20YEAR&facet.range.end=NOW&facet.range.gap=%2B1YEAR'
# 统计类型和导演
curl "http://localhost:8983/solr/films/select?q=*:*&rows=0&facet=on&facet.pivot=genre_str,directed_by_str"
```
Exercise 3
```shell
# 删除数据
bin/post -c localDocs -d "<delete><id>SP2514N</id></delete>"
bin/post -c localDocs -d "<delete><query>*:*</query></delete>"

# 重启服务
./bin/solr start -c -p 8983 -s example/cloud/node1/solr
./bin/solr start -c -p 7574 -s example/cloud/node2/solr -z localhost:9983

```


# 配置文件

Standlone模式下比较关键的配置文件均在solr.home目录下（可以是/var/solr/data目录）：

```shell
.
├── films                     # Core 名称目录
│   ├── conf                  ##### SolrCloud模式时放到ZK的configs目录上
│   │   ├── managed-schema    # 该文件是Schema的配置文件
│   │   └── solrconfig.xml    # 这个配置文件指定了core的一些高级配置，如数据目录等等
│   ├── core.properties       # Core的关键信息配置文件
│   └── data
├── solr.xml                  # 单个Solr server信息的配置  ##### SolrCloud模式时放到ZK的configs目录上
└── zoo.cfg                 

```
配置文件详细信息参考：

[solr.xml](https://lucene.apache.org/solr/guide/7_0/solr-cores-and-solr-xml.html#solr-cores-and-solr-xml)
[core.properties](https://lucene.apache.org/solr/guide/7_0/defining-core-properties.html#defining-core-properties)
[solrconfig.xml](https://lucene.apache.org/solr/guide/7_0/configuring-solrconfig-xml.html#configuring-solrconfig-xml)
[managed-schema](https://lucene.apache.org/solr/guide/7_0/documents-fields-and-schema-design.html)


使用CDH部署时，采用的是SolrCloud + HDFS的部署方式，默认情况下使用/metadata/solr/server作为部署目录。

此时安装目录为/opt/cloudera/parcels/CDH/lib/solr！！

使用SolrCloud时，config目录下的配置会在创建Collection时被上传到地址为/solr/configs/{collection name}的Znode中。


## configsets

configsets 实际上是一组可以重用的配置文件模板，configsets中的内容实际上是/var/solr/data/xxx/conf里的内容。

默认情况下configsets在/opt/solr/server/solr/configsets目录中。CDH中这些配置文件似乎是在ZK上？？


## schema
Solr的schema是一个单独的xml文件(managed-schema )，它存储以下信息：

- core和collection的字段和字段类型信息。
- core和collection的查询、存储规则，以及copy fields、dynamic fields的生成规则。

# SolrCloud

## 关键概念

群集可以托管多个Solr Collection。可以将Collection划分为多个Shards，每个Shards包含Collection的一部分内容。Shards的数量对Collection来说是确定的，是一种抽象概念。多个Shard可以让搜索请求并行执行。哪个Shard包含集合中的哪些文档取决于该Collection指定的分片策略。

replica是和Shard相对的物理概念，每个shard至少包含一个replica，并且其中一个Replicas是leader（通过ZK的选举机制获得）。

## Replica

由于Replica中需要进行leader，通常情况下Solr支持以下几种Replica类型：

- NRT：NRT副本（NRT = NearRealTime）维护一个transaction日志并在本地将新文档写入其索引。NRT类型的Replica随时都有资格被选为Leader
- TLOG： 此类Replica也维护事务日志，但不会更改本地的本地索引文档（不会更改索引）。TLOG类型的Replica通过从领导者复制索引来实现Index更新。TLOG同样可以成为Leader，并且在成为Leader后，他的行为和NRT类型的副本一致。
- PULL：不维护事务日志，也不更改本地索引文档。无法成为leader，也无法参与Leader选举。

几种Replica配置方式：

- All NRT replicas：适合小型集群，或者更新频率不高的大型集群；
- All TLOG Replicas： 每个分片的副本数量很高时;
- TLOG replicas plus PULL replicas: 每个分片的副本数量很高时，增加查询请求的吞吐量；

## Document Routing

通过API创建Collection时，可以使用router.name来自动Document使用的路由策略。

- compositeId ：通过文档ID部分前缀的hash值，进行路由。通常可以使用"!"在文档ID中分割出这一部分前缀，在查询时可以使用_route_来指定需要查询的Shard地址。compositeId方案支持二级路由，如以下类型的ID："USA!IBM!12345"。同时还支持shard_key/num!document_id格式的路由。
- implicit：使用router.field指定的字段来标识所属的shard

## Shard Splitting

Shard支持Splitting命令，用户可以通过[SPLITSHARD command](https://lucene.apache.org/solr/guide/7_0/collections-api.html#splitshard)将Shard一分为二。

# Solr on HDFS

Solr支持将索引和事务日志文件写入到HDFS中。通过JVM参数、solr.in.sh、solrconfig.xml三种方式可以指定Solr存储位置。

## 参数配置

Standalone指定以下参数：

```shell
bin/solr start -Dsolr.directoryFactory=HdfsDirectoryFactory
     -Dsolr.lock.type=hdfs
     -Dsolr.data.dir=hdfs://host:port/path
     -Dsolr.updatelog=hdfs://host:port/path
```

SolrCloud 指定以下参数
```shell
bin/solr start -c -Dsolr.directoryFactory=HdfsDirectoryFactory
     -Dsolr.lock.type=hdfs
     -Dsolr.hdfs.home=hdfs://host:port/path
```

使用Apache Solr时对接HDFS需要指定的参数：
```shell
SOLR_OPTS="$SOLR_OPTS -Dsolr.directoryFactory=HdfsDirectoryFactory"
SOLR_OPTS="$SOLR_OPTS -Dsolr.lock.type=hdfs"
SOLR_OPTS="$SOLR_OPTS -Dsolr.hdfs.home=hdfs://nameservice1/test_solr"
SOLR_OPTS="$SOLR_OPTS -Dsolr.hdfs.confdir=/etc/hadoop/conf"
SOLR_OPTS="$SOLR_OPTS -Dsolr.hdfs.confdir=/etc/hadoop/conf"
SOLR_OPTS="$SOLR_OPTS -Dsolr.hdfs.security.kerberos.enabled=true"
SOLR_OPTS="$SOLR_OPTS -Dsolr.hdfs.security.kerberos.keytabfile=/opt/solr/solr.keytab"
SOLR_OPTS="$SOLR_OPTS -Dsolr.hdfs.security.kerberos.principal=solr/lqing93@IDATA.RUIJIE.COM"
```

## 缓存机制

HdfsDirectoryFactory 会申请对外内存用来缓存HDFS上的块信息。此缓存通常需要非常大，可能需要提高运行Solr的特定JVM的堆外内存限制。

``` shell
# 以下参数限制JVM的堆外内存大小
-XX:MaxDirectMemorySize=20g
```

# 使用SolrCtl命令

solrctl命令是cloudera包装过的solr命令行工具，使用方式和原生solr不同。官方使用说明可以参考[链接](https://hadoop.rcc.uchicago.edu:7183/static/help/topics/search_solrctl_ref.html)。

```shell
# 在/var/lib/solr/demo目录创建配置文件模板
solrctl instancedir --generate /var/lib/solr/demo  -schemaless
# 上传本地目录中的配置文件到ZK，此时生成了一个新的configset
solrctl instancedir --create demo /var/lib/solr/demo
# 创建collection，并且使用同名configset
solrctl collection --create demo -s 2 -a -c demo -r 2 -m 2
# 编辑/var/lib/solr/demo目录下的文件后更新到Zookeeper
solrctl instancedir --update demo /var/lib/solr/demo
# reload collection应用更新后的配置
solrctl collection --reload demo
```

配置kerberos时参考以下命令
```shell
# step1
solrctl instancedir --generate /var/lib/solr/film  -schemaless
# step2
kinit -k solr
solrctl --jaas  /etc/zookeeper/conf/jaas.conf --debug --zk bdnode1:2181,bdnode2:2181,bdnode3:2181/solr  instancedir --create film /var/lib/solr/film
# step3
solrctl collection --create film -s 2 -a -c film -r 2 -m 4
```

# 参考文档

[W3Cschool:Apache Solr参考指南](https://www.w3cschool.cn/solr_doc/solr_doc-t3642fkr.html)

[官网 QuickStart](http://lucene.apache.org/solr/guide/7_7/solr-tutorial.html)

[solr命令的使用](https://lucene.apache.org/solr/guide/7_0/solr-control-script-reference.html#solr-control-script-reference)

[SolrCloud](https://lucene.apache.org/solr/guide/7_0/solrcloud.html#solrcloud)

[SolrCloud Configuration and Parameters](https://lucene.apache.org/solr/guide/7_0/solrcloud-configuration-and-parameters.html#solrcloud-configuration-and-parameters) 

[HdfsDirectoryFactory 的配置参数](http://lucene.apache.org/solr/guide/7_7/running-solr-on-hdfs.html#hdfsdirectoryfactory-parameters)

[如何在查询时指定分片](https://lucene.apache.org/solr/guide/7_0/distributed-requests.html#limiting-which-shards-are-queried)

[solrcloud-configuration-and-parameters](https://lucene.apache.org/solr/guide/7_0/solrcloud-configuration-and-parameters.html#solrcloud-configuration-and-parameters)