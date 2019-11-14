# [官方方案](https://hbase.apache.org/book.html#spark)

HBase 的官方文档提供了数个不同的Spark读写HBase的方法。

## Basic Spark

## Spark Streaming

## Spark Bulk Load

## SparkSQL/DataFrames

# Spark on HBase

Github上可以找几个Spark on HBase 的API算法库。

|Hub|说明|备注|
|----|----|----|
|[cloudera-labs/SparkOnHBase](https://github.com/cloudera-labs/SparkOnHBase)|该模块整合到CDH的发行版中，包含大量bulk相关API||
|[Huawei-Spark/Spark-SQL-on-HBase](https://github.com/Huawei-Spark/Spark-SQL-on-HBase)|通过HBase的Map/Reduce接口实现基于SQL的实时查询||
|[Apache HBase Connector](https://github.com/hortonworks-spark/shc)|hortonworks提供spark on API|
|[nerdammer/spark-hbase-connector](https://github.com/nerdammer/spark-hbase-connector)||

# 其他HBase引擎

|项目名称|有应用场景|备注|
|----|----|----|
|HBase Ganos|基于HBase的时空序列引擎|[locationtech/geomesa](https://github.com/locationtech/geomesa/tree/master/geomesa-hbase) <br /> [阿里的云服务](https://help.aliyun.com/document_detail/87287.html)|
|OpenTSDB|基于HBase的时序数据库||
