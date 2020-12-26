# 1. 介绍

为了解决有状态容器的问题，CoreOs公司提出了Operator Framework。核心思想是：“**进行Kubernetes二次开发，为每个有状态组件打造专门的Control，将运维知识固化到软件！**”。

Operator Framework包括以下三个部分：

- [SDK](https://github.com/operator-framework/operator-sdk/blob/master/doc/user-guide.md)：基于SDK能够在不了解Kubernetes API知识的情况下，快速开发Operators。

- [Lifecycle Manager](https://github.com/operator-framework/operator-lifecycle-manager/blob/master/Documentation/install/install.md)：对Operators的生命周期进行管理，可以认为OLM是所有Operator的管理组件。

- Metering：提供Operator资源使用报告。

## 1.1 Getting Started

CoreOS在官方GitHub上的提供了一个[QuickStart教程](https://github.com/operator-framework/getting-started)，帮助用户快速搭建[Memcached](https://memcached.org/)组件（分布式内存对象缓存）的Operator（[参考](https://github.com/LinQing2017/notes/blob/master/Kubernetes/OpSDK安装.md)）。

SDK工具安装完成后，使用operator-sdk命令可以生成Operator项目的工程模板。模板架构如下，参考[官方架构说明](https://github.com/operator-framework/operator-sdk/blob/master/doc/project_layout.md)可以知道各个目录的具体作用。

​                                

# 2. 开源Operators

自2016年CoreOS公司提出Operator框架后，相当多的[数据服务公司](https://commons.openshift.org/sig/operators.html)基于这个框架（或者这个理念）实现对应组件在Kubernetes层面的自动化管理！

当前以下组件能够找到的Operator（不完全统计）：

| 组件名称                                                     | Operator提供者      | 备注                                                         |
| ------------------------------------------------------------ | ------------------- | ------------------------------------------------------------ |
| [MongoDB](https://docs.opsmanager.mongodb.com/current/tutorial/install-k8s-operator/) | Mongo官方           | 集成在企业版中                                               |
| [Kafka](https://www.confluent.io/confluent-operator/)        | Confluent公司       | [Confluent](https://www.confluent.io/confluent-operator/)公司是一家数据平台公司，该公司以Kafka为核心组件打造数据平台。目前Kafka  Operator是开源的 |
| [Redis](https://redislabs.com/blog/redis-enterprise-operator-kubernetes/) | RedisLab            | Redis企业版的功能，但是也找到了[开源方案](https://github.com/spotahome/redis-operator)。 |
| [Mysql](https://github.com/oracle/mysql-operator)            | Oracle官方          | 开源，似乎是个实验版本                                       |
| [PostgreSQL](https://github.com/CrunchyData/postgres-operator) | CrunchyData公司     | CrunchyData专门提供PostgreSQL的商业解决方案                  |
| [Cassandra](https://github.com/instaclustr/cassandra-operator) | Instaclustr         | Instaclustr也是一家大数据公司el                              |
| [Spark](https://github.com/GoogleCloudPlatform/spark-on-k8s-operator) | GoogleCloudPlatform |                                                              |

 