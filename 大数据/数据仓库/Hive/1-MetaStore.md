# MetaStore的安装配置

严格上，MetaStore的安装方式，仅仅包括以下两种：

## Local/Embedded Metastore Database (Derby)

默认情况下，HIVE通过嵌入式的方式启动metastore。该方式中，用户的HIVE命令会启动一个MetaStore线程（DerbyDB）。该模式下只有一个客户端能够访问metastore服务，无法用户生产模式。

Local模式中元数据保存在外部数据库中（比如，Mysql等）。但是，这个模式下无需额外启动MetaStore进程，MetaStore服务嵌入在Hive的客户端进程中。

## Remote Metastore Database

- 远程存储需要单独起metastore服务，远程元存储的metastore服务和hive运行在不同的进程里；
  
- 每个客户端都在配置文件里配置连接到metastore的服务；
  
### 服务端MetaStore关键配置

|Config|Config Value|Comment|
|----|----|----|
|javax.jdo.option.ConnectionDriverName|com.mysql.cj.jdbc.Driver|需要用户自己下载驱动文件，并且保存到$HIVE_HOME/lib目录|
|javax.jdo.option.ConnectionURL|jdbc:mysql://\<host name\>/\<database name\>?createDatabaseIfNotExist=true|metadata数据库地址，<br> 如果使用本地模式，那么需要指定databaseName=/home/my/hive/metastore_db|
|javax.jdo.option.ConnectionUserName|\<user name \>||
|javax.jdo.option.ConnectionPassword|\<password \>||
|hive.metastore.warehouse.dir|\<base hdfs path\>||
|hive.metastore.thrift.bind.host|\<host_name \>|默认配置是localhost,需要确定这个配置需不需要配！！|

### 客户端配置

|Config|Config Value|Comment|
|----|----|----|
|hive.metastore.uris|thrift://\< host_name \>:\< port \>| 配置metastore的元数据地址，配置多个metastore时，可以用逗号隔开thrift://lqnode1:9083,thrift://lqnode2:9083,thrift://lqnode3:9083 |
|hive.metastore.local|false||
|hive.metastore.warehouse.dir|\<base hdfs path\>| 如果使用本地模式，那么该目录就是保存元数据的目录（metastore_db，默认是运行HCLi的当前目录）|

### 动态MetaStore发现

4.0.0以上版本，HiveClient能够基于Zookeeper动态发现MetaStore，参考[HIVE-20794](https://issues.apache.org/jira/browse/HIVE-20794)、[AdminManual Metastore Administration](https://cwiki.apache.org/confluence/display/Hive/AdminManual+Metastore+Administration#AdminManualMetastoreAdministration-RemoteMetastoreDatabase)

### 独立启动MetaStore元数据服务

```shell

schematool -initSchema -dbType mysql # 初始化化db

hive --service metastore -p <port_num> # 启动metaStore服务

```

# 参考

[E/R 模型](https://issues.apache.org/jira/secure/attachment/12471108/HiveMetaStore.pdf)