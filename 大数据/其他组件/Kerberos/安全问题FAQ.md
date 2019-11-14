# Hue

## 1. 访问hive

**问题**：**当前通过Hue的hive插件无法正常访问Hive**。

**解决**：没有解决，请使用命令行访问hive

**原因**：Hue通过HiveServer2的hive.server2.enable.doAs功能获取Hive中的数据，当前平台没有配置HS2的doAs配置。

## 2. 访问HBase

**问题**：**当前通过Hue的HBase插件无法正常访问HBase**。

**解决**：没有解决，请使用命令行访问HBase

**原因**：Hue通过HBase Thrift 服务访问 HBase，此时需要同时配置HBase Thrift的doAs和SSL功能，当前平台没有开启这两个配置。

## 3. 访问Impala

**问题**：**当前通过Hue的Impala插件无法正常访问Impala**。

**解决**：已经解决，但是没有和入安装包，请安装完手动配置。

**原因**：Hue 通过21050端口访问Impalad，配置Kerberos后需要修改下面的配置项才能正常使用Impala的相关功能。在“hue_safety_valve_server.ini 的 Hue Server 高级配置代码段（安全阀）”中添加：

```ini
[impala]
server_port=31050
```

Hue在连接impalad时是通过hue账户连接的，如果需要打开代理功能，那么需要配置 impersonation_enabled=True ！但是，[impala官方文档](https://impala.apache.org/docs/build/html/topics/impala_delegation.html)中提到使用HUE的代理功能时，需要环境部署Sentry。当前，Impala没有开启权限管理，因此hue可以访问任何表。

## 4. 访问HDFS文件

Hue访问大部分Hadoop服务时，是通过代理的方式进行的。Hue进行认证的身份永远是hue/_HOST@EXAMPLE.COM。在HDFS中hadoop.proxyuser.hue.hosts和hadoop.proxyuser.hue.groups都配置成了*，这样hue可以模拟任意用户。其他服务，如Oozie、Flume、Hive使用的也是这个原理！！

因此，如果需要在Hue上访问、修改hdfs、idata等用户的文件，那么请在Hue管理员界面中创建同名账号，然后使用该账号登录即可。

# Hive

## 1. 使用Beeline命令行

使用beeline连接Hive时需要注意以下几点：

- hive.users.in.admin.role需要在HS2和metastore中同时配置，否者不生效。
- 通过beeline登录后，需要使用set role admin命令获取管理员权限
- 使用idata登录后，beeline上的任务以hive用户身份提交到root.users.idata队列~~~！ 这种情况发发现/data/dataXXX/yarn/nm/usercache/hive的权限可能会不对，但是好像不是必现情况。
- 初始化安装完需要将HDFS上/user/hive的owner改成hive，不然beeline没法提交Yarn任务。（必现，已经和入安装包）

## 2. MetaStore 权限管理带来的性能问题

联调过程中发现MetaStore服务引入基于文件系统的权限问题时带来性能问题，导致流处理卡住。当前已经关闭这个功能。


# Zookeeper

## 1. 临时关闭认证

通过-Dzookeeper.skipACL=yes配置在ZK的JVM启动参数中，能够在配置不动的情况下跳过Keberos认证。

## 2. 如何修改Znode节点权限

Zookeeper添加Znode权限时是覆盖操作，在/solr上添加idata的权限应该像这样，

```shell
setAcl /solr sasl:idata:cdrwa,sasl:solr:cdrwa,world:anyone:r #表示为多个用户添加znode节点的权限
```

# Flume

##1. 指定hdfs插件的Kerberos配置

hdfs.kerberosPrincipal 配置需要指定全名，使用短名称不行。

## 2. Flume连接kafka

参考agent中kafka插件需要添加下面三个配置，

```
tier1.sources.r3.kafka.consumer.security.protocol = SASL_PLAINTEXT
tier1.sources.r3.kafka.consumer.sasl.mechanism = GSSAPI
tier1.sources.r3.kafka.consumer.sasl.kerberos.service.name = kafka
```

Flume Agent 的 Java 配置选项，添加下面两个配置：

```
 -Djava.security.krb5.conf=/etc/krb5.conf
 -Djava.security.auth.login.config=/opt/idata_security/etc/jaas.conf
```

# Kafka

## 1. 创建Kafka Topic

进行创建/修改Topic时，需要刷环境变量 export KAFKA_OPTS="${KAFKA_OPTS} -Djava.security.auth.login.config=/opt/idata_security/kafka/jaas.conf" 

这个改环境变量已经和入到平台安装包的/etc/profile.d/idata_security.sh文件中。

## 2. Spark消费Kafka的topic

使用DataFram直接读写Kafka时，需要在 sasl.mechanism 、sasl.kerberos.service.name、security.protocol、以及sasl.jaas.config这四个配置上加上"kafka."的前缀，否者Spark的DF初始化时会使用通常方式去连接Broker，导致认证失败。（参考：[Spark官方文档](https://spark.apache.org/docs/2.2.0/structured-streaming-kafka-integration.html#kafka-specific-configurations)）

## 3. 使用kafka命令行连接Topic

使用kafka-console-consumer、kafka-console-producer连接topic时请分别加上下面两个配置：

```
kinit -k idata
kafka-console-producer --producer.config  /opt/idata_security/kafka/client.properties ...
kafka-console-consumer --consumer.config  /opt/idata_security/kafka/client.properties ...
```



# HBase

## 1. HBase的Client配置

使用 hbase.client.kerberos.principal、hbase.client.keytab.file 可以在 hbase-site.xml 中指定 Client 的Kerberos参数（已经合入到平台安装包）。

## 2. Spark任务连接HBase

Oozie启动Spark任务时不会主动加载hbase-site.xml文件，导致Spark没有使用Kerberos认证的方式连接hbase。

可以添加以下配置解决：spark-conf/spark-defaults.conf 的 Spark 客户端高级配置代码段（安全阀）

```
   spark.hadoop.hbase.security.authentication=kerberos
   spark.hadoop.hbase.master.kerberos.principal=hbase/_HOST@IDATA.RUIJIE.COM
   spark.hadoop.hbase.regionserver.kerberos.principal=hbase/_HOST@IDATA.RUIJIE.COM
   spark.hadoop.hbase.zookeeper.quorum=bdnode3,bdnode4,bdnode5
```

idata应用在spark任务中没有通过hbase-site.xml来加载hbase配置的，可以在application.conf中添加下面的配置。！

```
hbase_security_authentication = kerberos
hbase_master_kerberos_principal = "hbase/_HOST@IDATA.RUIJIE.COM"
hbase_regionserver_kerberos_principal = "hbase/_HOST@IDATA.RUIJIE.COM"
hbase_client_kerberos_principal = "idata@IDATA.RUIJIE.COM"
hbase_client_keytab_file = "/etc/krb5.keytab"
```

# Hadoop

## 1. Yarn无法终止Spark任务

安全模式时通过Cloudera、Hue无法杀死Yarn上的任务，需要通过yarn  application -kill {app_id}在后台杀死Spark任务。

## 2. 在代码中切换认证HDFS的Kerberos用户

配置了代理配置后，可以使用doAs方式进行用户切换，如从idata临时切换到hdfs进行一些特权操作。

只使用proxy前，需求配置以下参数：

```
hadoop.proxyuser.idata.hosts： *
hadoop.proxyuser.idata.groups：*
```

示例代码

```java
package com.ruijie.idata.demo.security;
import org.apache.hadoop.conf.Configuration;
import org.apache.hadoop.fs.FileSystem;
import org.apache.hadoop.fs.Path;
import org.apache.hadoop.hdfs.DistributedFileSystem;
import org.apache.hadoop.hdfs.protocol.DatanodeInfo;
import org.apache.hadoop.security.UserGroupInformation;
import java.io.IOException;
import java.security.PrivilegedExceptionAction;

public class HDFS_SUPER {
​    public static void main(String[] args) throws InterruptedException {
​        final Configuration conf = new Configuration();
​        conf.addResource(new Path("/etc/hadoop/conf/hdfs-site.xml"));
​        conf.addResource(new Path("/etc/hadoop/conf/core-site.xml"));

​        // 使用Idata用户登入

​        KrbLoginDemo.krbLogin4Hadoop(conf, "idata", KrbLoginDemo.default_keytab);
​        try {
​            // 以idata用户的身份代理，hdfs进行操作 -- 成功

​            UserGroupInformation ugi_proxy_hdfs = UserGroupInformation.createProxyUser("hdfs", UserGroupInformation.getCurrentUser());
​            ugi_proxy_hdfs.doAs(new PrivilegedExceptionAction<Void>() {
​                public Void run() throws Exception {
​                    DistributedFileSystem fs_hdfs = (DistributedFileSystem) FileSystem.get(conf);
​                    datanode_usage(fs_hdfs);
​                    return null;
​                }
​            });

​            // idata用户的身份来操作 -- 失败
​            System.out.println(UserGroupInformation.getCurrentUser());
​            DistributedFileSystem fs_idata = (DistributedFileSystem) FileSystem.get(conf);
​            datanode_usage(fs_idata);
​        } catch (IOException e) {
​            e.printStackTrace();
​        }
​    }

​    public static void datanode_usage(DistributedFileSystem fs) {
​        try {
​            System.out.println(UserGroupInformation.getCurrentUser());
​            for (DatanodeInfo datanodeInfo : fs.getDataNodeStats()) {
​                System.out.println(datanodeInfo.getDfsUsedPercent());
​            }
​        } catch (IOException e) {
​            e.printStackTrace();
​        }
​    }
}
```

# Oozie

## 1. 提交Oozie任务用户不争取

默认情况下，Oozie优先使用 ${home}/.oozie-auth-token 缓存中票据。如果用户先使用HDFS等blacklist中的用户生成了该文件，会导致后续提交的任务的用户缓存文件中的用户，进而导致提交任务失败。

# Kerberos

## 1. 主机名对Kerberos的影响

当在/etc/hosts中为一个ip指定多个域名时，**域名的顺序**会影响ZK、Impala等服务的认证。如果别名先定义会导致认证失败。

以下方式配置/etc/hosts时会导致Kerberos认证失败

```
## Case 1
172.29.32.61   localhost localhost.localdomain localhost4 localhost4.localdomain4
::1         localhost localhost.localdomain localhost6 localhost6.localdomain6
172.29.32.61 bdnode1

## Case 2
172.29.32.64         mysqlVirtualHost
172.29.32.61 	       realhost
172.29.32.64         nginxVirtualHost
172.29.32.61         nginxRealHost1
172.29.32.62         nginxRealHost2

172.29.32.61 bdnode1
172.29.32.62 bdnode2
172.29.32.63 bdnode3
172.29.32.64 cdh.vip
```

以下方式配置/etc/hosts时不影响Kerberos

```
172.29.32.61 bdnode1
172.29.32.62 bdnode2
172.29.32.63 bdnode3
172.29.32.64 cdh.vip

172.29.32.64         mysqlVirtualHost
172.29.32.61 	       realhost
172.29.32.64         nginxVirtualHost
172.29.32.61         nginxRealHost1
172.29.32.62         nginxRealHost2
```

## 2. Kerberos的票据有效期

Kerberos ticket具有lifetime，超过此时间则ticket就会过期，需要重新申请或renew。

ticket lifetime取决于以下5项设置中的最小值：

- Kerberos server上/var/kerberos/krb5kdc/kdc.conf中max_life
- 内置principal krbtgt的maximum ticket life，可在kadmin命令行下用getprinc命令查看
- principal的maximum ticket life，可在kadmin命令行下用getprinc命令查看
- Kerberos client上/etc/krb5.conf的ticket_lifetime
- kinit -l 参数后面指定的时间

ticket过期后，如果想延长，一种方法是重新申请（需要输入密码），另一种是renew（不需要输入密码），每renew一次，就延长一个lifetime。不过renew操作本身也有lifetime，即在ticket renew lifetime，在此lifetime之内，才能进行renew操作。与上面的很相似，ticket renew lifetime取决于以下5项设置中的最小值：

- Kerberos server上/var/kerberos/krb5kdc/kdc.conf中max_renewable_life
- 内置principal krbtgt的maximum renewable life，可在kadmin命令行下用getprinc命令查看
- principal的maximum renewable life，可在kadmin命令行下用getprinc命令查看
- Kerberos client上/etc/krb5.conf的renew_lifetime
- kinit -r 参数后面指定的时间

**解決TGT超期的注意事項**：

- 不使用cache，配置删除ticketCache配置，useTicketCache=false，renewTGT=fasle
- Zookeeper的client不会读取ticketCache指定的cache地址，需要直接使用KRB5CCNAME的地址。
- 可以用 kinit -k idata -c /tmp/idata_krbcc 生成缓存文件。默认情况下，kinit如果不带-c，且没有指定KRB5CCNAME，默认存储读取的Cache为/tmp/krb5cc_{uid}，否则为环境变量指定的值。

## 3. 卸载过程中多人操作导致的一些问题

由于同一个节点中多个用户反复执行kinit会导致bash环境变量中的Kerberos用户切换，影响一些正在运行的脚本。因此请卸载、安装服务时，最好避免多个人同时操作节点1.


# 其他問題

## 1. 重新生成服务票据

- 重新生成服务票据如果失败，请检查节点1的kadmin服务是不是挂了
- 重新生成服务票据后，LDAP服务有可能需要时间同步，导致部分服务启动时认证失败，等一会就好了