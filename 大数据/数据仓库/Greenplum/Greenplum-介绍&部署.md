---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-08
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 1. 概述

GreenPlum中文社区的[介绍](https://greenplum.cn/intro/)中，将Greenplum定位成**开源大数据平台**，而不仅仅是一个MPP数据查询引擎。

![](https://gp.stage.vonbros.com/wp-content/themes/twentyseventeen/assets/images/architecture.png)

Greenplum的优势：

- 处理和分析各种数据源的数据的平台：包括hadoop、Hive、HBase、S3等等，支持结构化、半结构化、非结构化数据
- 数据水平分布、并行查询执行、专业优化器、线性扩展能力、多态存储、资源管理、高可用、高速数据加载
- 接口可扩展，支持SQL、JDBC和ODBC等行业标准
- 集成数据分析平台：MADlib (Github 245个Star，半死不活)
- 在金融、保险、证券等领域有众多应用案例，具备较为完善的生态
- 采用 ** Apache 2 协议 **


## 架构

![](https://greenplum.cn/gp6/graphics/highlevel_arch.jpg)

Greenplum基本架构包括：Master、SegmentHost。

- Master：Greenplum数据系统的入口，Client连接Master提供SQL。Master管理了全局系统目录，包含了有关Greenplum数据库本身的元数据（系统表）。Master的主备基于WAL预写式日志来实现主/备镜像。
- SegmentHosts：基于Postgresql 8.3的定制数据库，负责存储和处理用户数据。 一台Segment主机通常运行2至8个Greenplum的Segment。
- Interconect：Interconect是Greenplum数据库架构中的网络层，默认协议UDPIFC，如果使用TCP协议，那么Greenplum限制1000个Segment。


## 事务控制

Greenplum支持事务控制，当并发更新时Greenplum通过MVCC模型来保证事务数据一致性。

**MVCC模型**基于**快照**机制是Postgresql中的一个特性，能够管理数据行的多个版本。

### MVCC

数据库ACID特性的描述：

- Atomicity：事务的操作结果要么全部执行要么全部不执行
- Consistency：总是从一个一致的状态转换到另一个一致的状态
- Durability：事务一旦被提交，它对数据库中数据的改变就是永久性的，接下来即使数据库发生故障也不应该对其有任何影响。
- Isolation：事务的修改结果在什么时间能够被其他事务看到（SQL1992规范），隔离级别包括以下四个：

    - 未提交读：事务能够看到其他事务没有提交的修改，当另一个事务又回滚了修改后导致读取到脏数据，这种情况又称为 **脏读**
    - 已提交读：事务能够看到其他事务提交后的修改，这时会出现**一个事务内两次读取数据**不一致，这种模式下事务是**不可重复读**的。
    - 可重复读:在两次读取时读取到的同一行数据是一致的，但是两次查询可能查到行数不一致（其他事务出现新的插入），这种情况称为**幻读**。
    - 序列化级别：不允许出现幻读、脏读、不可重复读。

上述特性中，Isolation是关键，不同数据库实现了不同级别的隔离性，并且通常情况下使用**锁**和**MVCC**来解决这些问题。但是，上述四种隔离级别主要是基于锁机制进行论述的，MVCC机制实现的隔离级别只有**SI（快照隔离）**和**SSI（可序列化快照隔离）**，其中：

- **SI（快照隔离）**：对应读已提交和可重复读，并且该隔离级别中也不会出现幻读（如Greenplum）。

- **SSI（可序列化快照隔离）**：由于快照并发控制并不能真正意义上保证事务是“可串行化”的，所以事务间的并发操作依旧有可能引发数据异常现象。但这里的异常不同于之前提到的脏读、丢失更新的异常，而是一种业务数据间逻辑语义层面的异常，也可以说是由于未能满足数据间的语义约束而产生的异常，即**写偏序（Write skew）**。为了避免写偏序，可以使用SSI隔离级别实现，MYSQL、PG等数据库的MVCC引擎并没有实现SSI隔离级别

Greenplum中未提交读、已提交读隔离模式的效果和标准SQL一致；可重复读模式避免了不可重复读和幻读；Greenplum数据库并不完全支持可串行化模式（该模式时自动退化到可重复读模式），并且数据操作并非真正串行化的，**即Greenplum基于MVCC实现隔离，且实现了SI级别的隔离**

传统方案采用**读写锁**（读锁和读锁之间不互斥，写锁互斥其他所有锁），MVCC是一种完全使读写操作并发的方案（完全抛弃锁）。

用户执行事务时可以在SQL中指定，事务隔离级别：

```SQL
BEGIN;
SET TRANSACTION ISOLATION LEVEL REPEATABLE READ;
...
COMMIT;
```

MVCC的实现：
- 在PostgreSQL中，每一个事务(包括单条SQL)都会得到一个被称作为 XID 的事务ID。Session请求事务操作时，PostgreSQL递增XID并赋给这个事务。
- 每一行记录都存储了事务相关信息，这些信息用于判断当前事务是否可见。

    - xmin：在创建记录时，记录此时的事务id，后面每次update也会更新。
    - xmax：在更新或删除或lock时，记录此时的事务id；如果记录没有被删除，那么此时为0。
    - cmin：多语句事务存储创建这个元组的Command ID
    - cmax：多语句事务删除这个元组的Command ID

- 一个事务会看到 xid < xmin 的行（这些行已经commit），并且这些行 xid > xmax （这些行已经被删除）。

- cmin/cmax：用于多语句事务中，只在事务期间有意义，事务开始时该序列被重置为0。

- 每一个Segment数据库都有其自己的XID序列，Master会使用一个分布式事务ID，称为gp_session_id，Segment会会维护一个分布式事务ID到其本地XID的映射。

- 当一个Segment上的事务失败，会回滚所有Segment的修改。

- 一行支持二十亿个事务，这之后这一行将成为一个新行，通过一次VACUUM操作可以避免这样的情况。可以配置xid_warn_limit和 xid_stop_limit控制事务上限告警。

MVCC的实现存储了多个数据版本，非常容易造成**表膨胀**。VACUUM命令会标记过期行所使用的空间可以被重用。通常可以使用以下策略运行VACUUM命令：

- 重度更新的表，可能每天需要运行几次VACUUM。
- 运行了一个更新或者删除大量行的事务之后运行VACUUM。
- VACUUM FULL命令会把表重写为没有过期行，并且将表减小到其最小尺寸，同时该操作会锁表。
- 运行**VACUUM VERBOSE tablename**来得到一份Segment上已移除的死亡行数量、受影响页面数以及有可用空闲空间页面数的报告。

用户可以使用**LOCK LOCK**命令显示加锁（[参考]（https://gp-docs-cn.github.io/docs/ref_guide/sql_commands/LOCK.html））。

## 数据冗余和故障切换

部署Greenplum数据库系统时，Segment可以配置Mirror实例，当Primary节点宕机时，Mirror节点提供服务，如果系统中存在Segment没有配置Mirror，那么Segment会成为整个系统的单点故障。

Greenplum数据库中Segment镜像拓扑：

![](https://greenplum.cn/gp6/graphics/spread-mirroring.png)

用户可以选择一台不同于Master节点的主机上部署一个Master实例的备份或者镜像。 

后备Master利用事务日志复制进程保持与主Master同步，复制进程运行在后备Master上并且负责在主备Master主机之间同步数据。如果主Master失效，日志复制进程会停止，并且后备Master会被激活以替代它的位置。**Master失效时，主备切换不会自动发生，需要外部激励触发。**


# 2. 安装部署

官方Github上提供了从源码编译gpdb和gporca的完整步骤，同时也提供了预编译好的RPM和DEB包，以下安装不步骤参考网络上的一些文档，并非官方推荐的安装步骤（我没找到！！）。

## 准备工作

- 所有节点配置NTP服务
- 更新以下系统配置
```
sudo bash -c 'cat >> /etc/sysctl.conf <<-EOF
kernel.shmmax = 500000000
kernel.shmmni = 4096
kernel.shmall = 4000000000
kernel.sem = 500 1024000 200 4096
kernel.sysrq = 1
kernel.core_uses_pid = 1
kernel.msgmnb = 65536
kernel.msgmax = 65536
kernel.msgmni = 2048
net.ipv4.tcp_syncookies = 1
net.ipv4.ip_forward = 0
net.ipv4.conf.default.accept_source_route = 0
net.ipv4.tcp_tw_recycle = 1
net.ipv4.tcp_max_syn_backlog = 4096
net.ipv4.conf.all.arp_filter = 1
net.ipv4.ip_local_port_range = 1025 65535
net.core.netdev_max_backlog = 10000
net.core.rmem_max = 2097152
net.core.wmem_max = 2097152
vm.overcommit_memory = 2

EOF'

sudo bash -c 'cat >> /etc/security/limits.conf <<-EOF
* soft nofile 65536
* hard nofile 65536
* soft nproc 131072
* hard nproc 131072

EOF'

sysctl -p

```

- 在每个节点中创建gpadmin用户用于管理Greenplum，并且打通该节点的集群免密
```shell
groupadd -g 530 gpadmin
useradd -g 530 -u 530 -m -d /home/gpadmin -s /bin/bash gpadmin
echo "ruijie" | passwd --stdin gpadmin

su - gpadmin
# 只需要打通第一个节点到其他节点的ssh，之后执行 gpssh-exkeys -f hostlists 打通所有节点之间的互信
ssh-copy-id gpadmin@node11 && ssh-copy-id gpadmin@node12 && ssh-copy-id gpadmin@node13
```

## 安装Greenplum DB

下载RPM包，执行以下命令：

```shell
yum localinstall -y greenplum-db-6.0.0-rhel7-x86_64.rpm
```
安装完成后，安装目录为/usr/local/greenplum-db，同时还要将greenplum-db中的lib添加到ld.so.conf中：
```shell
sudo bash -c 'cat >> /etc/ld.so.conf.d/greenplum.conf <<-EOF
/usr/local/greenplum-db/lib
EOF'

ldconfig
```
在/home/gpadmin目录下配置环境变量、修改目录权限、创建配置目录

```shell
## /usr/local/greenplum-db/greenplum_path.sh 添加以下内容

source /usr/local/greenplum-db/greenplum_path.sh
export MASTER_DATA_DIRECTORY=/data/data_ssd/greenplum/data/master/gpseg-1
export PGPORT=5432
export PGDATABASE=gp_sydb

# 修改bin文件和数据目录权限：
chown -R  gpadmin:gpadmin  /usr/local/greenplum-db/
mkdir -p /data/data_ssd/greenplum # 数据目录
chown -R gpadmin:gpadmin /data/data_ssd/greenplum

# 创建配置目录
su - gpadmin
mkdir -p /home/gpadmin/conf
touch /home/gpadmin/conf/all_hosts #集群all_hosts文件,包含所有节点
touch /home/gpadmin/conf/seg_hosts #集群seg_hosts文件,包含所有segment节点
```

## 初始化数据库
登录master的gpadmin用户，验证免密是否成功：

```shell
# 验证免密是否可用
gpssh-exkeys -f /home/gpadmin/conf/all_hosts 

# gpssh命令提供类似ansible的公众
gpssh -f /home/gpadmin/conf/all_hosts

# 所有节点创建数据目录
mkdir -p /data/data_ssd/greenplum/data/master 
mkdir -p /data/data_ssd/greenplum/data/primary
mkdir -p /data/data_ssd/greenplum/data/mirror
```
配置文件模板位于/usr/local/greenplum-db/docs/cli_help/gpconfigs目录中，参考gpinitsystem_config创建配置文件/home/gpadmin/conf/gpinitsystem_config

```
RRAY_NAME="Greenplum Data Platform"
SEG_PREFIX=gpseg
PORT_BASE=6000
declare -a DATA_DIRECTORY=(/data/data_ssd/greenplum/data/primary /data/data_ssd/greenplum/data/primary /data/data_ssd/greenplum/data/primary)
MASTER_HOSTNAME=node11
MASTER_DIRECTORY=/data/data_ssd/greenplum/data/master 
MASTER_PORT=5432
TRUSTED_SHELL=ssh
CHECK_POINT_SEGMENTS=8
ENCODING=UNICODE
DATABASE_NAME=gp_sydb
MACHINE_LIST_FILE=/home/gpadmin/conf/seg_hosts

# 需要配置冗余时
# MIRROR_PORT_BASE=7000
# declare -a MIRROR_DATA_DIRECTORY=(/data/data_ssd/greenplum/data/mirror /data/data_ssd/greenplum/data/mirror /data/data_ssd/greenplum/data/mirror)
```

执行gpinitsystem -c /home/gpadmin/conf/gpinitsystem_config 初始化数据库。

如果需要使用冗余配置，则执行
```
gpinitsystem -c gpinitsystem_config  -h hostfile_exkeys -s {master备份节点} -S {master备份目录}
```
## 配置文件

初始化完成后，master节点的 **MASTER_DIRECTORY** 目录下会自动生成**gpseg-1**目录，该目录中的文件类似pg的配置文件，包含：postgresql.conf、pg_hba.conf等内容。

初始化成功后，Greenplum会自动创建管理员用户（默认情况下为执行初始化化程序的用户）。

初次启动时，用户需要使用管理员用户登录，并创建Client使用的账号以及修改账号登录方式（pg_hba.conf）。

## 基本操作

```shell
# 下面的所有操作在Master节点上运行，且使用gpadmin用户
# 启动
gpstart -a
# 关闭，-M fast表示关闭所有事务，并且回滚
gpstop -M fast
# 重启服务
gpstop -r
# 重载 pg_hba.conf 和 postgresql.conf，部分参数需要通过完全重启才能生效
gpstop -u

# 部分情况下，客户端进程会出现卡死，gp集群无法关闭，此时需要具有SUPERUSER权限的Greenplum用户登录postgres，杀死客户端进程。操作过程，参考：https://greenplum.cn/gp6/managing/startstop.html

# SELECT usename, pid, waiting, query, datname FROM pg_stat_activity;
# 上面的sql可以查出当前GP的活跃client，使用pg_cancel_backend(pid)、pg_terminate_backend(pid)可以强制退出这些线程。

# 查看服务的状态
gpstate -s

```

## 配置文件

Greenplum集群中包括：master参数和本地参数，这些参数存储每个实例的postgresql.conf、pg_hba.conf文件中。


- master参数：
  
    - 系统范围参数：编辑$MASTER_DATA_DIRECTORY/postgresql.conf文件
    - 数据库级别参数：使用**ALTER DATABASE xxx SET xxx TO xxx**修改
    - Role级别参数：使用**ALTER ROLE xxx SET xxx TO xxx;**修改
    - 会话基本参数：在会话中使用**SET XXX TO XXX**修改

- 本地参数：本地参数保存在每一个postgresql.conf文件（包括：primary和mirror）中，要更新参数，可以使用 gpconfig 命令（如，**gpconfig -c xxx -v xxx**），也可以使用这个命令查看Seg的参数（如，**gpconfig --show xxx**）。

关于参数配置说明，可以参考[服务器配置参数](https://gp-docs-cn.github.io/docs/ref_guide/config_params/guc_config.html)。

## 日志文件

以下方式可以查看GP集群的日志文件：

- 每个Master和Segment实例都在其数据目录的 pg_log中有它们自己的日志文件。
- Master的日志文件包含了大部分信息，应该总是首先检查它。
- gplogfilter工具可以用来检查Greenplum数据库日志文件。 如果要检查segment日志文件，使用gpssh在segment主机上执行gplogfilter工具。默认查找$MASTER_DATA_DIRECTORY 目录下的日志文件，用户也可以手工指定。


# 3. 高可用

## 高可用方案

- 硬件级别RAID：在磁盘级别实现数据冗余
- 数据存储总和校验：该机制是默认开启的，数据被写入磁盘时会计算校验和，下一次读取时检查校验和，从而达到防止磁盘上数据损坏的目的（涉及的配置项包括：**ignore_checksum_failure**和**HEAP_CHECKSUM**）。
- Segment镜像
- Master镜像
- 双集群：双ETL方案、"备份/恢复"方案
- 备份和恢复：gpbackup/gprestore工具备份/恢复Greenplum数据库，[参考](https://greenplum.cn/gp6/managing/backup-gpbackup.html)

### 配置Segment镜像

默认情况下，在GP集群运行时执行gpaddmirrors -p 10000，就能在本集群内创建Segment镜像，执行期间会提示输入mirror数据的存储位置。10000表示mirror服务的端口号在原primary基础上加上10000。

上述命令，以group方式创建mirror，如果用户需要mirror分散部分或者分布在另外的HOST上，那么需要定义文件指定mirror到primary的映射（[参考](https://gp-docs-cn.github.io/docs/admin_guide/highavail/topics/g-enabling-segment-mirroring.html)）。

**gp_segment_configuration**表记录了所有primary和mirror的状态，以及连接信息，这张表常用用于判断mirror的状态。这张表中mode字段描述了Seg的三种状态：

- Change Tracking Mode ：没有找到mirror实例
- resync：重新同步
- in-sync：同步完成

当mirror因为一些原因同步失败时，可以使用 **gprecoverseg** 进行一次增量同步，或者使用 **gprecoverseg -F** 进行全量同步。

**gp_segment_configuration**表的role和preferred_role表示 Segment 表示当前的状态，以及偏好状态。当它们不匹配时，就可能有每台硬件主机上活动主Segment数量造成的倾斜。为了重新平衡该集群并且让所有的Segment回到它们的首选角色，可以用-r选项运行gprecoverseg命令。

当Segment故障时，有以下恢复手段：

- 在Master节点执行**gprecoverseg**，将下线Segment重新上线。gprecoverseg会恢复数据文件，此时数据库的写活动被禁止。
- 当所有Segment状态为**Synchronized**时，可以运行**gprecoverseg -r**使Segment回到它们的首选角色。
- **gprecoverseg -F**是全量恢复手段：从活动segment实例（当前主实例）复制数据前， 删除离线segment实例的数据目录。
- **gprecoverseg -i recover_config_file**将失效Segment恢复到其他主机，[参考](https://greenplum.cn/gp6/highavail/topics/g-when-a-segment-host-is-not-recoverable.html)

### 配置Master镜像

当GP集群正在运行时，通过**gpinitstandby -s {standby_host}**能够快速启动一个Master的镜像。

通过**gpstate -f**，可以检查Master Mirror的运行状态，正常情况下：standby master的状态应该是passive，WAL sender状态应该是streaming。

**需要注意：StandbyMaster不能提供任何服务！**

主master故障时，需要手工执行**gpactivatestandby**（如，gpactivatestandby -d /data/master/gpseg-1）来激活后备Master。激活Master主机后，可以执行**psql dbname -c 'ANALYZE;'**。

关于Master恢复的一些问题：

- 激活后备Master后，官方建议一直将该Master作为主Master使用，并且初始化一个新的后备Master

- 要恢复原来的主Master遵循下面的步骤（下面将原Master主机成为MHost，当前Master主机称为SMHost）：

    - 备份 MHost 上的gpseg-1
    - 在 SMHost 上运行：gpinitstandby -s MHost
    - 检查 MHost 上后备Master的状态：gpstate -f（standby master 状态应该是passive，WAL sender状态应该是streaming）
    - 停止 SMHost 上的Master：gpstop -m （即把当前的主master停掉）
    - 在MHost上运行：gpactivatestandby -d $MASTER_DATA_DIRECTORY（即将当前备升级为主Master）
    - 移除 SMHost 上的gpinitstandby，并在MHost上执行：gpinitstandby -s SMHOST

# 4. 数据备份和恢复

数据备份和恢复有以下两种方式：

- 并行：每台Segment主机都同时把其数据写入到本地的磁盘存储上
- 非并行：数据必须通过网络从Segment被发送到Master，后者把所有的数据写入它的存储中。 

推荐使用并行方式，非并行方式时间上是将GP集群当做一个pg来执行任务。

## gpbackup和gprestore

gpbackup和gprestore在github上是一个[独立项目](https://github.com/greenplum-db/gpbackup/releases)，不属于gpdb工程，其release是两个可执行文件，下载后放到gpadmin用户目录下就可以使用。

gpbackup 和 gprestore 支持以下功能：

- 并行备份恢复
- 全量备份、增量备份
- 备份整个数据库，或者 数据库特定scheme和表
- gpbackup将元数据和数据分开成不同文件可读文件，这些文件放在各个节点上

```shell
# 基本备份操作
./gpbackup --dbname  benchtest --backup-dir /home/gpadmin/backups
# 基本恢复操作
./gprestore  --backup-dir /home/gpadmin/backups/ --timestamp 20191010111727 --create-db --jobs 8
```

其他关于GP集群备份的内容：

- [增量备份](https://greenplum.cn/gp6/managing/backup-gpbackup-incremental.html)
- [在特定存储设备上备份](https://greenplum.cn/gp6/managing/backup-boostfs.html)
- [自定义存储插件](https://greenplum.cn/gp6/managing/backup-plugin-api.html)


# 5. 扩容

对GP集群进行扩容需要注意的几点：

- 当Segment使用Mirror时，一次扩容最少需要两台机器（如果不使用Mirror则没有这种要求）；
- 括容之后Segment需要对表进行重平衡：

    - 重平操作是一次数据重写，会极大消耗磁盘IO，以及占用磁盘空间
    - **表在重平衡期间不可用**
    - 用户可以控制表的重平衡顺序
    - 重平衡不影响新创建的表

- 扩容之前的数据备份文件不可用，需要重新备份

扩容步骤：

- 准备节点：

    - 配置系统变量，必要时候进行性能测试
    - 安装Greenplum软件
    - 创建gpadmin用户
    - 配置SSH免密

- 初始化新节点：这个步骤将新的节点添加到GP集群中

    - 创建扩容文件，这个文件可以手工编辑，也可以通过 **gpexpand** 命令生成（如何生成该文件[参考](https://greenplum.cn/gp6/expand/expand-initialize.html)）。
    - 运行**gpexpand -i input_file**，将扩容实例上线，如果上述过程失败可以执行**gpexpand --rollback**回滚。

- 重分布表：此步骤一旦开始，那么需要重平衡的表变得不再可读写

    - 执行**gpexpand -d 60:00:00**可以开始表扩容操作，-d表示重分布的最大时间限制
    - 进行重分布时，通过gpexpand.status、gpexpand.expansion_progress、gpexpand.status_detail表可以查看重分布表的状态，调整gpexpand.status_detail的rank值还可以控制重分布表的顺序。

- 移除扩容操作

    - gpexpand -c

# 6.数据对象

## 数据库

Greenplum中数据库有**模板**的概念，用户可以从模板创建数据库，新的数据库会拥有模板的所有表和数据。

- 默认模板包括：template1所有新建库的默认模板、template0系统数据库（如postgres）的模板
- 从模板克隆数据库：**CREATE DATABASE new_dbname TEMPLATE old_dbname;**
- 本质上模板和数据库等价，任何数据库都能当做模板
- 查看当前数据库：**\l**、**查看pg_database表**

## 表空间

表空间用于**将数据库中的对象(如表、索引等)到不同的存储介质上**，不同表空间的区别在于**存储介质不同**。

```sql
-- 创建一个表空间，其存放目录要事先创建，并且所有Seg都能访问
CREATE TABLESPACE fastspace LOCATION '/fastdisk/gpdb';
-- 为Role赋权，使他能够在表空间上创建对象
GRANT CREATE ON TABLESPACE fastspace TO admin;
-- 创建表时指定表空间
CREATE TABLE foo(i int) TABLESPACE fastspace;
-- 指定默认表空间
SET default_tablespace = fastspace;
```

查询**pg_tablespace**可以得到当前环境的所有表空间，Greenplum创建之后包含默认表空间：

- pg_default ：默认表空间。由template1和template0数据库使用，存储位置为$PADATA/base/。
- pg_global  ：用于共享系统的catalogs，存储位置为$PADATA/global/。

Greenplum中的表空间和PG是一致的，可以参考这个[文章](https://www.cnblogs.com/lottu/p/9239535.html)。

## SCHEMA

Schema从**逻辑上组织一个数据库中的对象和数据**。 Schema**允许用户在同一个数据库中拥有多于一个对象（例如表）具有相同的名称而不发生冲突**，只要把它们放在不同的Schema中就好，**Public**是默认SCHEMA。

用户可以设置search_path配置参数来指定在其中搜索对象的可用schema的顺序。 在该搜索路径中第一个列出的方案会成为默认schema。 如果没有指定方案，对象会被创建在默认schema中。

```sql

-- 指定查找特定schema下的表
SELECT * FROM myschema.mytable;

-- 设定数据库的搜索顺序
ALTER DATABASE mydatabase SET search_path TO myschema, 
public, pg_catalog;

-- 查看搜索路径、以及schema
SHOW search_path;
SELECT current_schema();
```

## 表

### 表分布策略

支持三种分布策略：

- DISTRIBUTED BY（哈希分布）
- DISTRIBUTED RANDOMLY（随机分布）
- DISTRIBUTED REPLICATED（全分布）

关于分布策略有以下几点需要注意：

- 不显示指定分布策略时，表如何分布取决于[gp_create_table_random_default_distribution](https://gp-docs-cn.github.io/docs/ref_guide/config_params/guc-list.html#gp_create_table_random_default_distribution)

- 使用随机分布时，不能在表中指定PRIMARY KEY 或者 UNIQUE 列
- 对于DISTRIBUTED BY可以自定义操作符

### 表存储模型

- 堆存储：
  
    - 默认配置，模型和PostgreSQL相同
    - 堆表存储在OLTP类型负载下表现最好，适合频繁修改的的小表
    - 行级存储方式

- 追加优化存储：指定appendoptimized=true

    - 成批地被载入并且被只读查询访问的事实表；
    - 不推荐单行的INSERT语句
    - 更新表时有功能限制（如事务中不支持UPDATE和DELETE等）

- 选择面向行或者面向列的存储：列表(appendoptimized=true, orientation=column)

    - 支持行，列或两者的组合
    - 面向列的表存储只能用于追加优化表
    - 频繁的插入时，行表优于列表

- 压缩表： 指定(appendoptimized=true, compresstype=zlib, compresslevel=5);

    - 只适用于追加优化表
    - 可以进行整个表的压缩、或者指定列的压缩

参考介绍：[选择表存储模型](https://greenplum.cn/gp6/ddl/ddl-storage.html)

## 其他对象

- 序列：Greenplum数据库序列对象是一个特殊的单行表，用作数字生成器。 [参考](https://greenplum.cn/gp6/ddl/ddl-sequence.html)
- 索引：[参考](https://greenplum.cn/gp6/ddl/ddl-index.html)
- 视图:[参考](https://greenplum.cn/gp6/ddl/ddl-view.html)


# 参考

[GreenPlum中文社区](https://greenplum.cn/)

[PGSQL MVCC机制](http://mysql.taobao.org/monthly/2017/10/01/)

[Greenplum 表统计信息](https://greenplum.cn/gp6/intro/about_statistics.html)

[PgBouncer客户端配置](https://greenplum.cn/gp6/access_db/topics/pgbouncer.html)

[Greenplum数据库参考指南](https://gp-docs-cn.github.io/docs/ref_guide/ref_guide.html)

[Greenplum系统表表结构](https://gpdb.docs.pivotal.io/6-0/ref_guide/system_catalogs/gp_segment_configuration.html)

[gprestore和gpbackup介绍](https://greenplum.cn/gp6/managing/backup-gpbackup.html)