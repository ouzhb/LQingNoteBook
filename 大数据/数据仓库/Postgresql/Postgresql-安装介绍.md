---
title: 数据库调研笔记 -- PostgreSQL
categories: "笔记"
date: 2019-09-24
comments: true
toc: true
tags:
	- PostgreSQL
	- 数据库
---


PostgreSQL 调研笔记


<!--more-->

# Quick-Start

## 安装部署

```shell

yum install https://download.postgresql.org/pub/repos/yum/reporpms/EL-7-x86_64/pgdg-redhat-repo-latest.noarch.rpm
yum -y install postgresql11 postgresql11-server

# 安装完成后，安装目录为/usr/pgsql-11，postgresql-11-setup脚本用来进行环境初始化和升级操作。
# 需要注意的是：安装完RPM包后，如果想要修改默认的数据目录（/var/lib/pgsql/11/data），需要修改/usr/lib/systemd/system/postgresql-11.service中的环境变量PGDATA，并执行systemctl daemon-reload。
# postgresql-11-setup会创建postgres和postgres，并且设定用户的home目录为/var/lib/pgsql

/usr/pgsql-11/bin/postgresql-11-setup initdb
systemctl enable postgresql-11

# 修改 pg_hba.conf文件，放开用户远程登录权限，注意同时要放开listen_addresses配置为*
# host    all             root            all                     md5
# host    all             postgres        172.24.9.1/24           md5
#
systemctl start postgresql-11

# 修改postgres用户登录密码，创建root用户
sudo -u postgres psql -d postgres -c "ALTER USER postgres WITH PASSWORD 'rjbigdata_admin';"
sudo -u postgres psql -d postgres -c "CREATE ROLE root LOGIN REPLICATION CREATEDB PASSWORD 'rjbigdata'"
```

关于系统配置的最佳实践：

- 建议将硬盘挂载点的Owner设置为PostgreSQL用户，数据目录直接放置在该目录之下（有利于避免升级、clean时的权限问题）。
- 使用NAS文件系统时可以会导致数据损坏（官方建议 synchronously 方式挂载，并且关闭caching），参考[NFS的可能的问题](https://www.postgresql.org/docs/11/creating-cluster.html)。
- 关于型号量和共享内存的配置（[参考](http://www.postgres.cn/docs/10/kernel-resources.html)）,Linux涉及到的配置需要关注的有kernel.shmmax（最大段尺寸）和kernel.shmall（最大共享内存页面）
- 使用systemd必须注意IPC资源（共享内存和信号量） 不会被操作系统过早删除，默认情况下要避免这种情况需要将启动pg的用户设定为系统用户（id小于1000）以及修改/etc/systemd/logind.conf 中RemoveIPC=no。（参考[18.4.2. systemd RemoveIPC](https://www.postgresql.org/docs/11/kernel-resources.html)）
- 资源限制：maxproc、openfiles、datasize
- 防止PG在内存过渡调拨时Killer
	
	- 设定systemd文件中的PG_OOM_ADJUST_VALUE=-1000，这样保证子进程不被Killer ；
	- 设定systemd文件中的PG_OOM_ADJUST_FILE=/proc/self/oom_score_adj 的值为-1000（echo -1000 > /proc/self/oom_score_adj），保证postmaster不被kill
	- 降低PG内存相关配置，如shared_buffers 和work_mem）

- shared_buffers配置较大时，可以开启大页配置。评估页面数vm.nr_hugepages，可以参考 postmaster VmPeak / Hugepagesize 

- 关闭数据库时，发送不同的信号量，PG关闭的方式不同（kill -INT `head -1 /usr/local/pgsql/data/postmaster.pid`），**不要用-9关闭PG，危！！**

	- SIGTERM：智能关闭模式，不在接收新连接、会让现有的会话正常结束它们的工作。仅当所有的会话终止后它才关闭。 
	- SIGINT：服务器不再允许新的连接，并向所有现有服务器进程发送SIGTERM，让它们中断当前事务并立刻退出。然后服务器等待所有服务器进程退出并最终关闭。 如果服务处于在线备份模式，备份模式将被终止并致使备份无用。
	- SIGQUIT：服务器将给所有子进程发送 SIGQUIT并且等待它们终止。如果有任何进程没有在 5 秒内终止，它们将被发送 SIGKILL。主服务器进程将在所有子进程退出之后立刻退出，而无需做普通的数据库关闭处理。这将导致在下一次启动时（通过重放 WAL 日志）恢复。

## 账号管理

- PostgreSQL 基于 roles 对数据库用户进行权限管理。
- 根据roles的创建方式不同，可以指特定用户或者某一组用户，即包含user和groups两个概念）。
- Roles 基于可以和数据库的Objects绑定，或者将名下objects的权限赋予其他roles。
- Roles 与操作系统的用户是完全分开的，不会相互影响。
- 系统预创建的超级用户为 postgres ，可以 su - postgres 切换到该用户后登录pg。
- 用户也可以在执行psql命令时，使用-U指定登录的用户。

```sql
CREATE ROLE name;													-- 创建ROLE，等价于使用createuser name
DROP ROLE name;														-- 删除ROLE，等价于使用dropuser  name
SELECT rolname FROM pg_roles; 										-- 查询已有roles
CREATE USER name;													-- 创建用户
ALTER ROLE XXX 														-- 修改用户权限
CREATE ROLE root LOGIN REPLICATION CREATEDB CREATEEXTTABLE PASSWORD 'rjbigdata'    -- 创建一个roles，并赋予各种权限
GRANT group_role TO role1, ... ;									-- role赋权
REVOKE group_role FROM role1, ... ;									-- role回收权限
DROP ROLE name;
-- 关于Roles权限继承的实例，其中joe被设计成user，admin、wheel被设计成Role Group：

CREATE ROLE joe LOGIN INHERIT;				-- joe可以重其他roles中继承权限
CREATE ROLE admin NOINHERIT;				-- 不允许从其他roles中继承权限
CREATE ROLE wheel NOINHERIT;
GRANT admin TO joe;
GRANT wheel TO admin;
SET ROLE admin / SET ROLE wheel; 			-- 获取admin和wheel的权限
SET ROLE joe / SET ROLE NONE / RESET ROLE;  -- 恢复权限
```

### 权限类型

PostgreSQL中的权限（pg_roles中有roles的权限明细）：

- login privilege：roles有该权限时，可以作为一个普通用户登录
- superuser status：除了login privilege以外所有权限
- database creation
- role creation
- initiating replication：流复制的角色权限？？ 用来副本同步？
- password
- INHERIT：具有INHERIT属性的成员角色会自动使用其所属成员角色的特权，通常用来在pg中区别roles和users

### 删除账号

删除账号时有以下注意点：

- 需要将roles名下所有objects收回（ALTER TABLE bobs_table OWNER TO alice;）
- REASSIGN OWNED 可以将一个role名下所有object转义给另一个object
- DROP OWNED 删除role名下所有的object

### 默认Roles

PG提供的一些默认Role，[参考](https://www.postgresql.org/docs/11/default-roles.html)，这些Role名下关联了许多系统表。

## Client 认证

- 用户认证相关的配置文件为：pg_hba.conf，如何配置可以[参考](https://www.postgresql.org/docs/11/auth-pg-hba-conf.html)
- PG支持password、ldap、gss等方式的认证

```
# 配置pg允许远程连接，pg_hba.conf中追加下面一行
host    all             root            all                     md5
```

## SQL Language


```sql

-- 建表,支持的类型包括：int，smallint，实数，双精度，char（N），varchar（N），date, time, timestamp, interval，以及自定义类型
CREATE TABLE weather (
    city            varchar(80),
    temp_lo         int,           -- low temperature
    temp_hi         int,           -- high temperature
    prcp            real,          -- precipitation
    date            date
);

-- 插入
INSERT INTO weather VALUES ('San Francisco', 46, 50, 0.25, '1994-11-27');
INSERT INTO weather (date, city, temp_hi, temp_lo) VALUES ('1994-11-29', 'Hayward', 54, 37);

-- 文件批量导入，后端进程直接导入
COPY weather FROM '/home/user/weather.txt';
```
## 设置参数

- postgresql.conf文件可以通过pg_ctl reload命令，或者 pg_reload_conf() 函数重载（部分配置可能要重启生效）

	- postgresql.conf文件可以包含 include 'filename' 配置、include_dir 'directory'配置
	- 

- postgresql.auto.conf不应该手工编辑，这个文件保存了通过ALTER SYSTEM命令提供的设置，并且会覆盖postgresql.conf中的配置
- 通过SQL进行配置：ALTER SYSTEM（全局配置，等效于配置文件）、ALTER DATABASE、ALTER ROLE
- show/set 命令可以查看当前会话的配置，以及针对会话更新配置。
- 服务端启动时可以使用-c 指定配置，这些配置覆盖ALTER SYSTEM和配置文件配置
- 启动Client时可以使用环境变量指定，如（env PGOPTIONS="-c geqo=off -c statement_timeout=5min" psql）

### 配置内容

- 文件位置：[参考](http://www.postgres.cn/docs/10/runtime-config-file-locations.html)
- 连接和认证：[参考](http://www.postgres.cn/docs/10/runtime-config-connection.html)
    
	- max_connections：最大连接数配置，默认是100，实际user的最大连接数为max_connections - superuser_reserved_connections 

- 资源消耗：[参考](http://www.postgres.cn/docs/10/runtime-config-resource.html)
	
	- 内存配置：
		- shared_buffers : 一个合理的shared_buffers开始值是系统内存的 25%。默认是128mb
		- work_mem ：内部排序操作和哈希表使用的内存量，默认4mb。一个查询可能有好几个排序或者hash操作，每个操作会使用work_mem大小的内存。
		- maintenance_work_mem ：维护性操作（例如VACUUM、CREATE INDEX和ALTER TABLE ADD FOREIGN KEY）中使用的 最大的内存量，默认64mb。
		- autovacuum_work_mem : 指定每个自动清理工作者进程能使用的最大内存量。
		- temp_buffers ：每个数据库会话使用的临时缓冲区的最大数目，是会话的本地缓冲区，只用于访问临时表，默认是 8 兆字节（8MB）。
	- 工作线程配置：max_worker_processes 、 max_parallel_workers 、max_parallel_workers_per_gather （[并行查询参考](http://www.postgres.cn/docs/10/runtime-config-resource.html)）
	- 强制刷盘配置：backend_flush_after（默认不强制刷盘）
- 日志相关配置，[参考](http://www.postgres.cn/docs/10/runtime-config-wal.html)
- 流复制配置,[参考](http://www.postgres.cn/docs/10/runtime-config-replication.html)
- 其他配置，[参考](http://www.postgres.cn/docs/10/runtime-config.html)



# 高可用方案

各种方案[一览表](http://www.postgres.cn/docs/10/different-replication-solutions.html#HIGH-AVAILABILITY-MATRIX)

# 参考

[安装部署](https://www.postgresql.org/download/)

[Pgpool-II](https://www.pgpool.net/mediawiki/index.php/Main_Page)

[官方参考资料 & 论文集](https://www.postgresql.org/docs/11/biblio.html)

[PG的编码集配置](https://www.postgresql.org/docs/11/charset.html)

[PostgreSQL 10.1 手册](http://www.postgres.cn/docs/10/)

