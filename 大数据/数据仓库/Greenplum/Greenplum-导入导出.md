---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-11
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 数据装载

## 1. INSERT语句

INSERT语句只适合在小规模的堆表场景中使用，并且有以下特点：

- 单个INSERT命令中插入多行，如：
    ```sql
    INSERT INTO products (product_no, name, price) VALUES
    (1, 'Cheese', 9.99),
    (2, 'Bread', 1.99),
    (3, 'Milk', 2.99);
    ```
- 对于追加优化表， Greenplum数据库支持最多127个并发INSERT 事务插入到一个追加优化表。

### 性能测试

通过pgbench和sysbench进行基于INSERT、UPDATE等简单SQL的性能测试，有以下发现：

- 测试环境（32核CPU）中插入性能的瓶颈是CPU，50个线程时，单表TPS为15000~16000，单表单线程TPS为500。PS：单块SSD似乎最先到达瓶颈，但是对Greenplum进行扩容性能没有明显提升。
- **关闭optimizer配置能够极大提升简单SQL的性能，测试中optimizer配置开启时性能只有原来20%，并且巨量消耗Master的CPU资源。PS：这个配置默认开启！**
- Master节点上每个Client连接会产生多个Postgres进程（似乎和Segment数量有关）
- 测试中一条INSERT插入多行数据对TPS影响不大
- gp_enable_global_deadlock_detector = on 时可以极大的提升UPDATE操作的性能（10倍以上）

关于OLTP的性能测试，Greenplum官方给出了基于Greenplum 6的测试教程，测试在48核的 Master上得到了单表18000 TPS以上的性能。测试结果和实际基本符合。

参考：[官方文档](https://greenplum.org/oltp-workload-performance-improvement-in-greenplum-6/)、[中文社区翻译](https://greenplum.cn/2019/05/14/greenplum-6-oltp-60x/)

## 2. COPY语句

 COPY命令是非并行操作，数据流需要通过Master实例。

 - COPY只能用于表，不能用于视图。
 - ，支持常用的文件格式，如：txt、sql、csv、压缩文件、二进制格式等

```sql
-- COPY 语句只能在master节点上运行
COPY table_name FROM '/path/to/filename' WITH (FORMAT csv);
-- \copy 语句可以在client节点上运行，其基于COPY FROM STDIN语句从STDIN读取输入并发送给Master
\copy table_name FROM '/path/to/filename';

-- 使用COPY命令导出数据
COPY (SELECT * FROM pgbench_accounts limit 10) TO '/home/gpadmin/pgbench_accounts.csv' WITH csv;
```
默认情况下，COPY会在第一个错误处停止操作：如果数据包含一个错误，该操作失败并且没有数据被装载(即没有一条数据被导入)。

用户可以使用单行错误隔离模式，Greenplum会跳过包含格式错误的行并且装载正确格式化的行。需要注意的是：这里的错误指数据格式的错误，不包含约束错误。

```sql
-- 使用单行错误隔离模式，每个segment容许10行错误
COPY country FROM '/data/gpdb/country_data' 
   WITH DELIMITER '|' LOG ERRORS
   SEGMENT REJECT LIMIT 10 ROWS;
```

### 性能测试

使用COPY命令**单线程**，导入**1亿**条数据，数据原始大小9.7GB，用时136692.713 ms，平均**731567条/s**。

测试过程中，磁盘性能优先达到瓶颈。

调优建议：

- 在装载数据到新表中时，最后创建索引。
- 装载完成后执行VACUUM ANALYZE来为查询优化器更新表统计信息
- 在装载错误后运行VACUUM，清理缓存表。
## 3. 外部表并行读写

通过外部表，可以使向Greenplum导入导出数据并行化，使Greenplum真正具备并行读写的能力。

导入数据：
    
    - 准备数据文件
    - 创建外部表（初次导入时还要创建数据表）
    - INSERT INTO tablename  SELECT * from tablename_ext_temp;
    - 删除外部表、外部数据文件

### 外部表

根据外部表允许的操作，包括：

- 可读外部表：仅允许SELECT操作。
- 可写外部表：仅允许INSERT操作

根据外部表数据源的状态，包括：

- 普通（基于文件的）：访问静态平面文件
- Web（基于Web的）：访问动态数据源

    - 基于命令的web表：建表时将table关联到可执行脚本，每次查询时基于脚本的返回结果，因此该表对应的返回值是动态的，用户可以指定master或者特定segments运行这个脚本。

    - 基于URL的web表：定义表时LOCATION里定义基于http://协议的web服务文件路径，指定的http地址数目取决于GP集群中的Segment数目。

当前支持的外部表协议：

|协议类型|表类型|数据存放位置|说明|
|----|----|----|----|
|file|只能是可读表|Segment主机|每个Segment只能处理一个外部文件，所以单个Seg-Host上的文件数目，取决于运行Segment实例数目|
|gpfdist/gpfdists|可读/写表|gpfdist服务器（一个或者多个）|跨主机协议、支持数据压缩和数据转换|
|pxf、S3|自定义接口|Hadoop系统、对象存储等||

定义外部表：

```sql
-- file 外部表
CREATE EXTERNAL TABLE ext_expenses (
   name text, date date, amount float4, category text, desc1 text ) 
LOCATION ('file://host1:5432/data/expense/*.csv', 
          'file://host2:5432/data/expense/*.csv', 
          'file://host3:5432/data/expense/*.csv') 
FORMAT 'CSV' (HEADER); 

-- 基于命令的web外部表

CREATE EXTERNAL WEB TABLE log_output 
    (linenum int, message text) 
    EXECUTE '/var/load_scripts/get_log_data.sh' ON HOST
    FORMAT 'TEXT' (DELIMITER '|');

```

### gpfdist

通常情况下，gpfdist作为一个第三方服务运行在Greenplum集群之外的服务器上（一般是ETL服务器）。Greenplum和gpfdist服务之间通过HTTP/HTTPS协议通信。


``` shell
# 启动gpfdist服务，配置数据目录和日志目（建议先安装Greenplum，否则启动可能缺依赖）
gpfdist -d /data/data_ssd/gpfdist_files -p 18081 -l /var/log/gpfdist/gpfdist.log &
```

gpfdist的优势：

- 外部文件支持压缩、CSV等格式
- 支持将外部XML（json）文件读入Greenplum数据库（通过配置YAML格式的文件）
- 外部表可以连接一个或多个gpfdist实例（无论一个还是多个，Segment连接外部表时均是并行的）
- 一台ETL服务器上可以运行多个gpfdist实例（不同的数据目录，以及端口）

性能控制：

- gp_external_max_segs：数控制能同时访问单一gpfdist实例的Segment实例数量，默认64个；

### gpload

gpload是Greenplum提供的并行导入工具，工作原理基于gpfdist，用户通过定义YAML文件来控制gpload导入的表结构。

- 支持多种表导入模式
    - INSERT
    - UPDATE
    - MERGE
- gpload会在外部文件上重新拉起gpfdist进程，知道导入完成
- gpload会创建一张临时外部表，因此执行用户要有建外部表权限，以及写入数据权限


gpload的导入命令为:

```shell
gpload -f insert.yaml -l gpload.log
```
gpload使用的控制文件如下:
```yaml

---
VERSION: 1.0.0.1
DATABASE: pgbench
USER: benchtest
HOST: 172.24.9.12
PORT: 5432
GPLOAD:
   INPUT:
    - SOURCE:
         LOCAL_HOSTNAME:
           - 172.24.33.35
         PORT: 19090
         FILE:
           - /data/data_ssd/gpfdist_files/pgbench/pgbench_accounts/*
    - COLUMNS:
               - aid: integer
               - bid: integer
               - abalance: integer
               - filler: character(84)
    - FORMAT: csv
    - DELIMITER: ','
    - QUOTE: '"'
    - HEADER: false
   OUTPUT:
    - TABLE: public.pgbench_accounts_gpload_ins
    - MODE: INSERT
```

详细参数说明参考[官方文档](https://gp-docs-cn.github.io/docs/utility_guide/admin_utilities/gpload.html)


### 性能测试

使用**单点gpfdist服务**将外部表导入为GP的系统，导入**1亿**条数据，数据原始大小9.7GB，用时35962.171 ms，平均**2780699条/s**。


```sql
-- 创建内部堆表
CREATE TABLE pgbench_accounts (
    aid integer NOT NULL,
    bid integer,
    abalance integer,
    filler character(84)
) WITH (fillfactor='100') DISTRIBUTED BY (aid);

--  创建外部表
CREATE EXTERNAL TABLE pgbench_accounts_ext_tmp ( 
    aid integer, 
    bid integer, 
    abalance integer, 
    filler character(84) 
) LOCATION ('gpfdist://172.24.33.35:18081/pgbench/pgbench_accounts/*') FORMAT 'csv';
-- 导入堆表
INSERT INTO pgbench_accounts  SELECT * from pgbench_accounts_ext_tmp;
```


# 使用gpfdist和gpload转换外部数据

Greenplum支持将任意格式的数据导入到数据中，或者将数据库中表以任意格式导出，以下说明导入XML到Greenplum的表中。

XML文件内容（文件名为pricerecord.xml）如下，包含：itemnumber和price两个字段。

```xml
<?xml version="1.0" encoding="ISO-8859-1" ?>
<prices>
  <pricerecord>
    <itemnumber>708421</itemnumber>
    <price>19.99</price>
  </pricerecord>
  <pricerecord>
    <itemnumber>708466</itemnumber>
    <price>59.25</price>
  </pricerecord>
  <pricerecord>
    <itemnumber>711121</itemnumber>
    <price>24.99</price>
  </pricerecord>
</prices>
```
对应的表结构为：
```sql
CREATE TABLE prices (
	itemnumber integer,       
	price      decimal        
) DISTRIBUTED BY (itemnumber);
```

进行导入前，用户需要准备一个脚本工具解析XML文档，该工具不限格式，应当有如下输出：

```shell
# 需要注意的是：工具输出中不能带空行
708421|19.99
708466|59.25
711121|24.99
```

用户可以定义config.xml文件，将声明脚本解析工具，该文件中的参数[配置文件格式](https://greenplum.cn/gp6/load/topics/transforming-xml-data.html)：

```yaml
---
VERSION: 1.0.0.1
TRANSFORMATIONS:
    transformation_name1:              # 转换名称
        TYPE:     input               # 转换类型，input或者output
        COMMAND:  /bin/sh trans_script.sh %filename%     # 转换命令

    transformation_name2:         
        TYPE:     output              
        COMMAND:  /bin/sh trans_script.sh %filename% 

-- COMMAND中的 %filename% 在执行是被gpload配置文件中的定义替换
```

创建gpload配置文件如下：

```yaml
---
VERSION: 1.0.0.1
DATABASE: pgbench
USER: benchtest
HOST: 172.24.9.12
PORT: 5432
GPLOAD:
   INPUT:
     - TRANSFORM_CONFIG: config.yaml   # 定义转换配置
     - TRANSFORM: prices_input         # 定义要使用的转换
     - SOURCE:
         LOCAL_HOSTNAME:
           - 172.24.9.12
         PORT: 19090
         FILE: 
           - pricerecord.xml           # 待导入的xml文件（替换%filename% ）
     - COLUMNS:
         - itemnumber: integer
         - price: decimal
     - FORMAT: TEXT
     - DELIMITER: '|'
     - QUOTE: '"'
     - HEADER: false
   OUTPUT:
     - TABLE: public.prices
     - MODE: INSERT
```

**PS：用户也可以执行 gpfdist -c config.yaml 将装换加载到gpfdist中，创建外表直接读取xml文档**

```sql
CREATE READABLE EXTERNAL TABLE prices_readable (LIKE prices)
   LOCATION ('gpfdist://hostname:8080/prices.xml#transform=prices_input')
   FORMAT 'TEXT' (DELIMITER '|')
   LOG ERRORS SEGMENT REJECT LIMIT 10;
```

# 数据导出

使用**CREATE WRITABLE EXTERNAL TABLE**命令定义外部表时，可以将数据导出到本地。

- Segment把数据发送到gpfdist，后者会把数据写到指定的文件中；
- 外部表定义中定义多个gpfdist URI时，输出数据划分到多个文件之间；
- 可写的外部Web表把输出行发送到一个脚本（或者应用）作为输入。

```sql
-- 写入到gpfdist的外部表
CREATE WRITABLE EXTERNAL TABLE unload_expenses  ( LIKE expenses ) 
   LOCATION ('gpfdist://etlhost-1:8081/expenses1.out', 
             'gpfdist://etlhost-2:8081/expenses2.out')
FORMAT 'TEXT' (DELIMITER ',') DISTRIBUTED BY (exp_id);

-- 写入到HDFS的外部表
CREATE WRITABLE EXTERNAL TABLE unload_expenses  ( LIKE expenses ) 
   LOCATION ('pxf://dir/path?PROFILE=hdfs:text') 
FORMAT 'TEXT' (DELIMITER ',') DISTRIBUTED BY (exp_id);

-- 可写外部web表，EXECUTE程序处理insert到这个表的每一行
-- 参考https://greenplum.cn/gp6/load/topics/g-defining-a-command-based-writable-external-web-table.html
CREATE WRITABLE EXTERNAL WEB TABLE output (output text) 
    EXECUTE 'export PATH=$PATH:/home/gpadmin/programs; myprogram.sh' 
FORMAT 'TEXT' DISTRIBUTED RANDOMLY

-- 向外部表写入

GRANT INSERT ON writable_ext_table TO admin;
INSERT INTO writable_ext_table SELECT * FROM regular_table;
```

直接用COPY命令也可以导出，但是此时性能瓶颈受限于Master服务：

```sql
COPY (SELECT * FROM country WHERE country_name LIKE 'A%') 
TO '/home/gpadmin/a_list_countries.out';
```



# PGBench测试工具

pgbench是 PostgreSQL 上自带一个基准测试工具，能够让用户并发执行多次SQL语句，并且统计测试的TPS。

默认情况下，如果用户不提供自定义的测试SQL，那么测试时使用TPC-B方式进行OLTP测试，执行

```sql

-- 初始化测试数据库，比例因子-s（系数是10万），执行后pgbench中包括以下几张表：
--
--  table                   # of rows
--  ---------------------------------
--  pgbench_branches        1 * 1000
--  pgbench_tellers         10 * 1000
--  pgbench_accounts        100000 * 1000
--  pgbench_history         0 * 1000

pgbench -i -s 1000 pgbench

-- 重要的测试选项包括-c（客户端数量）、 -t（事务数量）、-T（时间限制）以及-f（指定一个自定义脚本文件）

-- pgbench 默认有三个内建脚本，分别是：tpcb-like、simple-update、select-only
pgbench -c 100 -j 100 -r -T 60 -P 1 -s 1000 -b tpcb-like pgbench
pgbench -c 100 -j 100 -r -T 60 -P 1 -s 1000 -b simple-update pgbench
pgbench -c 100 -j 100 -r -T 60 -P 1 -s 1000 -b select-only pgbench
```

以下是自定义测试脚本：

```
\set scale 10000
\set nbranches 1 * :scale
\set ntellers 10 * :scale
\set naccounts 100000 * :scale
\set aid random(1,:naccounts)
\set bid random(1,:nbranches)
\set tid random(1,:ntellers)
\set delta random(-5000,5000)

INSERT INTO pgbench_history (tid, bid, aid, delta, mtime) VALUES (:tid, :bid, :aid, :delta, CURRENT_TIMESTAMP);
```

# 


## 附：测试环境

服务器：6 * 3 Segment（Master和Segment混合部署，并且共用一块SSD，并且配置mirror）
CPU：Intel(R) Xeon(R) CPU E5-2620 v4 @ 2.10GHz 32核
内存：  125G    
网络：10GB光纤

## 附：创建外部表

```sql
-- 普通可读外部表
CREATE [READABLE] EXTERNAL TABLE table_name    
    ( column_name data_type [, ...] | LIKE other_table )
      LOCATION ('file://seghost[:port]/path/file' [, ...])
        | ('gpfdist://filehost[:port]/file_pattern[#transform]'
        | ('gpfdists://filehost[:port]/file_pattern[#transform]'
            [, ...])
        | ('gphdfs://hdfs_host[:port]/path/file')
      FORMAT 'TEXT'
            [( [HEADER]
               [DELIMITER [AS] 'delimiter' | 'OFF']
               [NULL [AS] 'null string']
               [ESCAPE [AS] 'escape' | 'OFF']
               [NEWLINE [ AS ] 'LF' | 'CR' | 'CRLF']
               [FILL MISSING FIELDS] )]
           | 'CSV'
            [( [HEADER]
               [QUOTE [AS] 'quote']
               [DELIMITER [AS] 'delimiter']
               [NULL [AS] 'null string']
               [FORCE NOT NULL column [, ...]]
               [ESCAPE [AS] 'escape']
               [NEWLINE [ AS ] 'LF' | 'CR' | 'CRLF']
               [FILL MISSING FIELDS] )]
           | 'AVRO'
           | 'PARQUET'
 
           | 'CUSTOM' (Formatter=<formatter specifications>)
     [ ENCODING 'encoding' ]
     [ [LOG ERRORS [INTO error_table]] SEGMENT REJECT LIMIT count
       [ROWS | PERCENT] ]

-- web可读外部表，每次读的数据动态变化
CREATE [READABLE] EXTERNAL WEB TABLE table_name    
   ( column_name data_type [, ...] | LIKE other_table )
      LOCATION ('http://webhost[:port]/path/file' [, ...])
    | EXECUTE 'command' [ON ALL
                          | MASTER
                          | number_of_segments
                          | HOST ['segment_hostname']
                          | SEGMENT segment_id ]
      FORMAT 'TEXT'
            [( [HEADER]
               [DELIMITER [AS] 'delimiter' | 'OFF']
               [NULL [AS] 'null string']
               [ESCAPE [AS] 'escape' | 'OFF']
               [NEWLINE [ AS ] 'LF' | 'CR' | 'CRLF']
               [FILL MISSING FIELDS] )]
           | 'CSV'
            [( [HEADER]
               [QUOTE [AS] 'quote']
               [DELIMITER [AS] 'delimiter']
               [NULL [AS] 'null string']
               [FORCE NOT NULL column [, ...]]
               [ESCAPE [AS] 'escape']
               [NEWLINE [ AS ] 'LF' | 'CR' | 'CRLF']
               [FILL MISSING FIELDS] )]
           | 'CUSTOM' (Formatter=<formatter specifications>)
     [ ENCODING 'encoding' ]
     [ [LOG ERRORS [INTO error_table]] SEGMENT REJECT LIMIT count
       [ROWS | PERCENT] ]
 
-- 普通可写外部表
CREATE WRITABLE EXTERNAL TABLE table_name
    ( column_name data_type [, ...] | LIKE other_table )
     LOCATION('gpfdist://outputhost[:port]/filename[#transform]'
      | ('gpfdists://outputhost[:port]/file_pattern[#transform]'
          [, ...])
      | ('gphdfs://hdfs_host[:port]/path')
      FORMAT 'TEXT'
               [( [DELIMITER [AS] 'delimiter']
               [NULL [AS] 'null string']
               [ESCAPE [AS] 'escape' | 'OFF'] )]
          | 'CSV'
               [([QUOTE [AS] 'quote']
               [DELIMITER [AS] 'delimiter']
               [NULL [AS] 'null string']
               [FORCE QUOTE column [, ...]] ]
               [ESCAPE [AS] 'escape'] )]
           | 'AVRO'
           | 'PARQUET'
 
           | 'CUSTOM' (Formatter=<formatter specifications>)
    [ ENCODING 'write_encoding' ]
    [ DISTRIBUTED BY (column, [ ... ] ) | DISTRIBUTED RANDOMLY ]
 
-- web可写外部表
CREATE WRITABLE EXTERNAL WEB TABLE table_name
    ( column_name data_type [, ...] | LIKE other_table )
    EXECUTE 'command' [ON ALL]
    FORMAT 'TEXT'
               [( [DELIMITER [AS] 'delimiter']
               [NULL [AS] 'null string']
               [ESCAPE [AS] 'escape' | 'OFF'] )]
          | 'CSV'
               [([QUOTE [AS] 'quote']
               [DELIMITER [AS] 'delimiter']
               [NULL [AS] 'null string']
               [FORCE QUOTE column [, ...]] ]
               [ESCAPE [AS] 'escape'] )]
           | 'CUSTOM' (Formatter=<formatter specifications>)
    [ ENCODING 'write_encoding' ]
    [ DISTRIBUTED BY (column, [ ... ] ) | DISTRIBUTED RANDOMLY ]
```

# 参考

[GP系统配置参数](https://cloud.tencent.com/developer/article/1447227)

[XML转换示例](https://greenplum.cn/gp6/load/topics/transforming-xml-data.html)