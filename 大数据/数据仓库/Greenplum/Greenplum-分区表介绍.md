---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-25
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 分区表

Greenplum 支持分区表，但是**分区**的概念需要和**分布式**区别开，Greenplum中所有表都是分布式的（分布在不同Segment上），但是并非所有表都是分区表。

分区是进行**逻辑划分**，而分布是表的**物理划分**，前者减少查询的数据扫描量，后者提供并行查询能力。

对Greenplum来说分区表是通过表继承、规则、约束来实现的。

![](https://greenplum.cn/gp6/graphics/partitions.jpg)

- 分区操作会创建一个顶层（父）表以及一层或者多层子表；
- 这些父表和子表都可以独立查询，子表看上去和普通表无异；
- 使用pg_relation_size函数查询父表的存储空间时，大小为0；
- 父表和子表之间存在继承关系（即表结构、NOT NULL，DEFAULT，CHECK三种约束），修改父表的表结构子表会同步修改；
- 创建分区后可以用带有PARTITION子句的ALTER TABLE修改父表；
- 可以在父表插入语句，或者直接插入到子表。直接插入子表时会触发check检查，如果检查失败会返回一个错误；
- 删除父表时，子表会一并删除
- 复制表复制表不能进行分区；
- 多级分区可以基于同一个分区键，如可以基于年、月、日进行三级分区；
- 已有表不能改为分区表，用户需要创建新的表将原有表的数据导入；
- 分区表最多能有32,767个分区；
- 分区表上的主键或者唯一约束必须包含所有的分区列；

分区策略：**除非查询优化器能基于查询谓词排除一些分区，否则分区技术不能改进查询性能。**

对外部表进行分区时会产生一些限制：参考[分区表的限制](https://greenplum.cn/gp6/ddl/ddl-partition.html#topic80)



## 分区操作

### 默认分区

- 默认分区用来装载其他分区CHECK约束失败的行；
- 优化器在扫描时总会扫描默认分区（因此默认分区会影响分区表性能；
- 多级分区，一旦存在默认分区，那么每一级都需要保存默认分区；
- 含有默认分区时，用户可以从默认分区中分裂出新的分区；

### pg_partitions视图

- pg_partition：跟踪分区表以及它们的继承层次关系。
- pg_partition_templates：展示使用一个子分区模板创建的子分区。
- pg_partition_columns：显示在一个分区设计中用到的分区键列。

### 分区表操作

增加分区：

```sql
-- 假设原有分区范围为“2016-01-01 - 2017-02-01”，下面的语句将分区范围拓宽
ALTER TABLE sales ADD PARTITION 
    START (date '2017-02-01') INCLUSIVE 
    END (date '2017-03-01') EXCLUSIVE;、

ALTER TABLE sales ADD PARTITION 
            START (date '2017-02-01') INCLUSIVE 
            END (date '2017-03-01') EXCLUSIVE
      ( SUBPARTITION usa VALUES ('usa'), 
        SUBPARTITION asia VALUES ('asia'), 
        SUBPARTITION europe VALUES ('europe') );

-- 为sales表的第12分区，添加一个2级分区
ALTER TABLE sales ALTER PARTITION FOR (RANK(12))
      ADD PARTITION africa VALUES ('africa');

-- 添加默认分区
ALTER TABLE sales ADD DEFAULT PARTITION other;

```

重命名分区：

```sql

-- 分区表使用下列命名习惯：<parentname>_<level>_prt_<partition_name>
-- 重命名父表，子表会跟着修改
ALTER TABLE sales RENAME TO globalsales;
-- 修改表的分区名
ALTER TABLE sales RENAME PARTITION FOR ('2016-01-01') TO jan16;

```

删除分区：

```sql
-- 删除分区
ALTER TABLE sales DROP PARTITION FOR (RANK(1));
-- trancate分区
ALTER TABLE sales TRUNCATE PARTITION FOR (RANK(1));
```

交换分区(参考[用外部表交换叶子子分区](https://greenplum.cn/gp6/ddl/ddl-partition.html#topic80))：

```sql
-- 将分区sales_1_prt_1和jan12表交换，此后jan12表成为sales的分区
CREATE TABLE jan12 (LIKE sales) WITH (appendoptimized=true);
INSERT INTO jan12 SELECT * FROM sales_1_prt_1 ;
ALTER TABLE sales EXCHANGE PARTITION FOR (DATE '2012-01-01') 
WITH TABLE jan12;
```

分裂分区：

```sql
-- 分裂普通分区
ALTER TABLE sales SPLIT PARTITION FOR ('2017-01-01') AT ('2017-01-16')
INTO (PARTITION jan171to15, PARTITION jan1716to31);

-- 分裂默认分区
ALTER TABLE sales SPLIT DEFAULT PARTITION 
START ('2017-01-01') INCLUSIVE 
END ('2017-02-01') EXCLUSIVE 
INTO (PARTITION jan17, default partition);
```

修改分区模板：修改后原有的分区不发生变化，参考[修改子分区模板](https://greenplum.cn/gp6/ddl/ddl-partition.html#topic80)


## 例子

日期范围分区：使用单个date或者timestamp列作为分区键

```sql
-- 在[START,END)范围内进行分区，每个分区的长度是‘1 day’，因此产生365个分区
CREATE TABLE sales (id int, date date, amt decimal(10,2))
DISTRIBUTED BY (id)
PARTITION BY RANGE (date)
( START (date '2016-01-01') INCLUSIVE  
   END (date '2017-01-01') EXCLUSIVE
   EVERY (INTERVAL '1 day') );

-- 指定生成按月的分区，显示指定分区范围
CREATE TABLE sales (id int, date date, amt decimal(10,2))
DISTRIBUTED BY (id)
PARTITION BY RANGE (date)
( PARTITION Jan16 START (date '2016-01-01') INCLUSIVE , 
  PARTITION Feb16 START (date '2016-02-01') INCLUSIVE ,
  PARTITION Mar16 START (date '2016-03-01') INCLUSIVE ,
  PARTITION Apr16 START (date '2016-04-01') INCLUSIVE ,
  PARTITION May16 START (date '2016-05-01') INCLUSIVE ,
  PARTITION Jun16 START (date '2016-06-01') INCLUSIVE ,
  PARTITION Jul16 START (date '2016-07-01') INCLUSIVE ,
  PARTITION Aug16 START (date '2016-08-01') INCLUSIVE ,
  PARTITION Sep16 START (date '2016-09-01') INCLUSIVE ,
  PARTITION Oct16 START (date '2016-10-01') INCLUSIVE ,
  PARTITION Nov16 START (date '2016-11-01') INCLUSIVE ,
  PARTITION Dec16 START (date '2016-12-01') INCLUSIVE 
                  END (date '2017-01-01') EXCLUSIVE );

```

按数字范围分区：表使用单个数字数据类型列作为分区键列。

```sql
-- 下面的建表语句会创建11个分区
CREATE TABLE rank (id int, rank int, year int, gender 
char(1), count int)
DISTRIBUTED BY (id)
PARTITION BY RANGE (year)
( START (2006) END (2016) EVERY (1), 
  DEFAULT PARTITION extra ); 
```

定义列表分区表：使用任意允许等值比较的数据类型列作为它的分区键列

```sql
-- 创建rank_1_prt_boys、rank_1_prt_girls、rank_1_prt_other三个分区
CREATE TABLE rank (id int, rank int, year int, gender 
char(1), count int ) 
DISTRIBUTED BY (id)
PARTITION BY LIST (gender)
( PARTITION girls VALUES ('F'), 
  PARTITION boys VALUES ('M'), 
  DEFAULT PARTITION other );
```

多级分区：

```sql
-- 基于时间和值的多级分区

CREATE TABLE sales (
    trans_id int, 
    date date, 
    amount decimal(9,2), 
    region text
) DISTRIBUTED BY (trans_id)
PARTITION BY RANGE (date) 
SUBPARTITION BY LIST (region)
SUBPARTITION TEMPLATE ( 
    SUBPARTITION usa VALUES ('usa'), 
    SUBPARTITION asia VALUES ('asia'), 
    SUBPARTITION europe VALUES ('europe'), 
    DEFAULT SUBPARTITION other_regions
)(
    START (date '2011-01-01') INCLUSIVE
    END (date '2012-01-01') EXCLUSIVE
    EVERY (INTERVAL '1 month'), 
    DEFAULT PARTITION outlying_dates 
);

-- 三级分区表

CREATE TABLE p3_sales (id int, year int, month int, day int, region text)
DISTRIBUTED BY (id)
PARTITION BY RANGE (year)
SUBPARTITION BY RANGE (month)
SUBPARTITION TEMPLATE (
    START (1)  END (13)  EVERY (1),  DEFAULT SUBPARTITION other_months 
)
SUBPARTITION BY LIST (region)
SUBPARTITION TEMPLATE (
    SUBPARTITION usa VALUES ('usa'),
    SUBPARTITION europe VALUES ('europe'),
    SUBPARTITION asia VALUES ('asia'),
    DEFAULT SUBPARTITION other_regions 
)( 
    START (2002) END (2012) EVERY (1), 
    DEFAULT PARTITION outlying_years 
);

```

# 参考

[参考文档](https://greenplum.cn/gp6/ddl/ddl-partition.html).