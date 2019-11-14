---
title: 数据库调研笔记 -- ClickHouse之SQL语法
categories: "笔记"
date: 2019-06-19
comments: true
toc: true
tags:
	- 数据库
	- ClickHouse
---


数据库调研笔记 -- ClickHouse


<!--more-->

# SQL语法

下面的内容简单说明Clickhouse中SQL和标准SQL的差别，以及他的特点：

## SELECT

如果查询中不包含DISTINCT，GROUP BY，ORDER BY子句以及IN和JOIN子查询，那么它将仅使用O(1)数量的内存来完全流式的处理查询。对于其他高内存消耗的查询，可以使用参数控制内存，并且通过磁盘进行外部排序。

### From语句
From 之后可以是表、子查询、表函数、ARRAY JOIN子句、JOIN子句。**子查询无需指定别名，即使指定了也会被忽略**。

### SAMPLE语句
MergeTree表中可以使用SAMPLE来进行采样，并且系统在不同的时间，不同的服务器，不同表上总以相同的方式对数据进行采样。
```SQL
SAMPLE k  # 0<k<1 表示采样百分比，k>=1 表示采样条数
#例子
SELECT
    Title,
    count() * 10 AS PageViews
FROM hits_distributed
SAMPLE 0.1
WHERE
    CounterID = 34
    AND toDate(EventDate) >= toDate('2013-01-29')
    AND toDate(EventDate) <= toDate('2013-02-04')
    AND NOT DontCountHits
    AND NOT Refresh
    AND Title != ''
GROUP BY Title
ORDER BY PageViews DESC LIMIT 1000

```

### ARRAY JOIN 子句

ARRAY JOIN可以进行与数组（nested数据类型）的连接。

假设存在下面的表：
```sql
 CREATE TABLE arrays_test (s String, arr Array(UInt8)) ENGINE = Memory
```

|s|arr|
|---|---|
|Hello|[1,2]|
|World|[3,4,5]|
|Goodbye|[]|

使用Array Join将arr列展开

```sql
SELECT s, arr, a FROM arrays_test ARRAY JOIN arr AS a
```

|s|arr|1|
|---|---|---|
|Hello|[1,2]|1|
|Hello|[1,2]|2|
|World|[3,4,5]|3|
|World|[3,4,5]|4|
|World|[3,4,5]|5|

Array Join可以使用在nested数据结构上：

```sql
CREATE TABLE nested_test (s String, nest Nested(x UInt8, y UInt32)) ENGINE = Memory

INSERT INTO nested_test VALUES ('Hello', [1,2], [10,20]), ('World', [3,4,5], [30,40,50]), ('Goodbye', [], [])

SELECT s, nest.x, nest.y FROM nested_test ARRAY JOIN nest

SELECT s, nest.x, nest.y FROM nested_test ARRAY JOIN nest.x
```

Array Join的其他特点：

- 当多个具有相同大小的数组使用逗号分割出现在ARRAY JOIN子句中时，ARRAY JOIN会将它们同时执行（直接合并，而不是它们的笛卡尔积）。假设有2个数组，数组长度为2，ARRAY JOIN的结果依然是2行。
- 在一个查询中只能出现一个ARRAY JOIN子句。
- 如果在WHERE/PREWHERE子句中使用了ARRAY JOIN子句的结果，它将优先于WHERE/PREWHERE子句执行，否则它将在WHERE/PRWHERE子句之后执行，以便减少计算。

### Join

```SQL
SELECT <expr_list> FROM <left_subquery>
[GLOBAL] [ANY|ALL] INNER|LEFT|RIGHT|FULL|CROSS [OUTER] JOIN <right_subquery>
(ON <expr_list>)|(USING <column_list>) ...
```

**ANY 和 ALL 的区别**：
ANY表示存在多个与左表关联的数据，那么系统仅返回第一个与左表匹配的结果，即结果和左边行数一致。ALL表示返回全部结果，这和标准SQL一致。默认情况下是ALL。

**GLOBAL ... JOIN**：
当使用普通的JOIN时，查询将被发送给远程的服务器。并在这些远程服务器上生成右表并与它们关联。换句话说，右表来自于各个服务器本身。

当使用GLOBAL ... JOIN，首先会在请求服务器上计算右表并以临时表的方式将其发送到所有服务器。这时每台服务器将直接使用它进行计算。

**Join的其他限制**：

- 右表（子查询的结果）将会保存在内存中。如果没有足够的内存，则无法运行JOIN。
- 只能在查询中指定一个JOIN。若要运行多个JOIN，你可以将它们放入子查询中。
- 每次运行相同的JOIN查询，总是会再次计算 - 没有缓存结果。
- 在各种类型的JOIN中，最高效的是ANY LEFT JOIN，然后是ANY INNER JOIN，效率最差的是ALL LEFT JOIN以及ALL INNER JOIN。
- 推荐你使用子查询的方式执行JOIN。
- 推荐你使用子查询的方式执行JOIN。
```SQL
SELECT
    CounterID,
    hits,
    visits
FROM
(
    SELECT
        CounterID,
        count() AS hits
    FROM test.hits
    GROUP BY CounterID
) ANY LEFT JOIN
(
    SELECT
        CounterID,
        sum(Sign) AS visits
    FROM test.visits
    GROUP BY CounterID
) USING CounterID
ORDER BY hits DESC
LIMIT 10


# USING子句用于指定要进行链接的一个或多个列，系统会将这些列在两张表中相等的值连接起来。如果列是一个列表，不需要使用括号包裹。同时JOIN不支持其他更复杂的Join方式。
```

### WHERE

- PREWHERE语句与WHERE子句的意思相同。主要的不同之处在于表数据的读取。 当使用PREWHERE时，首先只读取PREWHERE表达式中需要的列。然后在根据PREWHERE执行的结果读取其他需要的。

- 在一个查询中可以同时指定PREWHERE和WHERE，在这种情况下，PREWHERE优先于WHERE执行。

- 如果将'optimize_move_to_prewhere'设置为1，并且在查询中不包含PREWHERE，则系统将自动的把适合PREWHERE表达式的部分从WHERE中抽离到PREWHERE中。

### GROUP BY

- MySQL不同的是（实际上这是符合SQL标准的），SELECT、HAVING、ORDER BY不能够获得一个不在GROUP BY中的非聚合函数列（除了常量表达式）。但是可以使用‘any’（返回遇到的第一个值）、max、min等聚合函数使它工作。

- 如果查询表达式列表中仅包含聚合函数，则可以省略GROUP BY子句，这时会假定将所有数据聚合成一组空“key”。

```SQL
SELECT
    count(),
    median(FetchTiming > 60 ? 60 : FetchTiming),
    count() - sum(Refresh)
FROM hits
```
- 对于GROUP BY子句，ClickHouse将 NULL 解释为一个值，并且支持NULL=NULL。

- GROUP BY中允许将临时数据转存到磁盘上，以限制对内存的使用。 max_bytes_before_external_group_by这个配置确定了在GROUP BY中启动将临时数据转存到磁盘上的内存阈值。如果你将它设置为0（这是默认值），这项功能将被禁用。如果有临时数据被刷到了磁盘中，那么这个查询的运行时间将会被延长几倍（大约是3倍）。

- 在分布式查询处理中，外部聚合将会在远程的服务器中执行。为了使请求服务器只使用较少的内存，可以设置distributed_aggregation_memory_efficient为1。

### LIMIT N BY 子句

LIMIT N BY COLUMNS 子句可以用来在每一个COLUMNS分组中求得最大的N行数据。

```sql
SELECT
    domainWithoutWWW(URL) AS domain,
    domainWithoutWWW(REFERRER_URL) AS referrer,
    device_type,
    count() cnt
FROM hits
GROUP BY domain, referrer, device_type
ORDER BY cnt DESC
LIMIT 5 BY domain, device_type
LIMIT 100

# 查询将会为每个domain, device_type的组合选出前5个访问最多的数据，但是结果最多将不超过100行（LIMIT n BY + LIMIT）。
```

### HAVING 子句

- HAVING子句可以用来过滤GROUP BY之后的数据，类似于WHERE子句。 同样，HAVING子句中的条件必须出现在Group by中。
- WHERE于HAVING不同之处在于WHERE在聚合前(GROUP BY)执行，HAVING在聚合后执行。 
- 如果不存在聚合，则不能使用HAVING。

### ORDER BY 子句
- 表达式列表中每一个表达式都可以分配一个DESC或ASC（排序的方向）。如果没有指明排序的方向，将假定以ASC的方式进行排序。
- 对于字符串的排序来讲，你可以为其指定一个排序规则，在指定排序规则时，排序总是不会区分大小写。
```sql
#例如：
ORDER BY SearchPhrase COLLATE 'tr' 
# 使用土耳其字母表对它进行升序排序，同时排序时不会区分大小写，并按照UTF-8字符集进行编码。
```
- 表达式中相同值的行将以任意的顺序进行输出
- NaN 和 NULL 的排序规则：
    - 当使用NULLS FIRST修饰符时，将会先输出NULL，然后是NaN，最后才是其他值。
    - 当使用NULLS LAST修饰符时，将会先输出其他值，然后是NaN，最后才是NULL。
    - 默认情况下与使用NULLS LAST修饰符相同。
    - 当使用浮点类型的数值进行排序时，不管排序的顺序如何，NaNs总是出现在所有值的后面。(升序时NaNs最大，降序时最小)
```SQL
SELECT * FROM t_null_nan ORDER BY y NULLS FIRST
```
- 如果你在ORDER BY子句后面存在LIMIT并给定了较小的数值，则将会使用较少的内存。否则，内存的使用量将与需要排序的数据成正比。
- 对于分布式查询，如果省略了GROUP BY，则在远程服务器上执行部分排序，最后在请求服务器上合并排序结果。这意味这对于分布式查询而言，要排序的数据量可以大于单台服务器的内存。
- 如果没有足够的内存，可以使用外部排序（在磁盘中创建一些临时文件）。可以使用max_bytes_before_external_sort来设置外部排序

### DISTINCT 子句

- 如果存在DISTINCT子句，则会对结果中的完全相同的行进行去重。
- 当不存在ORDER BY子句并存在LIMIT子句时，查询将在同时满足DISTINCT与LIMIT的情况下立即停止查询。
- 可以与GROUP BY配合使用。
- 在处理数据的同时输出结果，并不是等待整个查询全部完成。
- 在SELECT表达式中存在Array类型的列时，不能使用DISTINCT。
- DISTINCT可以与 NULL一起工作，就好像NULL仅是一个特殊的值一样，并且NULL=NULL。

### LIMIT

- LIMIT N：选取前n行
- LIMIT N,M：选取从低n行开始的m行数据

### UNION ALL 子句

- UNION ALL子句可以组合任意数量的查询

```sql
SELECT CounterID, 1 AS table, toInt64(count()) AS c
    FROM test.hits
    GROUP BY CounterID

UNION ALL

SELECT CounterID, 2 AS table, sum(Sign) AS c
    FROM test.visits
    GROUP BY CounterID
    HAVING c > 0
```
- UNION ALL中的查询可以同时运行，它们的结果将被混合到一起。
- 查询的结果结果必须相同（列的数量和类型）。
- 如果两个查询中有相同的列，但是类型不同但是兼容，CK会自动进行转化。
- 列名可以是不同的。在这种情况下，最终结果的列名将从第一个查询中获取。

### INTO OUTFILE 子句

- INTO OUTFILE filename 子句用于将查询结果重定向输出到指定文件中
- 执行的结果文件将在客户端建立，如果文件已存在，查询将会失败。 

### IN 运算符
- 运算符的左侧是单列或列的元组。
```sql
SELECT UserID IN (123, 456) FROM ...
SELECT (CounterID, UserID) IN ((34, 123), (101500, 456)) FROM ...
```
- 左侧是单个列并且是一个索引，并且右侧是一组常量时，系统将使用索引来处理查询。
- 右侧可以是一个表的名字，或者是一个子查询

```sql
UserID IN users
UserID IN (SELECT * FROM users)
SELECT (CounterID, UserID) IN (SELECT CounterID, UserID FROM ...) FROM ...
```
- 如果操作符的右侧是一个Set引擎的表时（数据总是在内存中准备好），则不会每次都为查询创建新的数据集。

- IN操作符的左右两侧应具有相同的类型。
- IN操作符的子查询中可以出现任意子句，包含聚合函数与lambda函数。
```sql
SELECT
    EventDate,
    avg(UserID IN
    (
        SELECT UserID
        FROM test.hits
        WHERE EventDate = toDate('2014-03-17')
    )) AS ratio
FROM test.hits
GROUP BY EventDate
ORDER BY EventDate ASC
```
- IN操作符总是假定 NULL 值的操作结果总是等于0

在分布式表中使用IN或者Join:


    1. 当使用普通的IN时，查询总是被发送到远程的服务器，并且在每个服务器中运行“IN”或“JOIN”子句中的子查询。
    2. 当使用GLOBAL IN / GLOBAL JOIN时，首先会为GLOBAL IN / GLOBAL JOIN运行所有子查询，并将结果收集到临时表中，并将临时表发送到每个远程服务器，并使用该临时表运行查询。
    
    因此，对于非分布式查询，请使用普通的IN ／ JOIN。在分布式查询中使用IN / JOIN子句中使用子查询需要根据实际情况。

```sql
SELECT uniq(UserID) FROM distributed_table 
WHERE 
    CounterID = 101500 
AND 
    UserID IN (
        SELECT UserID FROM local_table WHERE CounterID = 34
    )

# IN子句中的数据集将被在每台服务器上被独立的收集，仅与每台服务器上的本地存储上的数据计算交集。

# 如果单个UserID的数据完全分布在单个服务器上，那么这将是正确且最佳的查询方式。

# 当UserID的数据分布在不同的服务器上时：

SELECT uniq(UserID) FROM distributed_table 
WHERE 
    CounterID = 101500 
AND 
    UserID IN (
        SELECT UserID FROM distributed_table WHERE CounterID = 34
    )

# 子查询将在每个远程服务器上执行。
# 因为子查询使用分布式表，所有每个远程服务器上的子查询将查询再次发送给所有的远程服务器

# 例如，如果你拥有100台服务器的集群，执行整个查询将需要10，000次请求，这通常被认为是不可接受的。因此这里要使用GLOBAL IN来替代IN。

SELECT uniq(UserID) FROM distributed_table 
WHERE 
    CounterID = 101500 
AND 
    UserID GLOBAL IN (
        SELECT UserID FROM distributed_table WHERE CounterID = 34
    )

```

## INSERT INTO

### INSERT

```SQL
INSERT INTO [db.]table [(c1, c2, c3)] VALUES (v11, v12, v13), (v21, v22, v23), ...
```

- 可以在查询中指定插入的列的列表，如：[(c1, c2, c3)]。对于存在于表结构中但不存在于插入列表中的列，它们将会按照如下方式填充数据：
    - 如果存在DEFAULT表达式，根据DEFAULT表达式计算被填充的值。
    - 如果没有定义DEFAULT表达式，则填充零或空字符串。

- 数据可以以ClickHouse支持的任何[输入输出](https://clickhouse.yandex/docs/zh/interfaces/formats/#formats)格式 传递给INSERT。

- ClickHouse会清除数据前所有的空白字符与一行摘要信息

### 使用SELECT的结果写入

- 写入与SELECT的列的对应关系是使用位置来进行对应的,尽管它们在SELECT表达式与INSERT中的名称可能是不同的。

- 在进行INSERT时将会对写入的数据进行一些处理，按照主键排序，按照月份对数据进行分区等。所以如果在您的写入数据中包含多个月份的混合数据时：
    - 数据总是以尽量大的batch进行写入，如每次写入100,000行。
    - 数据在写入ClickHouse前预先的对数据进行分组。

## CREATE

### 创建数据库

```SQL
CREATE DATABASE [IF NOT EXISTS] db_name
```

### 建表

```sql

# 可以用三种方式指定默认值
# 
#   DEFAULT expr： 普通默认值，如果INSERT中不包含指定的列，那么将通过表达式计算它的默认值并填充它。
#   MATERIALIZED expr ： 物化表达式，被该表达式指定的列不能包含在INSERT的列表中，因为它总是被计算出来的。
#   ALIAS expr ： 这样的列不会存储在表中。 它的值不能够通过INSERT写入，同时使用SELECT查询星号时，这些列也不会被用来替换星号。 但是它们可以显示的用于SELECT中，在这种情况下，在查询分析中别名将被替换。
#
CREATE TABLE [IF NOT EXISTS] [db.]table_name [ON CLUSTER cluster]
(
    name1 [type1] [DEFAULT|MATERIALIZED|ALIAS expr1],
    name2 [type2] [DEFAULT|MATERIALIZED|ALIAS expr2],
    ...
) ENGINE = engine

# 创建一张和表2结构相同的表，
CREATE TABLE [IF NOT EXISTS] [db.]table_name AS [db2.]name2 [ENGINE = engine]

# 创建一张和SELEC结构相同的表，并且用SELECT填充
CREATE TABLE [IF NOT EXISTS] [db.]table_name ENGINE = engine AS SELECT ...

```

### 临时表

ClickHouse支持临时表，其具有以下特征：

- 当回话结束时，临时表将随会话一起消失，这包含链接中断。
- 临时表仅能够使用Memory表引擎。
- 无法为临时表指定数据库。它是在数据库之外创建的。
- 如果临时表与另一个表名称相同，那么当在查询时没有显示的指定db的情况下，将优先使用临时- 表。
- 对于分布式处理，查询中使用的临时表将被传递到远程服务器。

### CREATE VIEW

支持普通视图和物化视图：

```sql
CREATE [MATERIALIZED] VIEW [IF NOT EXISTS] [db.]table_name [TO[db.]name] [ENGINE = engine] [POPULATE] AS SELECT ...
```

- 普通视图不存储任何数据，只是执行从另一个表中的读取。换句话说，普通视图只是保存了视图的查询，当从视图中查询时，此查询被作为子查询用于替换FROM子句。
- 物化视图存储的数据是由相应的SELECT查询转换得来的。在创建物化视图时，你还必须指定表的引擎 - 将会使用这个表引擎存储数据。

- 物化视图只会包含在物化视图创建后的新写入的数据。(不指定POPULATE的情况)

- 目前对物化视图执行ALTER是不支持的，因此这可能是不方便的。

# 参考

[官方文档](https://clickhouse.yandex/docs/zh/query_language)