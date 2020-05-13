# SQL 语法

## 数据类型

### 1. 基本类型包括：

- boolean
- char：1 bytes
- 数值类型：
    - smallint：2 bytes
    - integer：4 bytes
    - bigint：8 bytes
    - real：4 bytes，保留6位精度
    - double precision：8 bytes，保留15位精度
- text：支持多个unicode字符
- ip：可以保存IPv4/IPv6地址，用户在插入的时候，可以使用字符串的方式表示ip值
- timestamp with time zone
- timestamp without time zone：UTC 时间
- interval

### 2. 地理数据类型

- geo_point：保存经纬度数据
- geo_shape：保存几何形状数据（这些数据使用GeoJSON geometry 格式来定义）

### 3. 结构化数据

- object：可以包含任意其他类型的成员（一个object对象下，包含多个对象列）

```sql
create table my_table11 (
  title text,
  col1 object,
  col3 object(strict) as (  -- strict 表示拒绝存储其他没有的对象列，其他选择包括：dynamic（默认，插入新的列会更新schema并进行类型映射）, ignored（允许插入新列，但是不会更新schema以及进行类型映射） 
    age integer,
    name text,
    col31 object as (
      birthday timestamp with time zone
    )
  )
);

```

- array

```sql

SELECT '{ab, CD, "CD", null, "null"}'::array(text) AS arr;

```

### 4. 类型别名

|Alias|	Crate Type|
|---|---|
|int2	|smallint|
|short	|smallint|
|int	|integer|
|int4	|integer|
|int8	|bigint|
|long	|bigint|
|string	|text|
|name	|text|
|regproc	|text|
|byte	|char|
|float	|real|
|double	|double precision|
|timestamp	|timestamp with time zone|
|timestamptz	|timestamp with time zone|

## 建表语句

```sql
CREATE TABLE [ IF NOT EXISTS ] table_ident ( [
    {
        base_column_definition
      | generated_column_definition
      | table_constraint
    }
    [, ... ] ]
)
[ PARTITIONED BY (column_name [, ...] ) ]
[ CLUSTERED [ BY (routing_column) ] INTO num_shards SHARDS ]
[ WITH ( table_parameter [= value] [, ... ] ) ]
```

Cratedb表的基本特征：

- base columns：是表中被存储的列，这些列是可读并且可写的，具有名称、类型、默认值、约束等属性。
- generated columns：生成列也被持久化，但是插入/更新时根据表达式计算出值。
- table_constraints：可选的约束子句。Cratedb中包括：表约束和列约束两种，这两种约束可以相互转换。




