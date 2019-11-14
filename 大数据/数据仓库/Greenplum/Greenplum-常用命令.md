# PSQL 常用命令

- \q 退出
- \timing 打印SQL执行时间
- \l 打印当前所有数据库
- \c 切换数据库
- \dt+ 参看所有表
- \di+ 参看所有表索引
- \d tablename 查看表结构

# 常用工具

## 1.导出表结构


```sql
# -s 表示只dump表结构，-t表名
pg_dump -s -t pgbench_accounts pgbench

# 以下是输出
--
-- Name: pgbench_accounts; Type: TABLE; Schema: public; Owner: benchtest; Tablespace: 
--

CREATE TABLE public.pgbench_accounts (
    aid integer NOT NULL,
    bid integer,
    abalance integer,
    filler character(84)
)
WITH (fillfactor='100') DISTRIBUTED BY (aid);


ALTER TABLE public.pgbench_accounts OWNER TO benchtest;

--
-- Name: pgbench_accounts_pkey; Type: CONSTRAINT; Schema: public; Owner: benchtest; Tablespace: 
--

ALTER TABLE ONLY public.pgbench_accounts
    ADD CONSTRAINT pgbench_accounts_pkey PRIMARY KEY (aid);


--
-- Greenplum Database database dump complete
--

```

# 常用SQL

```sql

-- 修改表owner
ALTER TABLE public.tablename OWNER TO new_owner;

-- 为用户赋表的所有权限
GRANT ALL PRIVILEGES ON user TO tablename;

-- 为用户赋外表权限
alter user <user-name> CREATEEXTTABLE;

-- 创建用户
CREATE ROLE ruijie LOGIN REPLICATION CREATEDB CREATEEXTTABLE PASSWORD 'ruijie';
CREATE ROLE idata LOGIN REPLICATION CREATEDB CREATEEXTTABLE PASSWORD 'idata';

-- 修改并发配置
ALTER RESOURCE QUEUE pg_default WITH (ACTIVE_STATEMENTS=80);
```

```sql

-- 格式化显示
select pg_size_pretty(pg_database_size( 'MyDatabase' ));

-- 查询数据库大小
select pg_database_size( 'MyDatabase' );

-- 查询普通表大小

select pg_relation_size('tpcds.customer_demographics');

-- 查询分区表大小
CREATE OR REPLACE FUNCTION calc_partition_table(v_schemaname character varying, v_tablename character varying)
  RETURNS bigint AS
$BODY$
DECLARE
    v_calc BIGINT := 0;
    v_total BIGINT := 0;
    v_tbname VARCHAR(200);
    cur_tbname cursor for select schemaname||'.'||partitiontablename as tb from pg_partitions
   where schemaname=v_schemaname and tablename=v_tablename;
BEGIN
    OPEN cur_tbname;
    loop
        FETCH cur_tbname into v_tbname;
        if not found THEN
            exit;
        end if;
        EXECUTE 'select pg_relation_size('''||v_tbname||''')' into v_calc;
        v_total:=v_total+v_calc;        
    end loop;
    CLOSE cur_tbname;
    RETURN v_total;
end;
$BODY$
LANGUAGE plpgsql VOLATILE;

ALTER FUNCTION calc_partition_table(character varying, character varying) OWNER TO gpadmin;

-- 调用
SELECT calc_partition_table('v_schemaname','v_tablename');
```

```sql
-- 查看分区表信息

select * from pg_partitions where tablename='catalog_returns';

```

```sql

-- 查看表分布情况

select gp_segment_id,count(1) from {tablename} group by 1 order by 1; -- 这里1表示第一个列属性

select * from get_ao_distribution("tablename") order by 1; -- 针对appendonly表

-- 通过all_seg_sql查询所有节点的sql运行情况
-- 由于重分布之后Greenplum不会考虑数据是否均衡，因此可以通过使用下面的函数判断每个节点sql的运行情况
select * from all_seg_sql where sess_id = xxxx ;


-- 查看所有表的分布键

select 
    pg_attribute.attname,
    gp_distribution_policy.localoid::regclass 
from
    gp_distribution_policy, 
	(select generate_series(1,10)) i(i),
	pg_attribute 
where 
    gp_distribution_policy.distkey[i.i] is not null and
    gp_distribution_policy.localoid = pg_attribute.attrelid and 
    gp_distribution_policy.distkey[i.i] = pg_attribute.attnum 
order by i.i;

```

# 安装命令

```shell
gpinitsystem -c gpinitsystem_config  -h hostfile_exkeys
gpinitstandby -s node12
```


# 常见问题

## 关于python版本

gpadmin用户中由于Greenplum引用的python不是系统python，因此缺失很多模块导致yum等工具不能使用。

# 参考文档


[相关工具](https://gpdb.docs.pivotal.io/6-0/utility_guide/)