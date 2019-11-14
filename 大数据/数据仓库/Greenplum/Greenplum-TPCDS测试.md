---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-23
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum 调研笔记

<!--more-->

# 测试工具

使用[pivotalguru/TPC-DS](https://github.com/pivotalguru/TPC-DS/blob/master/rollout.sh) 工具进行测试，该工具是一个包含：数据生成、执行SQL、输入报告等功能。

## 安装步骤

```shell

# 在master节点安装以下工具
yum -y install git gcc bc
# 切换到gpadmin用户，clone工具的执行脚本
git clone https://github.com/pivotalguru/TPC-DS

# 编辑gpadmin用户的.bashrc文件，指定Greenplum的环境变量

```

在gpadmin用户目录中，创建脚本测试脚本tpcds.sh，并执行即可以开始测试：

```shell
#!/bin/bash
set -e
REPO="TPC-DS"                         
ADMIN_USER="gpadmin"
INSTALL_DIR="/home/gpadmin/pivotalguru"  # 存放TPC-DS工程的目录
EXPLAIN_ANALYZE="false"
RANDOM_DISTRIBUTION="false"              
MULTI_USER_COUNT="0"                    
GEN_DATA_SCALE="10"                      # 1000 约等于生成1T原始数据
SINGLE_USER_ITERATIONS="1"
RUN_COMPILE_TPCDS="false"
RUN_GEN_DATA="false"
RUN_INIT="true"
RUN_DDL="true"
RUN_LOAD="true"
RUN_SQL="true"
RUN_SINGLE_USER_REPORT="true"
RUN_MULTI_USER="true"
RUN_MULTI_USER_REPORT="true"
RUN_SCORE="true"

su --session-command="cd \"$INSTALL_DIR/$REPO\"; ./rollout.sh $GEN_DATA_SCALE $EXPLAIN_ANALYZE $RANDOM_DISTRIBUTION $MULTI_USER_COUNT $RUN_COMPILE_TPCDS $RUN_GEN_DATA $RUN_INIT $RUN_DDL $RUN_LOAD $RUN_SQL $RUN_SINGLE_USER_REPORT $RUN_MULTI_USER $RUN_MULTI_USER_REPORT $RUN_SCORE $SINGLE_USER_ITERATIONS" $ADMIN_USER

```

## pivotalguru/TPC-DS

测试工具的入口是rollout.sh，该脚本顺序调用每个子目录下的rollout.sh

```shell
## - 00_compile_tpcds: tpcds源码脚本会编译该目录的C语言源码，用于生成原始数据
##
## - 01_gen_data：数据生成脚本、SQL生成脚本
##      - 工具会在每一个Segment实例安装目录（xxx/pivotalguru）中并行生成原始数据，原始数据的格式为" XXX_{id}_{num_seg}.dat "
##      - 由于SQL脚本中需要处理表的分区信息，因此每次生成数据都会生成相应SQL脚本，生成的SQL被保存到05_sql目录中（sql的模板时TPC-DS本身提供的，位于00_compile_tpcds\query_templates）
##
## - 02_init：测试开始前的一些准备工作，包括生成Seg信息、保存配置等
## 
## - 03_ddl：创建表
##      - 工具会创建schema：tpcds、ext_tpcds
##      - 默认根据03_ddl\distribution.txt文件进行分片，也可以指定RANDOM_DISTRIBUTION
##
## - 04_load：将外部表导入到Greenplum中
##      - 工具会为每个SegmentHost启动gpfdist服务，并执行SQL直接将外部表导入
## 
## - 05_sql：执行单用户测试
##
## - 06_single_user_reports：生成单用户测试报告
##      - 测试信息在schemaname是tpcds_reports的表中
##
## - 07_multi_user：执行多用户测试
##
## - 08_multi_user_reports：生成多用户测试报告
## 
## - 09_score：生成测试评分
##
## - functions.sh：环境准备脚本，由rollout.sh调用
##
## - rollout.sh：测试入口脚本
```


# 参考