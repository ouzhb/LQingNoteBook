# 方案一

使用第三方插件持久化到Kafka，虽然官方提供的付费plugin可以持久化到MySQL、Mongo等数据库。但是github上能够找到的第三方插件，几乎都是通过Kafka持化的。


## 安装部署

部署 [IGIT-CN/emqx_kafka_bridge](https://github.com/IGIT-CN/emqx_kafka_bridge) 仓库提供的代码，该代码基于emqx 3.2 版本。

虽然 EMQ 4.x 和 3.x 应该是可以兼容插件版本的，但是4.x上安装该插件时出现的问题，所以选择在 3.2.7 版本下进行编译安装，安装过程包括以下几个步骤：

- 安装Erlang-OTP环境

```shell

# 下载 21.3 版本的安装包，下载地址 https://www.erlang.org/downloads/21.3

yum -y install make gcc gcc-c++ kernel-devel m4 ncurses-devel openssl-devel unixODBC-devel libtool libtool-ltdl-devel 
./otp_build autoconf  
./configure  
make  
make install 

# Erlang 默认安装路径为 /usr/local/lib/erlang，配置以下环境变量

export ERLANG_HOME=/usr/local/lib/erlang  
export PATH=$PATH:$ERLANG_HOME/bin 

```

- 安装rebar3 

```shell

# 只要从https://s3.amazonaws.com/rebar3/rebar3 下载然后添加到path目录即可（要建一个软连接名字为rebar3）
# 但是实际过程中，好像不用装这个东西

```

- 安装编译emqx

```shell

git clone https://github.com/emqx/emqx-rel.git emqx-rel
cd emqx-rel
git checkout -b v3.2.7 v3.2.7

# 修改rebar.config，在deps中添加下面一个依赖
{emqx_kafka_bridge,{git,"https://github.com/IGIT-CN/emqx_kafka_bridge",{tag, "v3.2.0"}}}

# 修改rebar.config，在relx中添加下面一个依赖
{emqx_kafka_bridge, load}

# 执行编译命令

make  # 编译可执行文件
make emqx-pkg # 编译rpm包
ls _packages/emqx

# 需要注意的emqx-rel工程是emqx的打包编译工程，在Makefile中会根据当前git命令的返回值从git上下载所有插件、broker的代码。
# 
# 可能出现的几个问题：
#	1. cowboy 依赖下载失败，可以在rebar.conf 中的deps添加 下面的依赖：
#       {cowboy,{git,"https://github.com/emqx/cowboy.git",{tag, "2.6.1"}}} 
#       {cowlib,{git,"https://github.com/ninenines/cowlib",{tag, "2.7.0"}}}
#       {ranch,{git,"https://github.com/ninenines/ranch",{tag, "1.7.1"}}}
#
#
#


```





