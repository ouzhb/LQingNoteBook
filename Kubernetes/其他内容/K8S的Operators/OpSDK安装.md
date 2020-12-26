# 安装Go-lang
解压安装包，配置以下环境变量。注意，GOPATH一般用来存放go语言项目的源码
``` sh
export GOROOT=/usr/local/go
export GOPATH=/usr/local/gopath
export PATH=${PATH}:${GOROOT}/bin:${GOPATH}/bin
```
# 安装dep工具

[dep](https://github.com/golang/dep) 工具是用来管理go语言依赖的! 通过以下命令安装：

``` sh
## 安装需要搭梯子！！
curl https://raw.githubusercontent.com/golang/dep/master/install.sh | sh
```

# 安装SDK

```sh
## 安装需要搭梯子！！
mkdir -p $GOPATH/src/github.com/operator-framework
cd $GOPATH/src/github.com/operator-framework
git clone https://github.com/operator-framework/operator-sdk
cd operator-sdk
git checkout master
make dep
make install
```

# 创建测试工程

```sh
# Go的包管理一般基于Git，因此需要配置一个默认的Git仓库
git config --global user.email "linqing@ruijie.com.cn"
git config --global user.name LinQing2017
# 在GOPATH目录下创建工程模板
mkdir -p $GOPATH/src/github.com/example-inc/
cd $GOPATH/src/github.com/example-inc/
# Operator 包含两个监视维度，namespace-scoped和cluster-scoped。默认情况是namespace维度，通过--cluster-scoped可以创建出cluster-scoped工程
operator-sdk new memcached-operator
cd memcached-operator
```
# 架构说明

| 目录 | 说明 |
| ------ | ------ | 
| cmd | 包含一个manager/main.go文件，该文件是Operator的入口，负责创建一个Operator Manager。Operator Manager负责定义pkg/apis/目录下的所有资源，同时启动pkg/controllers/目录下的所有控制器。PS：Kubernetes中Resources指Deployment、Service等，多个Resources构成一个App。针对Resources的操作，如删除、扩展等成为Controller。 | 
| pkg/apis | 包含用户的CRD，格式为pkg/apis/<group>/<version>/<kind>_types.go | 
|pkg/controller|包括用户的controller，格式为pkg/controller/<kind>/<kind>_controller.go|
|build|用来创建Operator的Dockerfile和相关脚本|
|deploy|部署Operator的yaml文件|
|Gopkg.toml|依赖描述|
|Gopkg.lock|依赖描述|
|vendor|依赖|

