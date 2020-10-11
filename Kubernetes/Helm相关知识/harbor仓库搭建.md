---
title: Kubernetes学习笔记 -- 安装Harbor仓库
categories: "笔记"
date: 2019-07-16
comments: true
toc: true
tags:
	- Kubernetes
    - Docker
    - Harbor
---

Kubernetes学习笔记 -- 安装Harbor仓库

<!--more-->

# 1.Docker安装部署

- 下载离线安装包 harbor-offline-installer-v1.10.2.tar.gz 解压
- 编辑 harbor.yml ，注意以下配置：

	- hostname：主机名（填：harbor）
	- http.port: http端口（填：18000）
	- https.port: https端口（填：18443）
	- https.certificate: CA签发到服务端的证书
	- https.private_key: 服务端私钥
	- data_volume: 数据目录，映射到主机上
- 执行prepare，生成配置
- docker-compose启动

# 2. Helm包安装

Helm包安装时以下配置需要注意：

- expose.type：当前Harbor支持ingress、nodePort、clusterIP、loadBalancer
- expose.tls.enabled：是否启用HTTPS
- expose.tls.secret.secretName：自定义域名密文
- expose.ingress.hosts.core：ingress生成的域名
- externalURL：该配置决定了Harbor的地址，配置
- jobservice.jobLogger：日志服务输出配置，集群部署的时候请设置成database
- persistence.imageChartStorage.disableredirect：使用外部存储时client是否直连

# 3.创建SSL证书

注意创建时，harbor节点主机名为：harbor

配置/etc/hosts 中全域名为：<ipaddress> harbor harbor.goblin.com

```shell

# 创建根证书

openssl genrsa -out ca.key 4096

openssl req -x509 -new -nodes -sha512 -days 3650 \
 -subj "/C=CN/ST=Fujian/L=FuZhou/O=LQ/OU=Goblin/CN=harbor.goblin.com" \
 -key ca.key \
 -out ca.crt
 
# 生成服务端证书
 
## 1. 服务端私钥
openssl genrsa -out harbor.goblin.com.key 4096
 
## 2. 服务端的签发请求
openssl req -sha512 -new \
    -subj "/C=CN/ST=Fujian/L=FuZhou/O=LQ/OU=Goblin/CN=harbor.goblin.com" \
    -key harbor.goblin.com.key \
    -out harbor.goblin.com.csr
 
## 3. 生成请求配置
cat > v3.ext <<-EOF
authorityKeyIdentifier=keyid,issuer
basicConstraints=CA:FALSE
keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @alt_names

[alt_names]
DNS.1=harbor.goblin.com
DNS.2=harbor.goblin
DNS.3=harbor
EOF
 
## 4. 生成服务端证书
openssl x509 -req -sha512 -days 3650 \
    -extfile v3.ext \
    -CA ca.crt -CAkey ca.key -CAcreateserial \
    -in harbor.goblin.com.csr \
    -out harbor.goblin.com.crt

# 配置Docker客户端证书  
  
## 1. 证书转换cert格式（Docker使用这个格式的证书）
openssl x509 -inform PEM -in harbor.goblin.com.crt -out harbor.goblin.com.cert
 
## 2. 配置docker端证书
mkdir -p  /etc/docker/certs.d/harbor.goblin.com:<https_port>
cp harbor.goblin.com.cert /etc/docker/certs.d/harbor.goblin.com:<https_port>
cp harbor.goblin.com.key /etc/docker/certs.d/harbor.goblin.com:<https_port>
cp ca.crt /etc/docker/certs.d/harbor.goblin.com:<https_port>

## 3. 修改/etc/hosts,添加：<ipaddress> harbor harbor.goblin.com

## 4. 登录验证
docker login lqdocker.goblin.com:18443 -u admin

```

# 4. FAQ

## 1. 关于Harbor集群

Harbor核心服务包括：core、jobservice、registry 这三个服务可以组成集群，并且每个core实例可以填写不同的**域名**以及**协议**。

可以通过**部署不同helm实例，并共享外部PG、Redis**的方式实现网络分离、多域名绑定等功能，部署时helm配置需要注意以下几点：

- jobservice.jobLogger：日志服务输出配置，集群部署的时候请设置成database

- 不同helm实例中，组件 secret 要配置成一致，否则集群内部通信会出现问题（jobservice、core、registry三个组件）

## 2. Harbor版本的相关问题

- 测试过程中发现1.9.x 版本之间**Login API**之间有一些差异，1.9.0版本没有进行csrf校验，而1.9.2、1.9.4版本进行了校验。但是1.9.x 版本校验方式又和2.0.x不相同
- 1.9.0 版本组集群时有同步问题
- 1.9.x版本的csrf校验可以通过配置文件关闭（1.9.x使用的是 BEEG框架，[参考](https://beego.me/docs/mvc/controller/xsrf.md)），相关配置项为**EnableXSRF=false**。
- 2.0.x没有找到关闭XSRF校验的配置
- 关于配置LDAP认证：手工创建了任意一个用户后，无法从DB认证切换到LDAP认证（即使删除掉DB用户也不行？？相信这是一个bug）。恢复方式删除用户，并且删除数据库中对应的表记录


# 参考

[安装包地址](https://github.com/goharbor/harbor)

[harbor-helm](https://github.com/goharbor/harbor-helm)

[配置SSL](https://goharbor.io/docs/1.10/install-config/configure-https/)