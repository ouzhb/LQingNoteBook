# EMQ公司

[EMQ](https://www.emqx.io/cn/mqtt/mqttx)公司是国内一家 IoT 方案提供商，该公司围绕他们的 MQTT Broker 提供包括平台、测试等一系列IoT解决方案。

目前EMQ公司包括以下几个版本的 MQTT 软件（[参考](https://www.emqx.io/static/files/EMQ_X_product_compare_cn.pdf)）：

- EMQ X Broker，开源的 MQTT Broker 服务
    - 官方宣称并发连接规模是**十万级**
    - 兼容 MQTT 5 以及 MQTT 3.x，以及 Qos、Retianed 等大部分常用的需求
    - 提供包括WebSocket在内多种连接方式
    - 能够集群但是不能扩展
    - 认证功能

- EMQ X Enterprise，企业级消息 MQTT Broker 服务，相比开源的版本，主要区别有以下几个：
    - 官方宣称并发连接规模是**百万级**
    - **能够进行集群扩展，而开源版本无法扩展**
    - 规则引擎的功能相比开源版本更强
    - **支持多种数据库的消息存储，开源版本完全没有这个功能**
    - 数据桥接的插件更多，比如**能够直接桥接数据到Kafka**，而开源版本不能
    - 提供运维服务、定制开发、等等

- EMQ X Platform，规模更大的IoT平台，消息通信部分的功能和EMQ X Enterprise一致

- EMQ X Edge 是轻量级多协议物联网边缘消息中间件，支持部署在资源受限的物联网边缘硬件。

- EMQ X Kuiper 基于 SQL 的轻量级边缘流式消息处理引擎，可以运行在资源受限的边缘设备上，用于实时处理来自于物联网设备的消息


# EMQ X Broker

EMQ X Broker是基于 **Erlang/OTP** 语言平台开发，支持大规模连接和分布式集群，发布订阅模式的开源 MQTT 消息服务器。

## 1. 安装部署


### 单机部署

从yum源安装

```shell
yum install -y yum-utils device-mapper-persistent-data lvm2
yum-config-manager --add-repo https://repos.emqx.io/emqx-ce/redhat/centos/7/emqx-ce.repo
# search 特定版本
yum list emqx --showduplicates | sort -r
yum install emqx
# 运行
systemctl start emqx

# 安装完成后，配置文件以及数据目录
# -----------------------------
# 配置文件路径：/etc/emqx
# 日志文件路径：/var/log/emqx
# 数据文件路径：/var/lib/emqx
#
# 安装完成后，服务的端口信息
# -----------------------------
# 1883	MQTT 协议端口
# 8883	MQTT/SSL 端口
# 8083	MQTT/WebSocket 端口
# 8081	HTTP API 端口
# 18083	Dashboard 管理控制台端口 admin / public

# PS：也可以容器化部署

docker run -d --name emqx -p 1883:1883 -p 8083:8083 -p 8883:8883 -p 8084:8084 -p 18083:18083 emqx/emqx

```

### Cluster部署

EMQX基于Erlang/OTP，该语言是一款历史悠久的动态语言，最初是爱立信为开发电信设备系统设计的编程语言平台。

Erlang 本身长处在于大规模的并发并发编程、并且还具备一个精炼的分布式模型，得益于这些语言上的优势 EMQX 相比其他 Java 技术栈的开源 MQTT Broker 或许能有更好的性能。

分布式集群中，client和主题的订阅关系可以为：client|cluster中的节点|订阅主题，这些订阅信息会会被解析成： **消息树**+**路由表**（相比单机环境多了一个路由表）。集群中所有节点维护相同的路由表，但是消息树各不相同（因为消息树的生成和Client订阅有关，client 连接集群中不同的节点，所以消息树各不相同）

![](https://docs.emqx.io/broker/latest/cn/_images/cluster_2.png)

EMQX集群原理可简述为下述两条规则:

- MQTT 客户端订阅主题时，所在节点订阅成功后广播通知其他节点：某个主题(Topic)被本节点订阅。 ———— 共享路由表
- MQTT 客户端发布消息时，所在节点会根据消息主题(Topic)，检索订阅并路由消息到相关节点。———— 根据路由信息，一路转发消息


使用过程中的一些发现：

- 当一个 broker 挂掉时，连接该 broker 的 client 会受到影响，需要重新连接；
- broker 重启时，client 需要重新订阅 Topic 因为这些信息不回在broker上保留；
- client 下线重新连接，订阅信息也会丢失，上线后需要重新订阅；
- Retain message 在集群方案下是正常的；
- **last will 集群方案下好像不行！**

## 2. 认证/访问控制

EMQ X 消息服务器 连接认证 和 访问控制 由一系列的认证插件(Plugins)提供，他们的命名都符合 emqx_auth_<name> 的规则，通过 allow_anonymous 配置可以打开匿名认证（默认是开启的）。可以同时开启多个认证模块，组成一个认证链，只要一个模块认证成功即可。

关于认证插件，可以参考[官网plugin](https://docs.emqx.io/broker/v3/cn/plugins.html)，可以关注下面几个:

- emqx_auth_http 插件：实现**连接认证**与**访问控制**的功能。它会将每个请求发送到指定的 HTTP 服务，通过其返回值来判断是否具有操作权限。
- emqx_auth_mysql/emqx_auth_pgsql 插件：实现**连接认证**与**访问控制**的功能, 当请求到来时，通过查询数据库来判断认证/访问权限。
- emqx_auth_redis 插件：通过访问 Redis 数据以实现 连接认证 和 访问控制 的功能。
- emqx_auth_mongo 插件： 类似 SQL 数据库

访问控制通过文件定义。


## 3. 配置SSL

按照以下步骤生成CA证书、EMQ证书、以及Client证书（只有双向认证的时候才需要）

```shell

# 准备
yum install openssl
mkdir /etc/emqx/ssl && chown emqx:emqx /etc/emqx/ssl 
cd /etc/emqx/ssl
echo 01 > /etc/pki/CA/serial
touch /etc/pki/CA/index.txt
# 生成 CA ,一般情况下根证书只要生成一次，之后所有的server/client证书都
openssl genrsa -out ca-key.pem 2048 # 私钥
openssl req -x509 -new  -key ca-key.pem -sha256 -days 3650 -subj "/C=CN/ST=Fujian/L=Fuzhou/O=EMQ/OU=Broker/CN=www.ruijie.cn" -out ca-csr.pem # 公钥

# 通过 CA 证书签发服务端证书
openssl genrsa -out server-key.pem 1024
openssl req -new -days 3650 -key server-key.pem -out server-cert.csr -subj "/C=CN/ST=Fujian/L=Fuzhou/O=EMQ/OU=Broker/CN=Server" # 创建一个签发请求
openssl ca -extensions v3_req -days 3650 -in server-cert.csr -out server-cert.pem -cert ca-csr.pem -keyfile ca-key.pem

# 修改文件配置
chmod +x /etc/emqx/ssl/*
chown emqx:emqx /etc/emqx/ssl/*

# 修改配置文件

listener.ssl.external.keyfile = /etc/emqx/ssl/server-key.pem
listener.ssl.external.certfile = /etc/emqx/ssl/server-cert.pem
listener.ssl.external.cacertfile = /etc/emqx/ssl/ca-csr.pem

# 重启服务
systemctl restart emqx

```

开启SSL后端，MQTT的端口为8883！

进行双向认证时，我们还需生成Client证书，生成方式类似服务端证书，额外的配置有：

```
## 开启双向认证
listener.ssl.external.verify = verify_peer
## 禁止单向认证
listener.ssl.external.fail_if_no_peer_cert = true
```

## 4. 配置EMQ代理

通常情况下，开启SSL后有两种方式配置SSL加密：

- nginx一侧配置SSL认证，后端连接1883端口（因此暴露到公网的是SSL 端口，而nginx到emqx一侧是不加密的端口）。
- nginx一侧不配置ssl，直接将ssl配置在emqx一侧。

启动nginx后，在nginx.conf最末尾添加下面的配置。
```
stream {
    upstream mqtt1883 {
      server 172.24.33.91:1883 weight=1;
      server 172.24.33.92:1883 weight=1;
      server 172.24.33.93:1883 weight=1;
     }
server {
        listen       1883;
        proxy_pass mqtt1883;
        proxy_buffer_size 3M;
        tcp_nodelay on;
        proxy_connect_timeout 150s;
        proxy_timeout 150s;
     }

upstream mqtt8883{
    #   hash $remote_addr consistent;
        least_conn;
        server 172.24.33.91:1883 weight=1;
        server 172.24.33.92:1883 weight=1;
        server 172.24.33.93:1883 weight=1;
}

server {
        listen 8883 ssl;
        proxy_connect_timeout 150s;
        proxy_timeout 150s;
        proxy_pass mqtt8883;
        proxy_buffer_size 3M;
        ssl_protocols       TLSv1 TLSv1.1 TLSv1.2;
        ssl_ciphers         AES128-SHA:AES256-SHA:RC4-SHA:DES-CBC3-SHA:RC4-MD5;
        ssl_certificate     /root/ssl_emqx/server-cert.pem;
        ssl_certificate_key /root/ssl_emqx/server-key.pem;
        ssl_session_cache   shared:SSL:10m;
        ssl_session_timeout 10m;
    }
```


## 4. 性能测试

参考官方的一个调优文档，以及官方性能测试报告，修改以下配置后进行调优：


- Linux 配置
```
# 创建添加/etc/modules-load.d/nf_conntrack.conf 文件自动加载以下模块

nfnetlink
nf_conntrack_ipv4
nf_defrag_ipv4

# /etc/sysctl.conf 添加下面的配置，并执行sysctl -p
fs.file-max = 2097152
fs.nr_open = 2097152
fs.file-max = 1048576
net.core.somaxconn = 32768
net.ipv4.tcp_max_syn_backlog = 16384
net.core.netdev_max_backlog = 16384
net.ipv4.ip_local_port_range = 1000 65535
net.core.rmem_default = 262144
net.core.wmem_default = 262144
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.core.optmem_max = 16777216
net.ipv4.tcp_rmem = 1024 4096 16777216
net.ipv4.tcp_wmem = 1024 4096 16777216
net.nf_conntrack_max = 1000000
net.netfilter.nf_conntrack_max = 1000000
net.netfilter.nf_conntrack_tcp_timeout_time_wait = 30
net.ipv4.tcp_max_tw_buckets = 1048576
net.ipv4.tcp_fin_timeout = 15

# 修改linux 最大连接数，在/etc/security/limits.conf 添加以下内容，修改完成重新连接shell 生效

*      soft   nofile      1048576
*      hard   nofile      1048576

# 修改 /etc/systemd/system.conf 添加
DefaultLimitNOFILE=1048576

```
- Erlang虚拟机配置

```
## Erlang Process Limit
node.process_limit = 2097152

## Sets the maximum number of simultaneously existing ports for this system
node.max_ports = 1048576

```

- EMQX 服务配置

```
listener.ssl.external.acceptors = 64
listener.ssl.external.max_connections = 102400
listener.ssl.external.send_timeout = 60s
listener.ssl.external.handshake_timeout = 60s
listener.ssl.external.max_conn_rate = 2500

node.async_threads = 32

```

- nginx 服务器的配置参考EMQX 服务器，配置文件参考IData的。

压力测试中几个注意的点：

- 单台压力机提供的并发上限，大概是6w左右。原因是由于 client 和 broker 固定的端口建立连接，因此测试时并发数受限于client上的可以用端口数（net.ipv4.ip_local_port_range）。默认情况下，linux 允许创建的线程是30000左右，会限制压测工具创建的连接数（一个线程一个连接），请修改 /proc/sys/kernel/pid_max 配置以及文件句柄数的相关配置。使用jmeter作为压测工具时，6w并发连接需要消耗60G左右内存，以及大量的CPU！

- 集群环境中，使用 nginx 代理时，单个 broker 最多提供60000w并发连接（原因是nginx的端口数量限制了和broker的连接）。

- 参考配置： 8vcore + 16GB 内存 --> 4w 并发连接比较稳定，连接数继续提高时会出现连接失败的情况

# 参考

[中文文档](https://docs.emqx.io/broker/latest/cn/)

[调优指南](https://docs.emqx.io/broker/latest/cn/tune.html#linux)

[【官方测试报告】](https://emq-xmeter-benchmark-cn.readthedocs.io/en/latest/index.html)
