# 功能&特性

- 云原生应用，部署简单（可以本地部署、容器部署、Kubernetes部署）；
- 基于 erasure code 方式的HA，容忍 disks/2 个目录在线时可读，(disks/2 + 1）个目录在线时可写；
    - 由于纠删码的工作原理，至少需要三个节点才能保证Minio在主机级别的高可用；
    - Minio 能够实现硬盘级别高可用；
    - 当前限制当minio实例，管理16个driver
- Minio在分布式和单机模式下，所有读写操作都严格遵守read-after-write一致性模型；
- 支持 TLS 访问服务（[参考](https://docs.min.io/cn/how-to-secure-access-to-minio-server-with-tls.html)）；
- 支持事件通知机制（[参考](https://docs.min.io/docs/minio-bucket-notification-guide.html)）



# 纠删码机制

纠删码是一种通过对原始的数据(data)进行编码得到冗余(parity)的数据备份手段。其思想是将n块原始的数据元素通过一定的计算，得到m块冗余元素（校验块），对于这n+m 块的元素，当其中任意的x块元素出错（包括原始数据和冗余数据）时，均可以通过对应的重构算法恢复出原来的n块数据。

minio 使用 erasure code（纠删码）的机制来解决数据的可靠性，有以下特点：

- minio使用的纠删码算法是 [klauspost/reedsolomon](https://github.com/klauspost/reedsolomon)
- 通常minio，n=m=disks/2（disks为整个minio系统管理的磁盘数目录），容忍 disks/2 个目录在线时可读，(disks/2 + 1）个目录在线时可写；


# 配置文件


参考：https://github.com/minio/minio/tree/master/docs/config

# 部署

## 分布式部署

在三节点环境中部署，每个节点启动一个Minio实例，每个实例管理2个Brick，对应的 docker-compose 文件如下：

```yaml
# run docker-compose -f minio.yml  up -d
version: '3.3'
services:
  minio:
    image: minio/minio:latest
    container_name: minio
    volumes:
      - "/data/minio/brick1:/data1"
      - "/data/minio/brick2:/data2"
    ports:
      - "9000:9000"
    environment:
      MINIO_ACCESS_KEY: admin
      MINIO_SECRET_KEY: omc_admin
    command: server http://node{1...3}:9000/data{1...2}
    network_mode: host
    restart: always
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:9000/minio/health/live"]
      interval: 30s
      timeout: 20s
      retries: 3
```


# 使用过程发现的问题

## MYSQL Notification

实际测试中发下，使用 Mysql 作为Target时存在一些问题，详情可以参考 [#8998](https://github.com/minio/minio/issues/8998)。

PS：

- dsn_string 的写法可以[参考连接](https://github.com/go-sql-driver/mysql)（例子：root:Ruijie_123@tcp(172.24.33.77:3306)/minio_event）。
- Minio 的版本变更非常的块，遇到bug可以尝试换一个版本。