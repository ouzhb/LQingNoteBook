# Harbor升级方案

当前环境部署情况：

| 环境  | Harbor版本 | 部署方式           | 数据库   | S3                                 |
| ----- | ---------- | ------------------ | -------- | ---------------------------------- |
| 长乐  | v1.5.0     | Docker Compose部署 | 本地单点 | http://portal.s3.nds.sdp           |
| 无锡  | v1.5.0     | Docker Compose部署 | 本地单点 | http://presls3.101.com             |
| awsca | v1.9.0     | Helm               | 本地单点 | https://s3-us-west-1.amazonaws.com |
| hk    | v1.9.0     | Helm               | 本地单点 | http://s3.hk.101.com               |

## 1. 长乐/无锡环境

```Harbor```在```1.6.0```版本中，将数据库服务从```Mariadb```切换成了```Postgresql```，并且即使```1.6.0```之后的版本升级，也并不是非常平滑。

如果要从 ```v1.5.0```升级到最新的```v2.0.2```，参考官方需要有以下升级路径：```v1.5.0``` -->```1.6.0```-->```1.7.0```-->```1.9.0```-->```2.0.2```。

考虑到长乐/无锡环境的**S3数据需要迁移**，因此建议：**直接在K8S环境中部署2.0.2版本的Harbor，通过编写脚本的方式讲将需要的Image导出到新的harbor中**。

### 升级v1.5.0版本并迁移数据库

```
# Harbor 停机
docker-compose down
# CP 拷贝数据库目录文件，并下载到指定机器进行数据库升级
tar -cf cl_prod_database.tar /data/harbor/database/
gzip cl_prod_database.tar

# 转换数据库文件格式，实际拿长乐环境的数据测试过，长乐环境的DB有14GB左右，导出原始数据后有4.8G左右。最大的表img_scan_job 有2200w条数据
harbor_db_path=${解压数据库文件到指定目录}
harbor_cfg=${指定v1.5.0版本harbor.cfg配置文件} 
docker run -it --rm -e DB_USR=root -e DB_PWD=root123 -v ${harbor_db_path}:/var/lib/mysql -v ${harbor_cfg}:/harbor-migration/harbor-cfg/harbor.cfg goharbor/harbor-migrator:v1.6.0 up

# 依次通过docker-compose启动 v1.6.3、v1.7.6、v1.9.4 版本的harbor

# 导出PGSQL中Registry数据库的数据
docker exec  harbor-db /bin/sh -c  "pg_dump -U postgres --no-owner --no-privileges --schema-only  registry" > registry-schema.sql
docker exec  harbor-db /bin/sh -c  "pg_dump -U postgres --inserts --data-only  registry" > registry-data.sql
```

为了加快数据库导入导出数据，建议清空 registry 库的 img_scan_job 表和 img_scan_overview 表：

```
docker run -it -d -v /root/lq/database:/var/lib/mysql:z -e MYSQL_ROOT_PASSWORD=root123 --name harbor-db-test vmware/harbor-db:v1.5.0

echo "truncate img_scan_job;" | mysql -proot123 -Dregistry
echo "truncate img_scan_overview;" | mysql -proot123 -Dregistry

docker stop harbor-db-test
docker rm harbor-db-test
```



### Helm 2.0.1 安装

```
helm repo add harbor https://helm.goharbor.io
helm fetch harbor/harbor --untar
helm install -n harbor ./  --namespace harbor --values xxx-value.yaml

PS：如何编辑value.yaml文件参考 http://git.sdp.nd/k8s/charts/tree/master/harbor
```

## 2. AWS/HK 环境

AWS/HK环境部署的是``` v1.9.0```可以平滑升级到最新版本，**但是Harbor使用的数据库是本地单点数据，因此升级前需要进行一次数据库迁移**。

### 数据库迁移操作

如下命令导出原有数据库的Schema：

```
kubectl -n harbor exec  harbor-harbor-database-0 -- pg_dump -U postgres --no-owner --no-privileges --schema-only  registry > registry-schema.sql
kubectl -n harbor exec  harbor-harbor-database-0 -- pg_dump -U postgres --no-owner --no-privileges --schema-only  notaryserver > notaryserver-schema.sql
kubectl -n harbor exec  harbor-harbor-database-0 -- pg_dump -U postgres --no-owner --no-privileges --schema-only  notarysigner > notarysigner-schema.sql
```
如下命令导出原有数据库的实际数据：
```
kubectl -n harbor exec  harbor-harbor-database-0 -- pg_dump -U postgres --inserts --data-only  registry > registry-data.sql
kubectl -n harbor exec  harbor-harbor-database-0 -- pg_dump -U postgres --inserts --data-only  notaryserver > notaryserver-data.sql
kubectl -n harbor exec  harbor-harbor-database-0 -- pg_dump -U postgres --inserts --data-only  notarysigner > notarysigner-data.sql
```


# 各个版本升级操作

## 1.5.0升级到1.6.0

原始数据库备份

```shell
# Harbor 服务停机
docker-compose down

# 获取迁移工具
docker pull goharbor/harbor-migrator:v1.6.0 

# 备份DB和Cfg数据，到备份目录
rm -rf /root/harbor_bakcup && mkdir -p /root/harbor_bakcup

# 作为环境变量供调用
backup_path=/root/harbor_bakcup
harbor_db_path=/data/harbor/database
harbor_cfg=/root/harbor/harbor.cfg # 1.50 版本的启动文件

# 执行数据库备份，备份完成后会输出一个SQL文件和harbor.cfg
docker run -it --rm -e DB_USR=root -e DB_PWD=root123 -v ${harbor_db_path}:/var/lib/mysql -v ${harbor_cfg}:/harbor-migration/harbor-cfg/harbor.cfg -v ${backup_path}:/harbor-migration/backup goharbor/harbor-migrator:v1.6.0 backup
```

数据库升级

```shell
# 进行数据库schema和数据的升级，会直接重写 ${harbor_db_path} 的数据文件，并且会重写 ${harbor_cfg} 文件
docker run -it --rm -e DB_USR=root -e DB_PWD=root123 -v ${harbor_db_path}:/var/lib/mysql -v ${harbor_cfg}:/harbor-migration/harbor-cfg/harbor.cfg goharbor/harbor-migrator:v1.6.0 up

# PS：如果使用了clair和notary，这两个东西要独立升级
```

## 1.6.0以上版本升级

1.6.0 以上版本Harbor将数据库的升级集成到了容器中，实现了自动化升级。用户只要下载高版本的Helm包，然后修改配置重启服务即可。

# 参考

[Harbor 镜像扫描](https://youendless.com/post/harbor_image_scan/)

[Harbor版本升级记录](https://www.dazhuanlan.com/2019/12/24/5e018a67c0cad/)

[Migration - 1.6.0](https://github.com/goharbor/harbor/blob/release-1.6.0/docs/migration_guide.md)

[Upgrade](https://goharbor.io/docs/2.0.0/administration/upgrade/)