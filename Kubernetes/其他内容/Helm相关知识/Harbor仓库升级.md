# Harbor升级方案

### 升级v1.5.0版本

```shell
# Harbor 服务停机
docker-compose down

# 备份数据
cp /data/harbor /data/harbor_backup

# 作为环境变量供调用
harbor_db_path=/data/harbor/database
harbor_cfg=/root/harbor/harbor_150.cfg # 1.50 版本的启动文件


# 升级数据库文件（# PS：如果使用了clair和notary，这两个东西要独立升级）
docker run -it --rm -e DB_USR=root -e DB_PWD=root123 -v ${harbor_db_path}:/var/lib/mysql -v ${harbor_cfg}:/harbor-migration/harbor-cfg/harbor.cfg goharbor/harbor-migrator:v1.6.0 up

# 依次通过docker-compose启动 v1.6.3、v1.7.6、v1.9.4 版本的harbor (数据目录为/data/harbor/database)

# 导出PGSQL中Registry数据库的数据
docker exec  harbor-db /bin/sh -c  "pg_dump -U postgres --no-owner --no-privileges --schema-only  registry" > registry-schema.sql
docker exec  harbor-db /bin/sh -c  "pg_dump -U postgres --inserts --data-only  registry" > registry-data.sql

# 导入数据到生产PG
```

为了加快数据库导入导出数据，建议清空 registry 库的 img_scan_job 表和 img_scan_overview 表：

```shell
docker run -it -d -v ${harbor_db_path}:/var/lib/mysql:z -e MYSQL_ROOT_PASSWORD=root123 --name harbor-db-test vmware/harbor-db:v1.5.0

echo "truncate img_scan_job;" | mysql -proot123 -Dregistry
echo "truncate img_scan_overview;" | mysql -proot123 -Dregistry

docker stop harbor-db-test
docker rm harbor-db-test
```

## Helm 环境数据库迁移

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

导入数据到DB：

```shell
# AWSCA 
PGPASSWORD={PASSWORD} psql -h {HOST_IP} -p 5431  -d registry -U {USERNAME} -f xxx.sql
# HK
PGPASSWORD={PASSWORD} psql -h {HOST_IP} -p 5431  -d registry -U {USERNAME} -f xxx.sql
# cl
PGPASSWORD={PASSWORD} psql -h {HOST_IP} -p 5431  -d registry -U {USERNAME} -f xxx.sql
```
# 升级问题回顾

- 1.7.6 - > 1.9.4 -> 2.0.2 每个版本升级时，harbor-core 都需要花费大量时间重启。如果部署在Kubernetes中，探针机制会主动杀死Harbor Core 导致升级失败。因此需要手工编辑Ready探针的时间（建议直接编辑存活探针到3个小时以上，或者在values.xml中加入配置core.livenessProbe.initialDelaySeconds）.
- Nginx双跳配置：如果使用的是Ingress暴露Harbor服务，并且Client通过Nginx的反向代理访问Harbor的API。此时要Harbor一侧配置一个Ingress，这个Ingress的配置和原来类似，但是Host指向Nginx代理的域名。（**没理解为啥！！！**）
- 升级过后LDAP Server连接异常，但是去除 LDAP搜索DN 搜索后恢复正常
- Properties 表有脏数据，导致debug日志疯狂刷新
- 远程Registry连接失败，处理方式删除registry表的记录并新建一个，然后将id字段改回旧的
- GC停止方式：删除数据库记录、admin_job表、清除redis缓存、重启jobservice
- HK 环境垃圾收集失败，registryctl 容器有 distribution 相关异常。社区的解决方案（[issues-2695](https://github.com/docker/distribution/issues/2695)）：降级到2.5.2版本（2.6.x、2.7.1 都有类似异常）

# 参考

[Harbor 镜像扫描](https://youendless.com/post/harbor_image_scan/)

[Harbor版本升级记录](https://www.dazhuanlan.com/2019/12/24/5e018a67c0cad/)

[Migration - 1.6.0](https://github.com/goharbor/harbor/blob/release-1.6.0/docs/migration_guide.md)

[Upgrade](https://goharbor.io/docs/2.0.0/administration/upgrade/)