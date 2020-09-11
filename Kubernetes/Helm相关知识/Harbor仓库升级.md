# Harbor升级方案

### 升级v1.5.0版本

```shell
# Harbor 服务停机
docker-compose down

# 备份数据
cp /data/harbor /data/harbor_backup

# 作为环境变量供调用
harbor_db_path=/data/harbor/database
harbor_cfg=/root/harbor/harbor.cfg # 1.50 版本的启动文件


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
docker run -it -d -v /root/lq/database:/var/lib/mysql:z -e MYSQL_ROOT_PASSWORD=root123 --name harbor-db-test vmware/harbor-db:v1.5.0

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

# 参考

[Harbor 镜像扫描](https://youendless.com/post/harbor_image_scan/)

[Harbor版本升级记录](https://www.dazhuanlan.com/2019/12/24/5e018a67c0cad/)

[Migration - 1.6.0](https://github.com/goharbor/harbor/blob/release-1.6.0/docs/migration_guide.md)

[Upgrade](https://goharbor.io/docs/2.0.0/administration/upgrade/)