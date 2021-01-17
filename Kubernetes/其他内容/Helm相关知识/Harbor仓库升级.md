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



# 1.9.x 升级2.0.2 分析

**重要事项**：

- 升级前请一定备份数据库！！！！！
- **请打开Debug日志**，从debug日志可以大概判断进度
- 做好充足准备升级时间可能极度漫长，**建议直接编辑存活探针到3个小时以上，或者在values.xml中加入配置core.livenessProbe.initialDelaySeconds**

## 升级源码分析

升级过程的时间长短主要取决于**artifact**表的大小！

**harbor-core**容器的工作目录如下：

```shell
/harbor/entrypoint.sh  
/harbor/harbor_core    # 主程序
/harbor/install_cert.sh
/harbor/migrations    # postgresql 的sql文件
/harbor/versions      # 各个组件的版本信息
/harbor/views
```

启动脚本为**entrypoint.sh**，内容如下：

```shell
#!/bin/sh
set -e
/harbor/install_cert.sh # 该脚本将用户的证书CP到容器内的指定位置
/harbor/harbor_core     
```

**harbor-core**的main函数中，**migration/migration.go**的**Migrate(database *models.Database) **方法是整个迁移操作的入口。

```go
func Migrate(database *models.Database) error {
	// 这里migrator是一个db迁移对象，来自于golang-migrate 这个迁移工具
	migrator, err := dao.NewMigrator(database.PostGreSQL)  
	if err != nil {
		return err
	}
	defer migrator.Close()
	
    // 获取当前DB的schema版本信息
	schemaVersion, _, err := migrator.Version()
	if err != nil && err != migrate.ErrNilVersion {
		return err
	}
	log.Debugf("current database schema version: %v", schemaVersion)
	// prior to 1.9, version = 0 means fresh install
	if schemaVersion > 0 && schemaVersion < 10 {
		return fmt.Errorf("please upgrade to version 1.9 first")
	}

    // 升级Schema
	// 更新pg中的schema文件，应该是从/harbor/migrations中读取sql文件在数据库中执行
    // 具体实现是 common/dao/pgsql.go:105 的 UpgradeSchema() 函数，该函数中调用的是 golang-migrate 工具的接口
	if err := dao.UpgradeSchema(database); err != nil {
		return err
	}

    // 升级DB的记录
	// 根据 schema_migrations 表的 data_version 版本判断数据的版本
	ctx := orm.NewContext(context.Background(), beegorm.NewOrm())
	dataVersion, err := getDataVersion(ctx)
	if err != nil {
		return err
	}
	log.Debugf("current data version: %v", dataVersion)
	// the abstract logic already done before, skip
	if dataVersion == 30 {
		log.Debug("no change in data, skip")
		return nil
	}

	// 数据升级入口函数，该节点非常消耗时间
	if err = upgradeData(ctx); err != nil {
		return err
	}

	return nil
}
```

**upgradeData(ctx context.Context)** 接口是数据升级的入口，数据升级时主要涉及到下面这几张表。该函数的主要结构即依照下面三张表构成了一个三层循环函数：

- project：项目信息表
- repository：镜像信息表
- artifact：镜像版本信息表

```go
func upgradeData(ctx context.Context) error {
    // abstractor 中封装了 regCli 以及 artifact 的db操作接口
	abstractor := art.NewAbstractor()
	pros, err := project.Mgr.List()
	if err != nil {
		return err
	}
    
    // 循环1
	for _, pro := range pros {
		repos, err := repository.Mgr.List(ctx, &q.Query{
			Keywords: map[string]interface{}{
				"ProjectID": pro.ProjectID,
			},
		})
		if err != nil {
			log.Errorf("failed to list repositories under the project %s: %v, skip", pro.Name, err)
			continue
		}
        
        // 循环2
		for _, repo := range repos {
			log.Debugf("abstracting artifact metadata under repository %s ....", repo.Name)
			arts, err := artifact.Mgr.List(ctx, &q.Query{
				Keywords: map[string]interface{}{
					"RepositoryID": repo.RepositoryID,
				},
			})
			if err != nil {
				log.Errorf("failed to list artifacts under the repository %s: %v, skip", repo.Name, err)
				continue
			}
            
            // 循环3：主要目的即更新artifact的数据库结构
			for _, art := range arts {
				// abstract 根据references字段递归更新数据库记录，实际生产中发现 这张表并没有 references 字段
                // 但是数据库中还有一张 artifact_reference 表，不过貌似也是空的
				if err = abstract(ctx, abstractor, art); err != nil {
					log.Errorf("failed to abstract the artifact %s@%s: %v, skip", art.RepositoryName, art.Digest, err)
					continue
				}
                // artifact
				if err = artifact.Mgr.Update(ctx, art); err != nil {
					log.Errorf("failed to update the artifact %s@%s: %v, skip", repo.Name, art.Digest, err)
					continue
				}
			}
			log.Debugf("artifact metadata under repository %s abstracted", repo.Name)
		}
	}

	// 更新函数
	return setDataVersion(ctx, 30)
}

// 递归的 abstract 
func abstract(ctx context.Context, abstractor art.Abstractor, art *artifact.Artifact) error {
	// abstract the children
	for _, reference := range art.References {
		child, err := artifact.Mgr.Get(ctx, reference.ChildID)
		if err != nil {
			log.Errorf("failed to get the artifact %d: %v, skip", reference.ChildID, err)
			continue
		}
		if err = abstract(ctx, abstractor, child); err != nil {
			log.Errorf("failed to abstract the artifact %s@%s: %v, skip", child.RepositoryName, child.Digest, err)
			continue
		}
	}
	// 同步abstract 的metadata信息 ~~~ 数据升级的关键
    // 调用 PullManifest 方法
	return abstractor.AbstractMetadata(ctx, art)
}
```

**AbstractMetadata**会调用registryCli来获取S3数据存储中的实际元数据信息，该方法实际上调用的是一个HTTP Get请求

对应的URL为 **/v2/{镜像名称}/manifests/{tag信息}**，示例如下。

```go
/v2/5fb68a6309d09c00102a3feb/consumer-video/manifests/sha256:fc89853b8e5c0c377549ad60444ff8ec6a26597969207b454fb3b8f078caca8a
```

未完待续！！







# 参考

[Harbor 镜像扫描](https://youendless.com/post/harbor_image_scan/)

[Harbor版本升级记录](https://www.dazhuanlan.com/2019/12/24/5e018a67c0cad/)

[Migration - 1.6.0](https://github.com/goharbor/harbor/blob/release-1.6.0/docs/migration_guide.md)

[Upgrade](https://goharbor.io/docs/2.0.0/administration/upgrade/)