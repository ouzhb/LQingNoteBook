# 说明

在单机环境中为了使用PVC、PV等功能，可以在环境中搭建Open targetd服务，为容器提供基于iscsi连接的数据存储。

## 1. 安装targetd

```shell
# 安装target套件
yum install -y targetcli targetd iscsi-initiator-utils
# 启动服务
systemctl enable target && systemctl start target
```

targetd 可以使用整个磁盘的容量，假设使用/dev/vdb作为存储，我们可以按照下面方式创建targetd的存储池

```shell
pvcreate /dev/vdb
vgcreate vg-targetd /dev/vdb
lvcreate -L 15G --thinpool pool vg-targetd
```

修改配置文件/etc/target/targetd.yaml并重启targetd

```shell

# defaults below; uncomment and edit
# if using a thin pool, use <volume group name>/<thin pool name>
# e.g vg-targetd/pool

pool_name: vg-targetd
user: admin
password: ciao
ssl: false
target_name: iqn.2003-01.org.linux-iscsi.minishift:targetd
```

## 2. initiator 配置

- 编辑 /etc/iscsi/initiatorname.iscsi 决定客户端名称

- systemctl restart iscsid && systemctl enable iscsid

## 3. 安装 iSCSI-Provisioner

Provisioner 的功能是：监听到Kubernetes请求PVC时自动连接targetd，创建Lun并生成pv，并且维护他的整个生命周期。 --- 动态创建PV

可以使用 [external-storage](https://github.com/kubernetes-incubator/external-storage) 项目下的 targetd 控制器实现上面的功能。



# 参考

[安装targetd](https://typefo.com/linux/iscsi-server-for-centos7.html)

[iscsi StorageClass](https://github.com/kubernetes-incubator/external-storage/tree/master/iscsi/targetd)