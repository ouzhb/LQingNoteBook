# 安装


## 使用 Centos 存储SIG源
在Centos7上可以通过以下方式重RPM包安装，但是只能安装Glusterfs6。

Centos7 上通过centos-release-gluster安装，严格来说这个不是官方提供的安装包。
```
yum -y install centos-release-gluster
yum -y glusterfs-server

# 参考：https://wiki.centos.org/zh/SpecialInterestGroup/Storage
# 参考：https://wiki.centos.org/SpecialInterestGroup/Storage/gluster-Quickstart
```

其他Repo提供Cento7的分发包：

```
[centos-gluster7]
name=CentOS-7- Gluster 7
baseurl=http://buildlogs.centos.org/centos/7/storage/x86_64/gluster-7/
gpgcheck=0
enabled=1
```

## 源码安装

按照以下步骤可以编译RPM包（[参考](https://docs.gluster.org/en/latest/Install-Guide/compiling-rpms/)）：

```shell
yum -y install gcc python-devel python-setuptools
yum -y install libacl-devel userspace-rcu-devel
yum -y --disablerepo=rhs* --enablerepo=*optional-rpms install git autoconf \
    automake bison dos2unix flex fuse-devel glib2-devel libaio-devel \
    libattr-devel libibverbs-devel librdmacm-devel libtool libxml2-devel lvm2-devel make \
    openssl-devel pkgconfig pyliblzma python-devel python-eventlet python-netifaces \
    python-paste-deploy python-simplejson python-sphinx python-webob pyxattr readline-devel \
    rpm-build systemtap-sdt-devel tar libcmocka-devel

git clone https://github.com/gluster/glusterfs.git
git checkout release-7

cd glusterfs
./autogen.sh
./configure
make
cd extras/LinuxRPM
make_glusterrpms
```

# Quick Start

```shell
# 参考：
# https://github.com/LinQing2017/DevOpsTools/tree/master/ha_ftpserver
```