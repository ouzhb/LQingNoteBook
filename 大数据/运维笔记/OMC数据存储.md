# Linux中使用随机数

Linux中可以使用以下方式生产随机数：

- $RANDOM环境变量
- /dev/random
- /dev/urandom
- haveged

经过比较使用 haveged 命令直接生成随机数的性能要高于从 /dev/urandom 或者 /dev/random。

注意：

- /dev/random 和 /dev/urandom 区别：

    - /dev/random 是真隨機數生成器，它會消耗熵值來產生隨機數，同時在熵耗盡的情況下會阻塞，直到有新的熵生成。
    - /dev/urandom 是僞隨機數生成器，它根據一個初始的隨機種子(這個種子來源就是熵池中的熵)來產生一系列的僞隨機數，而並不會在熵耗盡的情況下阻塞。

- 关于Linux环境下，随机数生成器和系统熵的关系可以参考[链接](https://www.twblogs.net/a/5d4d0a7dbd9eee541c30e613)，[链接](http://jhurani.com/linux/2017/11/01/entropy-explained.html)。


PS：测试过程中，即使使用 haveged 生成随机数，压力测试依然无法写满机械盘（使用率在30%~50%之间）


# 压缩性能

使用PIGZ可以并行压缩文件，或者解压文件。

```shell
# install
yum –y install pigz
# 压缩
tar -c files_dir/ | pigz -p 16 –c > compress_files.tar.gz
```

# rsync

使用命令方式同步

```shell
# pull
rsync -av node12:/data/data_ssd/omc/ /data/data_ssd/omc/
# push
rsync -av /data/data_ssd/omc/ node12:/data/data_ssd/omc/
```

使用DAEMON方式同步时，服务端/客户端启动DAEMON进程，并创建配置文件/etc/rsyncd.conf如下：

```shell
# 接收端配置

uid = root
gid = root
use chroot = no
max connections = 200
timeout = 300 
pid file = /var/run/rsyncd.pid
lock file = /var/run/rsync.lock
log file = /var/log/rsyncd.log

[OMC] 
path = /data/data_ssd/omc
ignore errors 
read only = false 
list = false 
hosts allow = 172.24.9.0/24
#host deny = 0.0.0.0/32
auth users = root
secrets file = /etc/rsyncd.passwd # 该文件需要在服务端指定，格式为root:passwd，权限为600

# 接收端启动rsyncd守护
rsync --daemon

# 发送端创建密码文件，权限为600
echo "ruijie" > /etc/rsync.passwd

# 发送端执行
rsync -a --password-file=/etc/rsync.passwd /data/data_ssd/omc/ node12::OMC
```

## 实时同步

实时同步通过 rsync + inotify 对每一个小文件进行在ftp服务器上进行实时同步。根据网上的测评：rsync + inotify 方案大约支持每秒200并发的同步。

由于 rsync + inotify 的方案需要配置脚本，使用起来较为复杂考虑使用 [sersync](https://github.com/wsgzao/sersync)进行实时同步，该工具是基于  rsync + inotify 的二次封装接口，有较多额外功能。

测试结果：

使用 sersync 在A主机生成300GB小文件，实时同步到B主机，当A主机的300GB小文件全部生成完毕，B主机同步了30GB左右文件。

# INODE资源

如果小文件的平均大小小于2KB，需要考虑文件系统可能出现INODE资源耗尽的场景，原因是：通常文件系统2KB大小创建一个inode！

关于INODE检查参考：[http://www.ruanyifeng.com/blog/2011/12/inode.html]）。

磁盘文件系统一旦建立INODE数目不能更改，因此格式化文件系统时需要注意：

- ext4 默认情况下16kb 空间创建一个INODE，如果使用 ext4 作为文件系统建议显式指定INODE数目（mkfs.ext4 /dev/xvdb1 -N {INODE数目} ）
- XFS 默认情况 2kb 创建创建一个INODE，如果小文件平均大小小于2KB，也需要显式指定。

# tmpfs

tmpfs可以将操作系统的内存虚拟化成硬盘设备，使应用的磁盘IO都发生在内存中性能得到极大提升。掉电或者unmout后，tmpfs中的数据就会丢失。

```shell
# 挂载64g内存盘到/data/tmpfs目录
mount tmpfs /data/tmpfs -t tmpfs -o size=64g
```
