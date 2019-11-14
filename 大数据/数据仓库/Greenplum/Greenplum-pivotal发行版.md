---
title: 数据库调研笔记 -- GreenPlum
categories: "笔记"
date: 2019-10-21
comments: true
toc: true
tags:
	- GreenPlum
	- 数据库
---

GreenPlum数据库，pivotal发行版安装

<!--more-->

# 安装Greenplum


Pivotal发行版的安装和开源Greenplum过程类似，有以下几点需要注意以下：

- Pivotal推荐的系统配置和开源版本略有不同，其中着重注意：kernel.shmall、kernel.shmmax、 vm.min_free_kbytes等内存相关配置。可以参考[Linux System Settings](https://gpdb.docs.pivotal.io/6-0/install_guide/prep_os.html#topic3)部分。

```properties
kernel.shmall = 16467429        # 任意时刻可以分配的所有共享内存段的总和的最大值(以页为单位)，整机内存页的一半， echo $(expr $(getconf _PHYS_PAGES) / 2) 
kernel.shmmax = 67450589184     # 共享内存段的最大尺寸，整机内存的一半，echo $(expr $(getconf _PHYS_PAGES) / 2 \* $(getconf PAGE_SIZE))
kernel.shmmni = 4096            # 系统范围内共享内存段的最大数量
vm.overcommit_memory = 2        # 允许超额分配内存
vm.overcommit_ratio = 95        # 应用进程能够使用的RAM的百分比，剩下的内存就会留给操作系统。默认值是50%
net.ipv4.ip_local_port_range = 1025 65535    # 本地自动分配的TCP, UDP端口号范围 
kernel.sem = 500 2048000 200 40960
kernel.sysrq = 1
kernel.core_uses_pid = 1
kernel.msgmnb = 65536
kernel.msgmax = 65536
kernel.msgmni = 2048
net.ipv4.tcp_syncookies = 1
net.ipv4.conf.default.accept_source_route = 0
net.ipv4.tcp_max_syn_backlog = 4096
net.ipv4.conf.all.arp_filter = 1
net.core.netdev_max_backlog = 10000
net.core.rmem_max = 2097152
net.core.wmem_max = 2097152
vm.swappiness = 10
vm.zone_reclaim_mode = 0                    # 禁用 numa, 或者在vmlinux中禁止
vm.dirty_expire_centisecs = 500
vm.dirty_writeback_centisecs = 100
vm.dirty_background_ratio = 0               # 内存大于64GB的主机使用此配置
vm.dirty_ratio = 0                          # 内存大于64GB的主机使用此配置
vm.dirty_background_bytes = 1610612736      # 配置约为1.5GB，系统脏页到达这个值，系统后台刷脏页调度进程
vm.dirty_bytes = 4294967296                 # 配置约为4GB
vm.min_free_kbytes = 2097152                # 内存的2%~3%
```


- Pivotal建议使用xfs系统，挂载参数为：rw,nodev,noatime,nobarrier,inode64

```shell
# 磁盘挂载的例子
/dev/data /data xfs nodev,noatime,nobarrier,inode64 0 0
```

- 使用blockdev命令修改硬盘的预读扇区为16384个（每个为512字节，默认情况下只预读256个扇区）

```shell
# 以下设定都是临时的，需要添加到开机启动中
/sbin/blockdev --getra /dev/sdb # 获取预读扇区的值
/sbin/blockdev --setra 16384 /dev/sdb # 修改预读扇区的值
```

- 修改磁盘的IO调度策略，Pivotal推荐的策略是deadline 

```shell
#  以下设定都是临时的，需要添加到开机启动中
echo deadline > /sys/block/sdb/queue/scheduler # 修改sbd的策略为deadline，默认即为deadline不用修改
#  想要彻底修改scheduler，需要编辑 /boot/grub2/grub.cfg，或者使用grubby 工具
grubby --update-kernel=ALL --args="elevator=deadline"
```
- 关闭 Transparent Huge Pages 功能

```shell
# 查看是否开启了transparent_hugepage
cat /sys/kernel/mm/transparent_hugepage/enabled
# 用久关闭，执行以下命令并重启
grubby --update-kernel=ALL --args="transparent_hugepage=never"
```

- 关闭RemoveIPC，原因参考：[18.4.2. systemd RemoveIPC](https://www.postgresql.org/docs/11/kernel-resources.html)


- pivotal建议添加了一个环境变量：export LD_PRELOAD=/lib64/libz.so.1 ps

# 安装 Greenplum Command Center

```shell
# 在master节点执行以下命令安装，按照交互提示选择即可
./gpccinstall-6.0.0 -W

# 安装完成后添加以下访问权限
local   gpperfmon       gpmon                   md5
host    all             gpmon   127.0.0.1/28    md5
host    all             gpmon   ::1/128         md5
host    gpperfmon       gpmon   all             md5

# 启动gpcc服务
gpcc start -W
# 查看gpcc服务状态
gpcc status -W
# WEB入口，登录用户为gpmon
http://{gpcc_host}}}:28080

```
  
# 参考

[Pivotal Greenplum® 6.0](https://gpdb.docs.pivotal.io/6-0/main/index.html)

[GPCC安装](https://gpcc.docs.pivotal.io/600/topics/install.html)