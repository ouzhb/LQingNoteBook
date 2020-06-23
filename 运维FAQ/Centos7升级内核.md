# Yum源升级

```shell
rpm --import https://www.elrepo.org/RPM-GPG-KEY-elrepo.org
yum install https://www.elrepo.org/elrepo-release-7.el7.elrepo.noarch.rpm

# 列出所有哦可用的内核安装包
# - lt：longterm，长期维护版
# - ml：mainline，最新稳定版
yum --disablerepo="*" --enablerepo="elrepo-kernel" list available

# 安装内核，
yum -y install --disablerepo="*" --enablerepo="elrepo-kernel" kernel-lt kernel-lt-devel
# gcc依赖kernel-lt-headers，因此如果环境安装了gcc需要卸载旧的kernel-lt-headers，然后重装
yum -y install --disablerepo="*" --enablerepo="elrepo-kernel" kernel-lt-headers 

# 检查 /boot/grub2/grub.cfg ，查看 ### BEGIN /etc/grub.d/10_linux ### 之下是否多出了新安装的内核
awk -F\' '$1=="menuentry " {print i++ " : " $2}' /etc/grub2.cfg
# 修改/etc/default/grub，GRUB_DEFAULT=1

# 执行
grub2-mkconfig -o /boot/grub2/grub.cfg

```

# 离线升级

```shell
# https://elrepo.org/linux/kernel/el7/x86_64/RPMS/ 提供了离线的RPM包下载

# 可以使用yum localinstall -y localinstall xxx.rpm 安装
```