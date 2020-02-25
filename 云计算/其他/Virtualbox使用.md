# 安装

```shell

# 可以下载好RPM包后用yum localinstall安装

wget http://download.virtualbox.org/virtualbox/debian/oracle_vbox.asc
rpm --import oracle_vbox.asc
wget http://download.virtualbox.org/virtualbox/rpm/el/virtualbox.repo -O /etc/yum.repos.d/virtualbox.repo
yum install VirtualBox-4.3

# 安装扩展包，以便使用RDP协议3389远程登录（即使用windows的远程登录）
wget http://download.virtualbox.org/virtualbox/4.3.2/Oracle_VM_VirtualBox_Extension_Pack-4.3.2-90405.vbox-extpack
VBoxManage extpack install Oracle_VM_VirtualBox_Extension_Pack-4.3.2-90405.vbox-extpack
```

# 创建虚拟机

```shell
# 创建虚拟机
VBoxManage createvm --name centos7 --ostype Linux_64 --register --basefolder /opt/virtualbox/
# 创建磁盘文件
VBoxManage createvdi --filename /opt/virtualbox/centos7.vdi --size 150000
# 创建虚拟机磁盘控制文件
VBoxManage storagectl centos7 --name storage_controller_1 --add ide
# 挂载光驱和磁盘控制文件
VBoxManage storageattach centos7 --storagectl storage_controller_1 --type hdd --port 0 --device 0  --medium /opt/virtualbox/centos7.vdi
VBoxManage storageattach centos7 --storagectl storage_controller_1 --type dvddrive --port 1 --device 0 --medium /opt/setup/CentOS-7.3-x86_64-LiveCD.iso

# 设定顺序
VBoxManage modifyvm centos7 --boot1 dvd
VBoxManage modifyvm centos7 --boot2 disk

# 创建桥接网络
VBoxManage modifyvm centos7 --nic1 bridged --cableconnected1 on --nictype1 82540EM --bridgeadapter1 <物理机桥接网卡地址> --intnet1 brigh1 --macaddress1 auto
# 创建nat网络
VBoxManage modifyvm centos7 --nic1 natnetwork --cableconnected1 on --nictype1 82540EM  --macaddress1 auto

# 设定计算资源
VBoxManage modifyvm centos7 --memory 2048
VBoxManage modifyvm centos7 --cpus 2

# 启动rdp模块
VBoxManage modifyvm centos7 --vrde on/off

# 后台启动虚拟机
VBoxManage startvm centos7 --type headless

# 关闭虚拟机
VBoxManage controlvm centos7 poweroff
```

# 常用命令

```shell
# 查看当前虚拟机
VBoxManage list vms
# 后台启动虚拟机
VBoxManage startvm centos7 --type headless
# 关闭虚拟机
VBoxManage controlvm centos7 poweroff
# 其他常用操作
VBoxManage controlvm pause|resume|poweroff|savestate centos7

# 添加nat转发

# 22 映射到宿主2222，协议有TCP、UDP
VBoxManage modifyvm "centos7" --natpf1 "guestssh,tcp,,2222,,22"
VBoxManage modifyvm "centos7" --natpf1 "guestidata,tcp,,18000,,8000"
```


# 参考

[安装包下载](http://download.virtualbox.org/virtualbox/)
[VBoxManage命令手册](https://www.virtualbox.org/manual/ch08.html)
[Centos6上安装](https://imxylz.com/blog/2014/08/14/install-virtualbox-in-command-line-of-centos-6/)