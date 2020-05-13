# 安装

```shell

# 下载安装证书

wget http://download.virtualbox.org/virtualbox/debian/oracle_vbox.asc
rpm --import oracle_vbox.asc

# 可以下载好RPM包后用yum localinstall安装 ps：需要注意centos6 最高只到5.2版本
wget http://download.virtualbox.org/virtualbox/rpm/el/virtualbox.repo -O /etc/yum.repos.d/virtualbox.repo
yum install VirtualBox-5.2

# 安装扩展包，以便使用RDP协议3389远程登录（即使用windows的远程登录），PS：要安装配套的扩展插件
wget http://download.virtualbox.org/virtualbox/5.2.36/Oracle_VM_VirtualBox_Extension_Pack-5.2.36.vbox-extpack     
VBoxManage extpack install Oracle_VM_VirtualBox_Extension_Pack-5.2.36-135684.vbox-extpack    
```

# 创建虚拟机

```shell
# 创建虚拟机
VBoxManage createvm --name centos7 --ostype Linux_64 --register --basefolder /opt/virtualbox/
# 创建磁盘文件
VBoxManage createvdi --filename /opt/virtualbox/centos7.vdi --size 50000
# 创建虚拟机磁盘控制文件
VBoxManage storagectl centos7 --name storage_controller_1 --add ide
# 挂载光驱和磁盘控制文件
VBoxManage storageattach centos7 --storagectl storage_controller_1 --type hdd --port 0 --device 0  --medium /opt/virtualbox/centos7.vdi
VBoxManage storageattach centos7 --storagectl storage_controller_1 --type dvddrive --port 1 --device 0 --medium /opt/setup/CentOS-7.3-x86_64-LiveCD.iso

# 设定顺序
VBoxManage modifyvm centos7 --boot1 dvd
VBoxManage modifyvm centos7 --boot2 disk

# 创建桥接网络
VBoxManage modifyvm centos7 --nic1 bridged --cableconnected1 on --nictype1 82540EM --bridgeadapter1 eth0 --intnet1 brigh1 --macaddress1 auto

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

# 关于NAT网络配置

virtualbox中有两种nat网络：

- NAT：默认的NAT网络，网段类似为 10.0.x.0，使用这个网络时，同网段的虚拟机网络是不通的。
- NATservice：用户创建的NAT网络。

PS： 无论用上面哪种方式，虚机内的ip要配置到相应网段。

```shell
# 虚拟机配置到默认nat网路，这种情况下虚拟机的网段是10.0.x.0/24 ，第一张网卡是10.0.2.0/24，第二张网卡是10.0.3.0/24依次类推
VBoxManage modifyvm centos7 --nic1 natnetwork --cableconnected1 on --nictype1 82540EM  --macaddress1 auto

# 22 映射到宿主2222，协议有TCP、UDP
VBoxManage modifyvm "centos7" --natpf1 "guestssh,tcp,,2222,,22"
VBoxManage modifyvm "centos7" --natpf1 "guestidata,tcp,,18000,,8000"
```

一些环境中使用默认NAT网段有些问题，因此建议此时可以自己创建nat服务，命令如下：

```shell
# 创建一个nat网络，并且打开dhcp功能
VBoxManage natnetwork add --netname natnet1 --network "192.168.15.0/24" --enable --dhcp on
# 将虚拟机添加到相应的nat网络
VBoxManage modifyvm centos7 --nic1 natnetwork --nat-network1 natnet1
# 创建端口转发（1022 --> 192.168.15.4:22）
VBoxManage natnetwork modify --netname natnet1 --port-forward-4 "ssh:tcp:[]:1022:[192.168.15.4]:22"
VBoxManage natnetwork modify --netname natnet1 --port-forward-4 delete ssh

# 列出当前所有nat网络
VBoxManage natnetwork list
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

```


# 参考

[安装包下载](http://download.virtualbox.org/virtualbox/)
[VBoxManage命令手册](https://www.virtualbox.org/manual/ch08.html)
[Centos6上安装](https://imxylz.com/blog/2014/08/14/install-virtualbox-in-command-line-of-centos-6/)

[VBoxManage网络配置](https://www.virtualbox.org/manual/ch06.html)