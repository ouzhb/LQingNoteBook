# 快速安装

升级内核、安装libvirt依赖

```shell
yum update -y 
yum install qemu libvirt libvirt-devel ruby-devel gcc qemu-kvm rsync -y
systemctl start libvirtd && systemctl enable libvirtd
```

安装vagrant、vagarnt-libvirt

```shell
yum -y localinstall vagrant_2.2.9_x86_64.rpm
vagrant plugin install vagrant-libvirt --plugin-clean-sources --plugin-source https://gems.ruby-china.com/ 

# 配置libvirt作为默认的虚拟化引擎
echo "export VAGRANT_DEFAULT_PROVIDER=libvirt" > /etc/profile.d/vagrant.sh 
source /etc/profile
```

# 常用命令

- vagrant init : 初始化一个Vagrantfile文件，xxx/xxx

- vagrant box：

    - list： 查看载入的镜像

    - add：添加载入的镜像。执行命令后，box镜像会被Copy到${user.home}/.vagrant.d/boxes/ 目录下

    ```shell
    # 从网络仓库载入
     vagrant box add centos/7 
    # 从本地文件载入
     vagrant box add centos/7  CentOS-7-x86_64-Vagrant-1611_01.LibVirt.box

    ```
- vagrant up/destroy


# 参考

[vagrant官网](https://www.vagrantup.com/intro)

[vagrant-libvirtd](https://github.com/vagrant-libvirt/vagrant-libvirt)