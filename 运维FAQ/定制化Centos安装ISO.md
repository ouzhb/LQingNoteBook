# 定制RPM包


- 安装纯净的CentOS-7-x86_64-Minimal-2003.iso，作为系统RPM下载环境

- 使用 yum -y install/localinstall <xxx>  --downloadonly --downloaddir=/<packages-dir> 下载到指定目录

- 登录打包环境，执行以下命令

    ```shell
    # 安装打包工具
    yum -y install createrepo mkisofs isomd5sum rsync anaconda syslinux
    # 挂载基础镜像目录
    mount /dev/cdrom /media
    # 创建工作目录，同步必要文件
    mkdir /ISO
    /usr/bin/rsync -a --exclude=repodata/ /media/ /ISO/
    mkdir -p /ISO/repodata
 
    # 从RPM包下载机器同步附加的RPM包到 /ISO/Packages
    rsync -vazu  172.24.33.80:/root/centos7-vbox-packages /ISO/Packages/

    ```
- 编辑打包脚本

    ```shell
    # 编辑/ISO/isolinux/isolinux.cfg中hd:LABEL=后的字符串为镜像名称

    # cp 原始安装包中的comps.xml脚本
     cp /media/repodata/*-minimal-x86_64-comps.xml /ISO/comps.xml
    ```

- 文件中添加如下group，并且加入到environment中
    ```xml
        
    <group>
        <id>rg-idata-vagrant</id>
        <name>RG-iData-Vagrant</name>
	    <packagelist>
            <packagereq type="default">VirtualBox</packagereq>
            <packagereq type="default">vagrant</packagereq>
            <packagereq type="default">expect</packagereq>
            <packagereq type="default">net-tools</packagereq>
	        <packagereq type="default">rsync</packagereq>
        </packagelist>
    </group>
    <environment>
        <optionlist>
            <groupid>rg-idata-vagrant</groupid>
        </optionlist>
    </environment>
    ```
- 生成仓库文件

    ```shell
    cd /ISO && createrepo -g repodata/comps.xml ./
    ```

- 生成镜像

    ```shell
    cd /ISO && genisoimage -joliet-long -V CentOS7-RGiData -o /root/images/CentOS7-RGiData.iso -b isolinux/isolinux.bin -c isolinux/boot.cat -no-emul-boot -boot-load-size 4 -boot-info-table -R -J -v -cache-inodes -T -eltorito-alt-boot -e images/efiboot.img -no-emul-boot /ISO
    
    ```
# 自动化安装

通过编辑ks.cfg，并指定isolinux.cfg 中相关命令，可以实现定制化的自动安装

```shell
label linux
  menu label ^Install CentOS 7
  kernel vmlinuz
  append initrd=initrd.img inst.stage2=hd:LABEL=CentOS7-RGiData inst.ks=cdrom:/isolinux/ks.cfg quiet
```

# 升级ISO内核版本

# 参考

[定制个自己的CentOS7系统](https://o-my-chenjian.com/2017/11/20/DIY-A-CentOS7-System/)

[订制rpm包到Centos7镜像中](https://www.cnblogs.com/weibin1/p/10522024.html)