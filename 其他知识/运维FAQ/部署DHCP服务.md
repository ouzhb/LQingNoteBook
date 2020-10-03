# DHCP服务

在 Centos7 上部署DHCP服务器有以下限制，DHCP 服务工作时要求：DHCP服务器与客户端应该在同一个物理网段内。

DHCP Server 可以使用以下三种方式分配，IP地址：

- 自动分配方式（Automatic Allocation），DHCP服务器为主机指定一个永久性的IP地址，一旦DHCP客户端第一次成功从DHCP服务器端租用到IP地址后，就可以永久性的使用该地址。
- 动态分配方式（Dynamic Allocation），DHCP服务器给主机指定一个具有时间限制的IP地址，时间到期或主机明确表示放弃该地址时，该地址可以被其他主机使用。
- 手工分配方式（Manual Allocation），客户端的IP地址是由网络管理员指定的，DHCP服务器只是将指定的IP地址告诉客户端主机。

```shell script

yum -y install dhcp
cp /usr/share/doc/dhcp-4.2.5/dhcpd.conf.example /etc/dhcp/dhcpd.conf

```

修改配置文件，添加下面的配置：

```

option domain-name "example.org";
option domain-name-servers ns1.example.org, ns2.example.org;

default-lease-time 600;
max-lease-time 7200;

log-facility local7;

subnet 172.24.33.0 netmask 255.255.255.0 {

   range 172.24.33.110 172.24.33.200;
   option routers 172.24.33.1;
   option domain-name-servers 114.114.114.114;
   option broadcast-address 172.24.33.255;
   default-lease-time 300;
   max-lease-time 7200;
}
```

# DNS 

