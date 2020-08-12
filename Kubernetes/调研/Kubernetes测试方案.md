# Kubernetes的性能指标

由于Kubernetes本身是管理平台，性能指标一般由**网络和存储插件**体现。

能够在K8S上直接体现的性能指标，一般只有以下几种：

- 99%Pod启动时间
- 99%API调用延时
- 集群容量：集群中Node和Pod数量的上限
- 集群负载：当前Pod总数/Pod容量

上述指标的测试，一般是在K8S集群不同负载水位线上（0%，50%，90%）时，使用测试工具反复对Pod/Deployment等工作负载进行创建、删除、扩缩容等核心操作。

常见的测试工具是[Kubemark](https://stupefied-goodall-e282f7.netlify.app/contributors/devel/kubemark-guide/)。

Kubemark 可以模拟出一个集群，这个集群中Master是真实的机器，所有的nodes是Hollow nodes。Hollow Node 具有完整的API接口，会对Master节点做出相应，但是不会真正创建Pod。通过 Kubemark，用户可以使用少量机器对 Kubernetes 控制平面的（Master节点）性能进行压力测试（官方文档中指出这种测试是e2e测试）。

# 网络性能测试

## 1. 测试虚拟机的网络性能

衡量虚拟机的网络性能指标和物理机类似，包括以下指标：

- 网络延时

- 网络实际带宽
- 网络PPS：PPS指每秒收发的TCP/UDP包的数量

上述三个指标中，网络延时使用```ping```命令就可以获得，但由于目标地址的不同比较难量化比较，因此衡量网络性能的主要指标还是**网络实际带宽**和**网络PPS**。

### 测试实际带宽

使用 iperf、iperf3、qperf 工具可以测试两个主机之间的实际带宽，以及带宽的抖动情况。

下面的命令以 iperf 为例，测试主机之间的带宽。

```shell
# 服务端，客户端同时安装 iperf
yum -y install iperf
# 服务端启动iperf 进程
iperf -s -i 1
# 客户端连接 iperf 发送tcp包测量带宽
iperf -c <ip_server>
```
服务端可以得到以下输出，其中Transfer是时间段内传输的字节数，Bandwidth 是该时间内的平均带宽。

```
------------------------------------------------------------
Server listening on TCP port 5001
TCP window size: 4.00 KByte (default)
------------------------------------------------------------
[  4] local 172.24.33.77 port 5001 connected with 172.29.55.234 port 34850
[ ID] Interval       Transfer     Bandwidth
[  4]  0.0- 1.0 sec  2.15 MBytes  18.0 Mbits/sec
[  4]  1.0- 2.0 sec  1.26 MBytes  10.5 Mbits/sec
[  4]  2.0- 3.0 sec   960 KBytes  7.87 Mbits/sec
```

### 测试主机的PPS

通过 **netperf** 和**sar**工具可以得到服务器的极限PPS，测试过程同样是**Server/Client**模式，其中Server是被测试机器。

```shell
# Server 和 Client 机器同时安装
yum -y install sysstat
rpm -ivh http://repo.iotti.biz/CentOS/7/x86_64/netperf-2.7.0-1.el7.lux.x86_64.rpm
# Server 启动 netperf 服务端
netserver -D -4
# Client 启动多个 netperf 发包进程
netperf -H 172.29.55.235 -l 300 -t TCP_RR -- -r 38,38 &
# 在Server一侧执行以下命令，观察网卡的PPS
sar -n DEV 2
```
下面的输出列出了每张网卡的收/发包速度， rxpck/s 表示每秒收包数，txpck/s 表示每秒发包数
```
Linux 3.10.0-957.el7.x86_64 (lqdocker) 	08/11/2020 	_x86_64_	(16 CPU)

03:59:46 PM     IFACE   rxpck/s   txpck/s    rxkB/s    txkB/s   rxcmp/s   txcmp/s  rxmcst/s
03:59:48 PM      eth0  11225.50  11336.50   1140.19   1151.03      0.00      0.00      0.00
03:59:48 PM     veth1      1.00      1.00      0.06      0.87      0.00      0.00      0.00
03:59:48 PM        lo      0.00      0.00      0.00      0.00      0.00      0.00      0.00
03:59:48 PM      br-0      1.00      1.00      0.05      0.87      0.00      0.00      0.00
03:59:48 PM   docker0      0.00      0.00      0.00      0.00      0.00      0.00      0.00
```

## 2. 测试Kubernetes的网络性能

Kubernetes的网络性能主要取决于**网络插件**。当Kubernetes在虚拟机中部署时，网络性能同时又受到**IaaS层网络**的影响。

主要指标包括：

- 网络吞吐量（带宽）
- 网络响应时延

根据访问方式的不同，Kubernetes的网络性能测试有以下几种场景：

- Kubernetes集群内宿主机和Pod通信
- Kubernetes集群内Pod和Pod之间通信
- Kubernetes集群内Pod和Service通信
- Kubernetes集群外部通过Ingress、NodePort、外部负载均衡服务范围服务

# 磁盘性能测试

Kubernetes 容器/虚拟机，测试磁盘性能的主要指标都是以下几个：

- 顺序读/写吞吐量
- 随机读/写QPS

通常测试磁盘性能可以使用fio工具，测试方式网上百度即可以。



# 测试结果

见：

# 参考

[Kubernetes集群性能测试](https://supereagle.github.io/2017/03/09/kubemark/)

[Linux系统中网络PPS值测量](https://www.ipcpu.com/2017/07/linux-network-pps/)

[使用iPerf进行网络吞吐量测试](https://www.jianshu.com/p/15f888309c72)

[kubernetes-性能测试方法简介](https://itnext.io/benchmark-results-of-kubernetes-network-plugins-cni-over-10gbit-s-network-36475925a560)