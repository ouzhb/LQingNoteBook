# 1. 背景

- 基于UDP协议
- 核心部分：加密密钥路由
	- 公钥和 IP 地址列表（AllowedIPs）关联起来
	- 每一个wg接口都有一个私钥和一个 Peer 列表
	- 每一个 Peer 都有一个公钥和 IP 地址列表
- 发送包时，AllowedIPs起到路由表的功能
  - **正常情况下，Peer的每个AllowedIPs，应该填写一条静态路由到wireguard设备!!!**
- 接收包时，AllowedIPs起到权限管理的功能：Packet的Source IP位于服务端的 AllowedIPs 列表时被接收，否则被丢弃

# 2.安装

```shell
yum remove kernel-lt-*  kernel-tools kernel-tools-libs kernel-headers -y

rpm --import https://www.elrepo.org/RPM-GPG-KEY-elrepo.org
rpm -Uvh http://www.elrepo.org/elrepo-release-7.0-2.el7.elrepo.noarch.rpm
yum --enablerepo=elrepo-kernel install kernel-ml kernel-ml-* perf -y

grep "^menuentry" /boot/grub2/grub.cfg | cut -d "'" -f2
grub2-set-default <内核版本>
grub2-editenv list

reboot
```



```shell
yum install epel-release elrepo-release
yum install kmod-wireguard wireguard-tools #5.6以上内核时只要安装wireguard-tools即可

echo "net.ipv4.ip_forward = 1" >> /etc/sysctl.conf
echo "net.ipv4.conf.all.proxy_arp = 1" >> /etc/sysctl.conf
sysctl -p /etc/sysctl.conf
```



# 配置

## 1. 服务端配置

```shell
ip link add dev wg0 type wireguard
# 通过 IP 命令来指定 wg0 设备的IP，而不在配置文件中通过Address参数去指定（否则 wg setconf 会失败）
ip address add dev wg0 10.200.200.2/24  
```

生成key

```shell
mkdir -p /etc/wireguard
umask 077
wg genkey > /etc/wireguard/privatekey
wg pubkey < /etc/wireguard/privatekey > /etc/wireguard/publickey
```

添加以下IP规则（PS：有必要的话写入 /etc/sysconfig/iptables）

```shell
iptables -I INPUT -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
iptables -I FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
iptables -I FORWARD -i wg0 -o wg0 -m conntrack --ctstate NEW -j ACCEPT
iptables -t nat -I POSTROUTING -s 10.200.200.0/24 -o em1 -j MASQUERADE # 这一条规则很重要
iptables -I INPUT  -p udp -m multiport --dport 51820 -j ACCEPT
```

生成配置，并启动wg0网卡

```shell
cat << EOF > /etc/wireguard/wg0.conf
[Interface]
# Name = Ali_Server
ListenPort = 51820
PrivateKey = <Server_PrivateKey>
EOF

wg setconf wg0 /etc/wireguard/wg0.conf 
# 启动wg0 网卡
ip link set up dev wg0
```

# 场景1

服务端有公网IP，Client无固定IP，配置Client经过Serverr的全局代理

服务端配置
```ini
[Peer]
# Name = Home_Win10
AllowedIPs =  10.200.200.11/32 # Clinet 不配置代理
PublicKey = <Client_PublicKey>
```

客户端配置
```ini
[Interface]
# Name = Home_Win10
PrivateKey = <Client_PrivateKey>
Address = 10.200.200.11/24  # VPN 网络中该Client的IP

[Peer]
# Name = Home_Win10
PublicKey = <Server_PublicKey>
AllowedIPs = 0.0.0.0/1, 128.0.0.0/1, ::/1, 8000::/1
Endpoint = goblin.lqingcloud.cn:51820  # Server 公网连接端点
PersistentKeepalive = 25

```

# 场景2

在以下机器之间创建子网**10.200.200.0/24**，包含以下机器：

- 10.200.200.2 ：Ali 云虚拟机有公网IP，作为转发的Server
- 10.200.200.47：公司服务器（局域网所在网段为172.24.135.0/24）
- 10.200.200.11：家里的PC

实现：

- 使子网之间的VPN  IP 可以相互访问
- **10.200.200.11可以访问172.24.135.0/24的其他机器**

服务端配置：

```ini
[Peer]
# Name = ND_CL_Client
AllowedIPs =  10.200.200.47/32, 172.24.135.0/24
PublicKey = 略

# 中继服务器上要手动添加以下静态路由规则：
ip route add 172.24.135.0/24 via 0.0.0.0 dev wg0
# 或者以下方式添加
route add -net 172.24.135.0/24 netmask 255.255.255.0 wg0
```

公司机器配置

```ini
[Interface]
# Name = ND_CL_Client
ListenPort = 51820
PrivateKey = 10.200.200.47/24

[Peer]
PublicKey =  略
AllowedIPs =  10.200.200.0/24
Endpoint =  略
PersistentKeepalive = 25
```

家里PC配置

```ini
[Interface]
# Name = Home_Win10
PrivateKey =  略
Address = 10.200.200.11/24 # VPN 网络中该Client的IP

[Peer]
PublicKey =  略
AllowedIPs = 10.200.200.0/24,172.24.135.0/24
Endpoint =  略
PersistentKeepalive = 25
```

PS：上述配置只能10.200.200.0/24内的节点相互访问

# 场景3

将Kubernetes 和 WireGuard 结合的CNI插件：[Kilo](https://github.com/squat/kilo)，该插件有以下能力。

## 1. CNI网络插件

Kilo 可以替代Flannel、Calico等常规网络插件，为Pod提供集群内的网络IP，此种模式下我们可以实现：

- 将多个不同网络的机器，加入到同一个K8S中进行管理，而这些节点不一定需要有稳定的公网IP（PS：这种能力非常适合一些IoT、边缘计算场景）～～

限制条件：

- 所有节点必须安装WireGuard（建议直接将系统内核升级到5.6以上版本）
- 所有节点必须提供一个可访问的UDP端口（默认是51820）
- Kilo在不同**Region**之间创建网络，需要知道节点的确切位置。Kubernetes Node 可以使用 [topology.kubernetes.io/region](https://kubernetes.io/docs/reference/kubernetes-api/labels-annotations-taints/#topologykubernetesioregion)为每个Node打标签，或者为每个Node添加[kilo.squat.ai/location](https://github.com/squat/kilo/blob/master/docs/annotations.md#location) **annotation**
- 每个**Region**中的至少一个节点必须是其他Node可访问的，即**每个Region中要有一个公网IP**，可用用 [kilo.squat.ai/force-endpoint](https://github.com/squat/kilo/blob/master/docs/annotations.md#force-endpoint)指定这个endpoint的访问地址

关于Kilo的组网细节参考官方文档：[topology](https://github.com/squat/kilo/blob/master/docs/topology.md)

## 2. 附加网络工具

**PS：以下讨论Flannel为网络插件，并且以vxlan模式工作。**

Kilo为K8S网络提供功能扩展，参考以下Yaml文件安装。

```shell
kubectl apply -f https://raw.githubusercontent.com/squat/kilo/master/manifests/kilo-kubeadm-flannel.yaml 
```

Kilo在每台主机的宿主机网络运行，启动后会进行以下操作：

- 为每台宿主机创建一张wireguard，名称为kilo0

- 在每台宿主机上创建对应的iptables规则，**PS：但是宿主机上51820的UDP端口依然要手工创建Iptable规则**

- 为每台机器创建Key和配置文件，并且持久化在**/var/lib/kilo**目录

- **默认创建的VPN子网为10.4.0.0/16，可以通过-subnet指定**

- 根据**--mesh-granularity**配置，Kilo创建的网络拓扑有所不同：

  - location：这种模式下一个Region中只有一个Node工作，该节点称为Master用户可以手动指定，也可以由Kilo自动选择。

  - full：所有节点加入到VPN网络中，并且相互称为Peer。**此时kilo0设备会替代flannel.1设备，拦截所有发到POD IP的包，并根据WireGurad配置文件的allowedIPs信息发送到相应节点！**

    - 点击的Peer信息如下：

      ```ini
      interface: kilo0
        public key: 略
    private key: (hidden)
        listening port: 51820
  
      peer: 略
    endpoint: 172.24.135.45:51820
        allowed ips: 10.190.0.0/24, 10.190.0.0/32, 10.4.0.1/32
      
      peer: 略
        endpoint: 172.24.135.46:51820
        allowed ips: 10.190.2.0/24, 10.190.2.0/32, 10.4.0.2/32
      
      ```
    
    - 路由表的状态如下：
    
      ```shell
Kernel IP routing table
    Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
    default         172.24.135.254  0.0.0.0         UG    0      0        0 em1
    10.4.0.0        *               255.255.0.0     U     0      0        0 kilo0
    10.190.0.0      10.4.0.1        255.255.255.255 UGH   0      0        0 kilo0  # H 表示路由路由单个IP
    10.190.0.0      10.4.0.1        255.255.255.0   UG    0      0        0 kilo0  # G表示发送到网关
    10.190.1.0      *               255.255.255.0   U     0      0        0 cni0   # *表示本地Pod网络，直接发送包到K8S的CNI设备
    10.190.2.0      10.4.0.2        255.255.255.255 UGH   0      0        0 kilo0
    10.190.2.0      10.4.0.2        255.255.255.0   UG    0      0        0 kilo0
      ```
    
  
- 向所有K8S集群中的Master添加Peer，略！




## 3. 存在问题

- 修改kilo的配置会影响flannel的路由配置，此时需要重启flannel容器来恢复路由
- 区域路由模式时（**--mesh-granularity=location**），出现多个node绑定同一个VPN IP（原因可能是抢夺Leader时，没有清理设备信息）。
- 一些配置修改不方便需要通过Annotations：https://github.com/squat/kilo/blob/master/docs/annotations.md

## 4. 参考配置

作为附加网络工具时，实现场景二的功能，参考一下配置：

- ds配置（**不影响flannel原有网络**）:

  ```shell
      - --kubeconfig=/etc/kubernetes/kubeconfig
      - --hostname=$(NODE_NAME)
      - --cni=false
      - --compatibility=flannel
      - --local=false
      - --mesh-granularity=location 
      - --encapsulate=never # 永远不拦截flannel的流量
  ```

- ds配置nodeSelector只部署一个节点

- Node打对应标签

  ```shell
    annotations:
      kilo.squat.ai/endpoint: 172.24.135.45:51820                     # 自动生成的
      kilo.squat.ai/internal-ip: 10.190.0.0/32                        # 自动生成的
      kilo.squat.ai/key: ++8u2w7NPb5CzIk47IHTx11L3Dw9OzvRmPSzDkU+cQ0= # 这个是公钥
      kilo.squat.ai/last-seen: "1613143193"
      kilo.squat.ai/leader: "true"                                    # 可以不设置
      kilo.squat.ai/persistent-keepalive: "25"                        # 一定要设置
      kilo.squat.ai/wireguard-ip: 10.4.0.1/16                         # 自动生成，没有测试能不能手动指定
  ```

- Peer 配置

  ```yaml
apiVersion: kilo.squat.ai/v1alpha1
kind: Peer
metadata:
  name: goblin-peer
spec:
  allowedIPs:
  - 10.4.0.1/32  # k8s 上VPN 网络
  - 10.200.200.0/24 # 公网服务端的 VPN 网络
  endpoint:  # 公网服务端的endpoint
    dns: goblin.lqingcloud.cn
    port: 51820
  persistentKeepalive: 10 # 没有用不生效
  publicKey: 略
  ```

- 公网中继服务器配置

  ```shell
  [Peer]
  AllowedIPs =  10.4.0.1/32,172.24.135.0/24
  PublicKey = 
  ```

- 公网添加静态路由

  ```
  route add -net 172.24.135.0/24 netmask 255.255.255.0 wg0
  route add -net 10.4.0.0/16 netmask 255.255.255.0 wg0
```
  
  

# 参考

[WireGuard 教程：WireGuard 的工作原理](https://fuckcloudnative.io/posts/wireguard-docs-theory/)

[WireGuard 教程：WireGuard 的搭建使用与配置详解](https://fuckcloudnative.io/posts/wireguard-docs-practice/)

[kilo](https://github.com/squat/kilo)

[非官方文档](https://github.com/pirate/wireguard-docs)