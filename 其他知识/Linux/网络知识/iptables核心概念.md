# 1. 基础概念

iptables 是Linux安全框架 **netfilter** 的代理客户端，通过 iptables 命令用户可以控制主机的网络流量，实现“网络地址转换”，“数据包过滤”，“数据包内容修改”等防火墙功能。

由于tcp/ip协议栈运行在内核空间，因此 netfilter 同样在系统内核中。

## 1. Chain（链）

链是数据包从源到达目的地时，经过的内核模块链，iptables包含以下Chain：

- PREROUTING 

- POSTROUTING

- INPUT

- OUTPUT

- FORWARD 

![数据包流经的chain](https://github.com/LinQing2017/LQingNoteBook/blob/master/images/iptables/chain.png)

## 2. 表

相同功能的规则集合叫做“表”，iptables已经定义了下面4种表：

- FILTER:：过滤功能

- NAT：网络地址转换功能

- MANGLE：解析报文，并重新封装

- RAW：关闭NAT表上启用的连接追踪机制？？？

不同的链可以保存不同类型表的规则，链和表的关系如下图（注意，同一条链中表的规则执行时具有优先级）：

![chain和表的关系](https://github.com/LinQing2017/LQingNoteBook/blob/master/images/iptables/chain2table.png)

## 3. 规则

表中的每一条记录称为规则，一条规则包含以下部分：

- 匹配条件：包括基本匹配条件，扩展匹配条件。

    - Source IP

    - Destination IP
    
    - Source Port / Destination Port

- 处理动作（target）：达成匹配条件时，进行的操作。

    - ACCEPT：允许数据包通过。

    - DROP：直接丢弃数据包，不给任何回应信息。

    - REJECT：拒绝数据包通过，必要时会给数据发送端一个响应的信息。

    - SNAT：源地址转换，解决内网用户用同一个公网地址上网的问题。

    - MASQUERADE：是SNAT的一种特殊形式，适用于动态的、临时会变的ip上。

    - DNAT：目标地址转换。

    - REDIRECT：在本机做端口映射。

    - LOG：在/var/log/messages文件中记录日志信息，但是不做任何操作。

通常情况，每条链对不匹配到任何规则的包有一个默认target，一般情况下为ACCEPT，即允许任何包通过。


# 2. 常用命令

```shell
## 查看表filter在所有链上的规则，其中-t默认情况下指定的表是filter
iptables -t filter -nvL 

## 查看指定表、指定链的规则
iptables -t filter -nvL INPUT

```

# 参考

- [iptables详解](http://www.zsythink.net/archives/tag/iptables/page/2/)