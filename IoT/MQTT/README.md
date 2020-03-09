# MQTT

MQTT 是一种工作在 TCP/UDP 上的轻量级消息发布/订阅传输协议，有以下基本特点：

- 使用发布/订阅消息模式，提供一对多的消息发布。

- MQTT工作于应用层，是基于TCP/IP之上的可靠连接，当然也有UDP版本叫做“MQTT-SN”，通常使用的都是TCP版。

- 负载内容屏蔽的消息传输？？（指的是传输内容加密？）

- 包含以下服务质量（QoS）：

    - QoS0至多一次：消息发布完全依赖底层TCP/IP网络，会发生消息丢失或重复。这一级别可用于如下情况，环境传感器数据，丢失一次读记录无所谓，因为不久后还会有第二次发送。这一种方式主要普通APP的推送，倘若你的智能设备在消息推送时未联网，推送过去没收到，再次联网也就收不到了。
    - QoS1至少一次：确保消息到达，但消息重复可能会发生，这个是最常用的消息QoS1。
    - QoS2只有一次：确保消息到达一次，一般用在一些比较严格的计费系统中。这种最高质量的消息发布服务还可以用于即时通讯类的APP的推送，确保用户收到且只会收到一次。

- Topic 订阅树：MQTT通过树型结构的方式组织Topic（和Kafka差别巨大），Client可以精确订阅或者使用通配符订阅（[参考](http://www.bewindoweb.com/268.html)）
    
    - 精确订阅主题，如/mydevice/abc
    - 通配符主题，如/mydevice/+/confs、/mydevice/qwer/#

- 不同类型的消息：

    - 保留消息（Retain Message）：每个主题可以有一条保留消息，如果有新的订阅者订阅了该主题，就需要将这条保留消息推送给它，通常用于新订阅主题的设备初始化。
    - 遗嘱消息（Will Message）：基于客户端，每个客户端在连接的时候可以保留一条遗嘱消息，当连接异常断开（例如网络中断、客户端崩溃、心跳信息没有发送）没有发送正常的DISCONNECT报文的时候，会将这条消息分发给订阅了这个遗嘱消息的客户端，其他客户端就知道它发生问题了，从而做相应的处理，通常用于设备的异常通知，例如当设备正常退出，手机APP需要显示设备正常退出，当设备异常退出，手机APP需要报警，那么手机APP只需要订阅设备的遗嘱消息，就能够知道它断开时是正常的还是异常的了。

- MQTT服务器至少应该支持的MQTT3.1.1版本。MQTT3.1是早期版本，为了兼容性可以去支持。MQTT5.0是最新版本（3.1.1的下一版本是直接跳到5.0的），性能会有提升，如果设备也能采用MQTT5.0，那么服务器支持MQTT5.0是很好的决定，对于普通开发者，只需要支持MQTT3.1.1即可。

## MQTT Broker

当前IoT平台开源程度没有想象的高，付费的HIVEMQ、EMQ X几乎功能应有尽有，大部分开源的方案没有cluster方案，性能和付费的相比也相去甚远！

### 1. 选型需要考虑的特性

参考网上的一些博客，整理MQTT Broker选型需要注意的点:

- 基本需求：
    
    - 支持的MQTT 3.x协议，同时支持 MQTT 5 更好（5.0版本目前各大Broker都在努力支持）
    - 支持QoS0、QoS1（可选QoS2）
    - 支持保留消息，遗嘱消息（据说保留消息没有那么重要！！！）

- 持久化：
    
    - 部分Broker可以持久化消息到本地，保证消息传递的可靠性（比如，QoS1 持久到磁盘时能够保证Broker崩溃，消息不丢失）
    - 另一种持久化是将Broker的消息转储到其他系统，最常见的是Mongdb、Kafka。如果没有这个功能，我们可能需要一个额外的进程（比如Spark Streaming），来订阅Broker上的指定消息，并写入存储系统。

- 多种连接方式，通常MQTT连接包括以下方式：

    - MQTT over TCP
    - MQTT over Websocket：在Websocket之上做MQTT封装，对APP这种客户端来说很友好
    - MQTT over TCP/SSL
    - MQTT over Websocket/SSL

- 支持集群

- 支持自定义验证方式，包括：

    - CONNECT阶段验证（ClientID、Username、Password、IP四种方式验证，大部分开源的Broker都只支持Username和Password的验证）
    - PUBLISH、SUBSCRIBE的验证（个人认为这个属于权限管理了？？？）

- 支持共享订阅：多个客户端订阅同一个主题，消息只会被分发给其中的一个客户端。共享订阅主要针对的是需要客户端负载均衡的场景，比如后端服务多个Worker，需要共享订阅来只让一个Worker得到数据（这个功能实际上可以使用Kafka实现，在Broker中不是必须的）。

### 2. 不同Broker搭建方案的比较

主要参考MQTT官方给出的几个LIST进行筛选：[MQTT Server 功能列表](https://github.com/mqtt/mqtt.github.io/wiki/server-support)，[MQTT Broker List](https://github.com/mqtt/mqtt.github.io/wiki/brokers),[MQTT Broker介绍](https://github.com/mqtt/mqtt.github.io/wiki/servers)

- 百万/千万连接级别：

    - 开源方案：没有找到~~~！！！
    - 商业方案：[hivemq](https://www.hivemq.com/), [emqx](https://www.emqx.io/cn/)都在官方网站宣称支持1000w的客户端连接。其中，hivemq是国外的功能比较全面，生态完善，emqx是国产软件评价还不错

- 十万连接级别：部分比较优秀的开源方案可以，但是很多都没有集群方案，其他功能也比较匮乏！

    - emqx broker（开源版本）：支持集群，但是不能扩容，首选！
    - [emitter](https://github.com/emitter-io/emitter)：完全开源的集群方案，git上2.2k个star，第二选择！
    - [Mosquitto](https://mosquitto.org/): 轻量的broker，主要目的是运行在嵌入是设备中，有二次开发的集群版本[hui6075/mosquitto-cluster](https://github.com/hui6075/mosquitto-cluster)。
    - [moquette](https://moquette-io.github.io/moquette/):基于java，比较轻量的单机方案

- 其他方案：
    
    - 对一些AMQP添加插件，使他支持MQTT协议，当前AcitveMQ和Rabbit MQ都支持这么搞！但是AMQP场景和IoT场景的不太一样，可能需要进行一些性能测试来判断接入数目。
    - 直接买阿里云的[MQTT服务](https://cn.aliyun.com/product/mq4iot)(可以包月或者按需付费)


**总结**: 如果要用开源方案上生产环境，可选的可能只有 emqx broker 和 emitter （因为他们可以高可用）。


## 2. 相关工具

### 1. Spark 整合 MQTT Broker

[参考](http://bahir.apache.org/docs/spark/current/spark-sql-streaming-mqtt/)

### 2. MQTT Server 压力测试

压力测试工具：

- [emqtt-bench](https://github.com/emqx/emqtt-bench)



# 参考

[【MQTT协议介绍】](https://www.runoob.com/w3cnote/mqtt-intro.html)

[【MQTT协议中文版】](https://mcxiaoke.gitbooks.io/mqtt-cn/content/mqtt/01-Introduction.html)

[【MQTT中文网】](http://mqtt.p2hp.com/)

[【MQTT Broker 数据结构：订阅树】](http://www.bewindoweb.com/268.html)

[【MQTT Broker的需求】](http://www.bewindoweb.com/244.html)