---
title: HDFS 的块检查
categories: "大数据" 
date: 2019-03-10
comments: false
toc: true
tags:
	- HDFS
	- 安全
---

简单说明HDFS DataNode 在运行过程中执行的：block scanner, volume scanner, directory scanner, disk checker

<!--more-->


# DataNode

DataNode在运行过程中，通过不同的扫描机制处理以下问题：

- 何时/如何检查文件的完成性；
- 如何保证内存中的metadata信息和实际的一致性；
- 处理读取块时候的IO异常；

## Block Scanner && Volume Scanner

block的checksum信息，这些信息和block文件放在相同的目录，文件名为 { block-id }-xxxx.meta。这些meta文件用来检查块的完整性。对于finalized状态的block，在hdfs上应该包括block文件+meta文件。

Block Scanner是为了检查block文件是否存在损坏。

DataNode在进行Block Scanner时，针对每个Volume会启动一个检查线程，这个检查线程称为Volume Scanner。Volume Scanner 负责读取自己磁盘目录下的所有block，并且进行校验码计算，将计算结果和meta文件中的值比较来判断块文件是否完整。

通常情况下，block scanner要撸一便整个DN下的所有块文件，会发生巨大的IO。DN通过以下机制在节约IO带宽的同时，尽快发现坏块：

- DN在惊醒Block Scanner时将块分为：可疑块和常规块。可疑块指client（用户和其他DN）在读取过程中发生本地IO异常的块，这些块被DN放到队列中，在扫描时优先进行检查。

- 任意一个块被标记为可疑块，DN会立即检查这个块，并且跟踪这个块的状态10 min。之后块被移除可疑列表。

- 当可疑块的队列为空时，DN依次检查其他的块。当所有块检查完成以后，Volume Scannr 进入休眠状态，等待下一个Scanner周期。默认情况扫描周期间隔为3周（dfs.datanode.scan.period.hours）。

- 如果一个周期内无法扫描完成所有块，那么不会进入休眠，而是直接进入下一个周期。在休眠过程中一旦有块变成可疑块，那么扫描线程会立刻被唤醒。

- DN会将扫描的进度保存在scanner.cursor中，重启DN会从该文件恢复上次的扫描

- dfs.block.scanner.volume.bytes.per.second  可疑限制volume scan的带宽，默认为1m每秒


## Directory Scanners

DN会在内存中保存Block的位置信息（指的是block文件所处的目录），以及Block的状态信息。

DN 通过Directory Scanners保证内存位置信息和实际的一致性。当Directory Scanners发现block文件、meta文件丢失时，会将block标记为corrupted，DN在下一次汇报中，将这个块汇报给NN。Directory Scanners只检查finalized状态的block，并且DN开机后立即运行。

|参数|默认值|说明|
|---|---|---|
|dfs.datanode.directoryscan.throttle.limit.ms.per.sec|1000|扫描线程在一秒内允许运行的的毫秒数，默认为1000ms，即不限制|
|dfs.datanode.directoryscan.threads|1|扫描的线程数|
|dfs.datanode.directoryscan.interval|21600 |两次扫描的间隔时间，默认为6小时|

## Disk Checker

DN 检查HDFS用户是否有Volume目录下，finalized、tmp、rbw这三个目录的读写权限，并且只检查这三个目录，并不会检查子目录。

Disk Checker是hdfs中非常保守的检查机制，只有在DN进程操作block发现IOException时才会发生，并且执行一次只需要5~6s。

如果Volume在Disk Checker中失败，那么整个volume会被DN禁用，当volume Checker失败的Volume超过 dfs.datanode.failed.volumes.tolerated，那么DN会关闭。

# 心跳和Report

DN的心跳和Block Report，即向NN汇报block以及自身的各种信息，这些信息基于上述各种扫描的结果。

## 心跳

默认情况下DN 三秒发一次心跳，心跳的信息包括：磁盘容量，使用率等等基础信息。

## Block Report

分为增量Report 和 全量Report。Block Report 发送的内容包括：

- block ID
- generation stamp
- block在DN上的文件大小

心跳、block report的代码，均是实现在org.apache.hadoop.hdfs.server.datanode.BPServiceActor中。其中，BPServiceActor#offerService的offerService方法是DN的BP主循环，负责调用心跳（sendHeartBeat方法）、全量block Report（blockReport方法）。

同时，BPServiceActor中还实现了IncrementalBlockReportManager对象，BPServiceActor调用该对象的sendIBRs接口进行增量回报。

全量汇报会对NN和DN都产生压力，默认情况下当DN的Block超过100w时，DN会将报告分多次发送（该配置为dfs.blockreport.split.threshold）。当NN初次启动时，NN处理全量块汇报的时间长度，会影响HDFS的启动速度。



# Case

在生产环境的五节点环境中，间歇性发现DN出现Oom的现象。

调整JVM内存从 1G 到 2G DN进程不再推出，但是观察DN的内存使用情况发现每个6小时，DN的内存使用率有一个明显的尖峰。 由于该集群的小文件非常之多，总共有将近300w个块，平均每个DN节点需要管理180w个块文件。

由于内存尖峰的周期是6个小时，所以怀疑DN的以下两个操作（默认周期均是6小时）占用内存：

1. 6个小时一次的全量block report
2. 6个小时一次的Directory Scanner

以下两个方式验证：
1. 通过 hdfs dfsadmin -triggerBlockReport ip:50020 手工出发全量block report

2. 修改dn节点的Directory Scanners周期

结果发现，通过hdfs dfsadmin -triggerBlockReport触发全量汇报时，DN的内存没有任何波动。而改动Directory Scanners周期的DN，内存尖峰周期变为2个小时。

![oozie-err](https://github.com/LinQing2017/notes/blob/master/pictures/DirectoryScanner_GC.png?raw=true)

上述测试，可以验证Directory Scanner是造成DN内存上升的原因。

分析Directory Scanners 源码（org.apache.hadoop.hdfs.server.datanode.DirectoryScanner）发现，DN在进行Scanner时需要为每个Block创建一个ScanInfo对象，该对象的固定大小108byte，并且会创建大量临时File对象。

参考环境5156488块，平均每个DN节点309w个block（五节点3副本）。因此在执行Directory Scanner时需要额外申请318mb的内存，并且而外创建608w个File对象。

![oozie-err](https://github.com/LinQing2017/notes/blob/master/pictures/DirectoryScanner_mem.png?raw=true)


参考[HDFS-4461](https://issues.apache.org/jira/browse/HDFS-4461)发现，Directory Scanners在Hadoop 2.1时已经被优化过一轮。原先Scanner保存的是File对象而不是字符串。


该对象包含以下内容：

``` java
static class ScanInfo implements Comparable<ScanInfo> {

	// blockId 8byte
    private final long blockId;
	// block文件的相对地址，格式为subdir9/subdir9/blk_1074334048（随着block数目的增加subdir会多级嵌套，所以这里大小不定。固定是16bit*30/8=60byte）
    private final String blockSuffix;
	// meta文件名的后缀，格式为 blk_1074334048_593224.meta，后缀应当为_593224.meta（固定是17位16bit*12/8=24byte）
    private final String metaSuffix;
	// 引用对象保存一个指针 8byte
    private final FsVolumeSpi volume;
	// block 文件大小 8byte
    private final long blockFileLength;

	// Directory Scanner进行扫描时，需要创建临时File对象，获取block文件的句柄信息。
	File getBlockFile() {
      return (blockSuffix == null) ? null :
        new File(volume.getBasePath(), blockSuffix);
    }
	// Directory Scanner进行扫描时，需要创建临时File对象，获取block meta文件的句柄信息。
	File getMetaFile() {
      if (metaSuffix == null) {
        return null;
      } else if (blockSuffix == null) {
        return new File(volume.getBasePath(), metaSuffix);
      } else {
        return new File(volume.getBasePath(), blockSuffix + metaSuffix);
      }
    }

	......

```






# 参考

[HDFS DataNode Scanners and Disk Checker Explained](https://blog.cloudera.com/blog/2016/12/hdfs-datanode-scanners-and-disk-checker-explained/)

[ The Hadoop Distributed File System ](http://www.aosabook.org/en/hdfs.html)

[Directory Scanner 引发的GC案例](http://hadoop-common.472056.n3.nabble.com/HDFS-DirectoryScanner-is-bothering-me-td4165601.html)