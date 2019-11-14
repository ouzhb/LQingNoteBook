---
title: Yarn磁盘管理
categories: "大数据" 
date: 2019-10-17
comments: false
toc: true
tags:
	- Yarn
---


<!--more-->
# Yarn磁盘管理

Yarn中NodeManager使用的最重要的两个本地目录：

- yarn.nodemanager.local-dirs：Container的缓存目录
- yarn.nodemanager.log-dirs：保存容器日志的本地目录，每个nodemanager都会有这样一个目录，目录格式为：application_${appid}/container_{$contid}，其中包含：

	- stderr
	- stdout
	- prelaunch.err
	- prelaunch.out
	- container-localizer-syslog

通常情况下，Yarn会自动清理这两个目录的日志，涉及到下面的二个参数，日志保留时间为二个参数的时间之和：

- yarn.nodemanager.delete.debug-delay-sec：App完成后DeletionService会在指定时间后删除本地日志。默认值为0
- yarn.nodemanager.log.retain-seconds：保留日志时间，只有当日志聚合关闭时有效。默认值为3小时

Yarn支持HDFS上的日志聚合功能，逻辑为：Application任务运行时日志写在log-dirs，运行完成以后 DeletionService 服务，把日志移动到HDFS上，然后删除本地日志。

日志聚合涉及的配置包括：

- yarn.log-aggregation-enable：是否开启日志聚合
- yarn.nodemanager.remote-app-log-dir：聚合根目录
- yarn.nodemanager.remote-app-log-dir-suffix：聚合目录，hdfs上日志保存位置为{yarn.nodemanager.remote-app-log-dir}/${user}/{yarn.nodemanager.remote-app-log-dir-suffix}
- yarn.log-aggregation.retain-seconds：聚合日志保留时间
- yarn.log-aggregation.retain-check-interval-seconds：清理任务运行时间间隔

Yarn支持对local-dirs和log-dirs进行健康检查，相关配置为yarn.nodemanager.disk-health-checker.XXXXXX：

- min-healthy-disks：log-dir/local-dirs健康目录的最小值，如果低于这个值，nn会被剔除
- max-disk-utilization-per-disk-percentage：监控log-dir和local-dirs的使用空间阈值，高于这个值磁盘被标记成不健康。
- disk-utilization-watermark-low-per-disk-percentage：bad状态目录恢复为可用的空间水线
- min-free-space-per-disk-mb：健康目录的最小剩余空间