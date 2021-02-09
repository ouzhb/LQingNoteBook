# 监控体系
## 1.机器指标

以下指标通过Node-Exporter和SDP-Exporter获取

- CPU 使用率、CPU 负载
- 内存使用率
- 系统盘使用率、磁盘IO操作耗时占比
- 数据盘使用率
- 宿主机网络联通性

## 2.关键服务可以用性

以下服务使用blackbox-exporter进行拨测（http/https）

- etcd
- kube-apiserver（包括VIP）
- kube-controller-manager
- kube-scheduler
- harbor
- coredns
- falcon-agent
- sdp-paas-api

以下服务监控容器状态

- kube-flannel
- kube-proxy


## 3.其他服务

- iptable规则检查
- 宿主机DNS检查

# 告警体系

# 异常场景处理

## 1.节点故障

## 2. Harbor 仓库不可用

## 3. Nginx 故障

 