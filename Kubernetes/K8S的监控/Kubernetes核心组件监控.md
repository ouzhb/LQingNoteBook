# 监控体系
## 1.机器指标

以下指标通过Node-Exporter和SDP-Exporter获取

- CPU 使用率、CPU 负载
- 内存使用率
- 系统盘使用率、磁盘IO操作耗时占比
- 数据盘使用率
- 宿主机网络联通性，关机告警！

## 2.关键服务可以用性

以下服务使用blackbox-exporter进行拨测（http/https）

在Master节点要拨测的服务：

| 服务名称                | 拨测地址                        | SSL                      | 返回值 |
| ----------------------- | ------------------------------- | ------------------------ | ------ |
| etcd                    | https://127.0.0.1:2379/healthy  | etcd-healthcheck-client  |        |
| kube-apiserver          | https://127.0.0.1:6443/healthz  | admin.conf 生成的SSL证书 |        |
| kube-controller-manager | https://127.0.0.1:10257/healthz | admin.conf 生成的SSL证书 |        |
| kube-scheduler          | https://127.0.0.1:10259/healthz | admin.conf 生成的SSL证书 |        |
|                         |                                 |                          |        |

所有节点拨测的服务：

| 服务名称       | 拨测地址                        | SSL                      | 返回值 |
| -------------- | ------------------------------- | ------------------------ | ------ |
| falcon-agent   | http://127.0.0.1:1988/health    |                          |        |
| kube-proxy     | http://127.0.0.1:10249/healthz  |                          |        |
| kubelet        | https://127.0.0.1:10250/healthz | admin.conf 生成的SSL证书 |        |
|                |                                 |                          |        |

其他拨测的服务：

| 服务名称     | 拨测地址                    | SSL  | 返回值 |
| ------------ | --------------------------- | ---- | ------ |
| coredns      | http://<POD_ID>:8080/health |      |        |
| harbor       | http://<POD_ID>:8080/health |      |        |
| sdp-paas-api |                             |      |        |
|              |                             |      |        |
|              |                             |      |        |

以下服务监控容器状态

- kube-flannel



## 3.其他服务

- iptable规则检查
- 宿主机DNS检查

# 告警体系

# 异常场景处理

## 1.节点故障

## 2. Harbor 仓库不可用

## 3. Nginx 故障

 