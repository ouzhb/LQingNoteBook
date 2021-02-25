# Kubernetes 运维指标监控

以下内容介绍 SDP Kubernetes 集群在日常运维场景下围绕 Prometheus 实现的监控、告警方案。

## 1.机器指标

以下指标通过Node-Exporter和SDP-Exporter获取

| 监控内容                | 采集数据源                        | 告警阈值                     |
| ----------------------- | ------------------------------- | ------------------------ |
| CPU 使用率              | [Node-Exporter](https://github.com/prometheus/node_exporter) | 85% |
| CPU 5min 平均负载 | [Node-Exporter](https://github.com/prometheus/node_exporter) | 128 |
| 磁盘IO使用率 | [Node-Exporter](https://github.com/prometheus/node_exporter) | 80% |
| 系统内存使用率 | [Node-Exporter](https://github.com/prometheus/node_exporter) | 90% |
| 数据盘使用率（/data 目录） | 自定义Prometheus采集器 | 80% |
| 宿主机网络联通性（Ping） | [Blackbox-Exporter](https://github.com/prometheus/blackbox_exporter) | 100ms |

## 2.关键服务拨测

在Master节点要拨测的服务：

| 服务名称                | 拨测地址                                                     | 拨测节点   | SSL                      |
| ----------------------- | ------------------------------------------------------------ | ---------- | ------------------------ |
| etcd                    | https://127.0.0.1:2379/healthy                               | K8S Master | etcd-healthcheck-client  |
| kube-apiserver          | https://127.0.0.1:6443/healthz                               | K8S Master | admin.conf 生成的SSL证书 |
| kube-controller-manager | https://127.0.0.1:10257/healthz                              | K8S Master | admin.conf 生成的SSL证书 |
| kube-scheduler          | https://127.0.0.1:10259/healthz                              | K8S Master | admin.conf 生成的SSL证书 |
| kube-proxy              | http://127.0.0.1:10249/healthz                               | 所有节点   |                          |
| kubelet                 | https://127.0.0.1:10250/healthz                              | 所有节点   | admin.conf 生成的SSL证书 |
| falcon-agent            | http://127.0.0.1:1988/health                                 | 所有节点   |                          |
| Harbor                  | https://wxext-registry.101.com/api/v2.0/ping<br />https://wxext-registry.101.com/api/v2.0/health | 长乐环境   |                          |
| SDP-K8S服务             | https://k8s-api.sdp.101.com/$waf/ping                        | 长乐环境   |                          |

## 3. 其他监控

- kube-flannel

- iptable规则检查
- 宿主机DNS检查

## 4. Prometheus 配置

略

# Granfana监控面板

长乐环境Granfana地址：http://grafana.k8s.sdp.nd ，当前包括以下监控面板：

- 机器资源监控：K8S所有机器硬件信息监控

- SDP-Exporter：服务拨测、数据目录监控、主机Ping时延

- ETCD：K8S ETCD集群监控

- GPU监控：GPU使用信息监控

- 集群监控/实例监控：Tomcat容器资源监控

Granfana监控面板如下图，每个面板都可以在左上角的**环境选择**下拉框中切换环境：

![image-20210224105849548](..\..\images\k8s\granfana-dashboard.png)



 