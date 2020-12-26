# 相关指标

监控Kubernetes集群需要采集的指标，包括：

- 基础设施的指标，即node的CPU、内存、网络等指标；
    - Kubelet 服务本身会采集所在节点的相关，endpoint为：/api/v1/nodes/$1/proxy/metrics
    - 一些额外的指标可以通过部署node-export获取
    
- 容器基础设施，即所有容器的CPU、内存、网络等指标；
    - Kubelet 服务本身集成了cAdvisor，用户可以直接通过Kubelet组件获取给节点上容器相关监控指标，endpoint为：/api/v1/nodes/$1/proxy/metrics/cadvisor
    
- Kubernetes组件指标，这些指标主要来自api-service
    - api-service：https://<cluster-ip>:6443/metrics

- Service、Endpoint、Ingress可用性的监控

- 用户应用指标


# Prometheus 服务发现

在监控K8S时，Prometheus通过```kubernetes_sd_configs```配置去动态发现集群中的target。

Prometheus还支持以下动态发现机制：
- ```azure/ec2/digitalocean/openstack/gce_sd_config```：动态发现不同平台的虚拟机
- ```consul_sd_config```：基于consul动态发现
- ```dockerswarm_sd_config```：动态发现services、tasks、nodes
- ```dns_sd_config```：基于DNS的动态发现（即根据服务器的DNS A, AAAA, SRV记录进行查询）
- ```file_sd_config```：基于文件的发现，文件里面是一些静态配置
- ```marathon_sd_config```：类似于K8S，是一种容器编排工具

Prometheus 动态发现的 target 含有以下 label （称为 Discovered Labels）：

- __address__ : 当前Target实例的访问地址<host>:<port>
- __scheme__ : 采集目标服务访问地址的HTTP Scheme，HTTP或者HTTPS
- __metrics_path__ : 采集目标服务访问地址的访问路径
- __param_<name> : 采集任务目标服务的中包含的请求参数
- __meta_xxx : 和服务相关的若干label

通常，以 __ 开头的 Discovered Labels 是在系统内部使用的，这些标签不会被写入到样本数据中，即样本数据中没有这些label。

## kubernetes_sd_configs

Prometheus 能够支持一下模式进行服务发现，包括：

- node：发现集群中的node的 kubelet 地址，如```__address__="172.29.55.234:10250"```
- service: 发现集群中所有Service的ip+端口，主要用来对服务进行黑盒的可用性检查
- pod：发现所有pod的ip+暴露端口，如果pod没有暴露端口，那么__address__只有ip信息，如果暴露了多个端口那么会形成多个target
- endpoint：发现所有endpoint的ip+端口，一个endpoint中可能包含了多个ip，每个ip都会被发现为一个target
- ingress：发现所有的ingress地址

上述所有的发现方式，都会在target中采集到资源的namespace、label、annotation等配置。


## relabel_config

relabel_config 发生在 Prometheus 获取 Discovered Labels 之后，从 target 拉数据之前。

这个过程会对 Discovered Labels 进行重写，有以下目的：

- 由 ```__address__``` 生成样本数据中的 ```instance``` 标签；
- 重写 ```__metrics_path__```，部分endpoint的url地址不是标准的，需要重写；
- 保留一些 Discovered Labels 到样本数据的 label 中；
- 每个scrable可以串行配置多个relable；
- relable 包含下面几个动作：
    - replace（重写值或者加入新值）
    - labelmap（重命名）
    - labelkeep/labeldrop（过滤target）
    

获取kubelete内置的cAdvisor数据：

```yaml
scrape_configs:
- bearer_token_file: /var/run/secrets/kubernetes.io/serviceaccount/token
  job_name: kubernetes-nodes-cadvisor
  kubernetes_sd_configs:
  - role: node
  relabel_configs:
  - action: labelmap
    regex: __meta_kubernetes_node_label_(.+)
  - replacement: kubernetes.default.svc:443  # 默认action，replace
    target_label: __address__
  - regex: (.+) # 默认action，replace
    replacement: /api/v1/nodes/$1/proxy/metrics/cadvisor
    source_labels:
    - __meta_kubernetes_node_name
    target_label: __metrics_path__
```

# 监控方案

## 原生 Prometheus Helm

原生 Helm 包主要通过配置文件进行相应的包管理。


# 参考

[prometheus-book](https://yunlzheng.gitbook.io/prometheus-book)

[relabing](https://yunlzheng.gitbook.io/prometheus-book/part-ii-prometheus-jin-jie/sd/service-discovery-with-relabel)
