# 说明

blackBox-exporter 是 Prometheus 官方提供的拨测插件，能够提供 http(s)、dns、tcp、icmp 的监控数据采集。

# 配置文件

```yaml

```

# 对接Prometheus配置

```yaml
# 访问所有falcon node节点的falcon-agents
    - job_name: 'falcon-agents'
      bearer_token_file: /var/run/secrets/kubernetes.io/serviceaccount/token
      params:
        module: [http_2xx]
      metrics_path: /probe
      kubernetes_sd_configs:
        - role: node
      relabel_configs:
        - source_labels: [__meta_kubernetes_node_address_InternalIP]
          target_label: __param_target
          regex: (.+)
          replacement: $1:1988/health
        - target_label: module 
          replacement: http_2xx
        - source_labels: [__meta_kubernetes_node_address_InternalIP]
          target_label: instance
        - target_label: __address__
          replacement: prometheus-blackbox-exporter:9115
      scrape_interval: 5m
      scrape_timeout: 30s

    - job_name: 'kube-nodes'
      bearer_token_file: /var/run/secrets/kubernetes.io/serviceaccount/token
      params:
        module: [icmp]
      metrics_path: /probe
      kubernetes_sd_configs:
        - role: node
      relabel_configs:
        - source_labels: [__meta_kubernetes_node_address_InternalIP]
          target_label: __param_target
        - target_label: module 
          replacement: icmp
        - source_labels: [__meta_kubernetes_node_address_InternalIP]
          target_label: instance
        - target_label: __address__
          replacement: prometheus-blackbox-exporter:9115
      scrape_interval: 5m
      scrape_timeout: 30s
```

# 参考
[Github地址](https://github.com/prometheus/blackbox_exporter)
[Chart 地址](https://github.com/prometheus-community/helm-charts/tree/main/charts/prometheus-blackbox-exporter)