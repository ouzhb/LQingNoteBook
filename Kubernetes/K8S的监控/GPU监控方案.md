# 参考

[dashboard](https://grafana.com/grafana/dashboards/12239)
[dcgm-exportor](https://github.com/NVIDIA/gpu-monitoring-tools)

其中，dcgm-exportor修改以下配置：

- serviceMonitor.enabled=false ，这个貌似是普米Operator的配置
- 使用宿主机网络
- arguments: ["null"]，原有配置部分网卡会出现兼容性异常
- 挂载宿主机目录：/var/lib/kubelet/pod-resources，如果改了kubelet的工作目录，请修改ds文件

普米配置文件
```yaml
      - bearer_token_file: /var/run/secrets/kubernetes.io/serviceaccount/token
        job_name: dcgm-exporter
        kubernetes_sd_configs:
        - role: node
        relabel_configs:
        - source_labels:  ["__meta_kubernetes_node_label_nvidia"]
          regex: "enabled"
          action: keep
        - regex: (.*)
          replacement: ${1}:9400
          source_labels:
          - __meta_kubernetes_node_address_InternalIP
          target_label: __address__
        - replacement: /metrics
          target_label: __metrics_path__
        - action: labelmap
          regex: __meta_kubernetes_node_label_(.+)
        scheme: http
        tls_config:
          ca_file: /var/run/secrets/kubernetes.io/serviceaccount/ca.crt
          insecure_skip_verify: true
        scrape_interval: 30s
        scrape_timeout: 15s
```