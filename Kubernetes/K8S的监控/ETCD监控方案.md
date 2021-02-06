# ETCD 监控方案

创建 etcd 证书
```shell 

# 测试服务可用性
curl -Lk --cert /etc/etcd/healthcheck-client.crt --key /etc/etcd/healthcheck-client.key  https://localhost:2379/metrics	  
curl -Lk --cert /etc/etcd/healthcheck-client.crt --key /etc/etcd/healthcheck-client.key  https://localhost:2379/health

kubectl  create secret tls etcd-healthcheck-client --cert=/etc/etcd/healthcheck-client.crt --key=/etc/etcd/healthcheck-client.key --namespace prometheus
```

修改prometheus配置挂载证书到容器（PS：Helm部署普米时有独立的配置可以挂在，不用手工修改）
```yaml
  extraSecretMounts:
     - name: etcd-healthcheck-client
       mountPath: /etc/secrets/etcd-healthcheck-client
       subPath: ""
       secretName: etcd-healthcheck-client
       readOnly: true
```

配置job

```yaml
      - job_name: etcd
        scheme: https
        metrics_path: /metrics
        tls_config:
          cert_file: "/etc/secrets/etcd-healthcheck-client/tls.crt"
          key_file: '/etc/secrets/etcd-healthcheck-client/tls.key'
          insecure_skip_verify: true
        static_configs:
        - targets:
          - 172.24.135.72:2379
          - 172.24.135.73:2379
          - 172.24.135.74:2379
        scrape_interval: 1m
        scrape_timeout: 30s
```
etcd 部署在Master节点时
```yaml
      - job_name: etcd
        scheme: https
        metrics_path: /metrics
        tls_config:
          cert_file: "/etc/secrets/etcd-healthcheck-client/tls.crt"
          key_file: '/etc/secrets/etcd-healthcheck-client/tls.key'
          insecure_skip_verify: true
        kubernetes_sd_configs:
        - role: node
        relabel_configs:
          - source_labels:  ["__meta_kubernetes_node_labelpresent_node_role_kubernetes_io_master"]
            regex: "true"
            action: keep
          - regex: (.*)
            replacement: ${1}:2379
            source_labels:
            - __meta_kubernetes_node_address_InternalIP
            target_label: __address__
          - action: labelmap
            regex: __meta_kubernetes_node_label_(.+)
        scrape_interval: 1m
        scrape_timeout: 30s
```

异常告警配置
```yaml
略
```
更新prometheus配置
```shell
curl -XPOST http://prometheus.k8s.sdp.nd/-/reload
```