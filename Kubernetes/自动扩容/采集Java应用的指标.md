# Spring Boot Actuator

Spring Boot Actuator 可以帮助用户监控和管理 Spring Boot 应用，用户可以通过连接 Java 进程 JMX 或者 HTTP Endpoints 来获取相关的信息。

PS： Actuator 有 2.x 和 1.x 两个版本，差距较大。 

```
implementation "org.springframework.boot:spring-boot-starter-actuator"
```

引入上述 Java 依赖后，通过下面配置可以暴露出 actuator 提供的默认端点。

```yaml
  # 所有 endpoint 在 http 上暴露
management:
  endpoints:
    web:
      exposure:
        include: "*"
  # health endpoint 暴露组件相关细节
  endpoint:
    health:
      show-details: always
```

## 1. Health Endpoint

Health 可以判断组件的健康状况。

访问 ```/actuator/health``` 可以拿到服务的整体健康情况，这个状态由多个子项构成，用户可以**自定义健康状态子项**或者使用**Actuator 原生提供的健康检查项**。

```yaml
{
    "status": "UP",                             # 服务整体健康情况
    "details": {
        "minioHealthCheck": {                   # 自定义的 Minio 服务状态检查 
            "status": "UP",
            "details": {
                "minioStatus": "OK"
            }
        },
        "diskSpace": {                          # 提供的磁盘空间检查 
            "status": "UP",
            "details": {
                "total": 107321753600,          # 这个貌似是 java 进程启动目录的大小
                "free": 77873278976,
                "threshold": 10485760
            }
        },
        "refreshScope": {
            "status": "UP"
        }
    }
}
```

### 使用K8S的健康检查

K8S 包括以下三种检查探针：

- livenessProbe：存活探针，如果存活探针失败，pod会被删除重建
- readinessProbe：判断容器是否处于可用Ready状态, 达到ready状态表示pod可以接受请求, 如果不健康， 从service的后端endpoint列表中把pod隔离出去
- startupProbe：启动探针，主要是为了解决一些慢启动应用而设计的探针。但程序未启动完成时，有startupProbe检查服务的可用性。一旦startupProbe通过，livenessProbe 就开始工作。

每种探针包括三种工作方式 HTTP、TCP、Exec：

- HTTP：200或300范围内的HTTP响应，它会将应用程序标记为健康，否则它被标记为不健康。
- Exec：在容器内执行一个命令，如果退出码为0，则容器标记为健康
- TCP：在指定端口可建立连接，即探针成功

## 2. Promethues Endpoint  

Spring 中引入以下依赖，可以为 Actuator 添加 Promethues Endpoint 。

```yaml
    implementation "org.springframework.boot:spring-boot-starter-actuator"
    implementation "io.micrometer:micrometer-registry-prometheus"
```

Endpoint 地址：```/actuator/prometheus```，此时采集到的主要是 JVM 的指标。

配置 Promethues 的服务发现：


```yaml
- job_name: goblin-pods
  kubernetes_sd_configs:
  - role: pod
  relabel_configs:
  - action: keep
    regex: goblin
    source_labels: [__meta_kubernetes_pod_label_component]
  - action: replace
    regex: (.+)
    replacement: $1
    source_labels: [__meta_kubernetes_pod_label_component, __meta_kubernetes_pod_label_app]
    separator: _
    target_label: application
  - action: labelmap
    regex: __meta_kubernetes_pod_label_(.+)
  - replacement: /httpd_v1/actuator/prometheus
    target_label: __metrics_path__
  scrape_interval: 5m
  scrape_timeout: 30s
```

### 添加自定义指标



# 参考

[Spring Boot Actuator:健康检查、审计、统计和监控](https://bigjar.github.io/2018/08/19/Spring-Boot-Actuator-%E5%81%A5%E5%BA%B7%E6%A3%80%E6%9F%A5%E3%80%81%E5%AE%A1%E8%AE%A1%E3%80%81%E7%BB%9F%E8%AE%A1%E5%92%8C%E7%9B%91%E6%8E%A7/#%E5%88%9B%E5%BB%BA%E4%B8%80%E4%B8%AA%E6%9C%89Actuator%E7%9A%84Spring-Boot%E5%B7%A5%E7%A8%8B)

[K8S 的探针检查机制](https://www.jianshu.com/p/5def7f934d2e)