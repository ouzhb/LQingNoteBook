---
title: Prometheus API
categories: "笔记"
date: 2019-07-23
comments: true
toc: true
tags:
	- Prometheus
---

Prometheus 调研


<!--more-->

# Client API

核心对象：

- Collector：收集器，根据Metrics的类型
- CollectorRegistry：Collector在其中进行注册

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
from prometheus_client import start_http_server, Summary
import random
import time

"""
创建一个Summary类型的指标，Summary是一个Collector对象;
    1. 第一个参数是metric的name
    2. 第二个参数是metric的Help信息
    3. 第三个参数是指标的label名称
"""
REQUEST_TIME = Summary('request_processing_seconds', 'Time spent processing request', ["program"])
REQUEST_TIME_WITH_LABEL = REQUEST_TIME.labels(program="Test") # 在指定的label上赋值


# 通过装饰器的方式对process_request的执行时间进行采样
@REQUEST_TIME_WITH_LABEL.time()
def process_request(t):
    """A dummy function that takes some time."""
    time.sleep(t)


if __name__ == '__main__':
    # 创建http服务，将metric暴露给采集器
    start_http_server(6789)
    # Generate some requests.
    while True:
        process_request(random.random())

```
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from prometheus_client import start_http_server, Gauge, CollectorRegistry, push_to_gateway

# 监控指标需要添加的Label
LABELS = {}

# 需要进行监控的指标
SERVICE_MAP = {}

# Metrics_MAP
Metrics_MAP = {}
Registry = CollectorRegistry()

def init_metric():
    """
    初始化量测值
    :return:
    """
    for key in SERVICE_MAP.keys():
        gauge_metric = Gauge("memory_usage_" + key.lower(), key + " service use memory", set(LABELS.keys()),registry=Registry)
        Metrics_MAP.update({key: gauge_metric})


def update_metric(sample):
    """
    更新测量值
    :param sample:
    :return:
    """
    for key in sample.keys():
        gauge_metric = Metrics_MAP.get(key)
        gauge_metric.labels(**LABELS).set(sample[key])


if __name__ == "__main__":

    init_metric()
    if exporter_config.get("http_endpoint"):
		# 创建一个http服务暴露采样信息
        start_http_server(exporter_config.get("http_port"), registry=Registry)
		while True:
        	sample = metric() # metric()获取采样值
        	update_metric(sample=sample)
        	time.sleep(30)

    elif exporter_config.get("push"):
		# Gateway发送采样信息
        push_to_gateway(exporter_config.get("gateway"),
            job=exporter_config.get("job"),
            grouping_key=LABELS,
            registry=Registry)
    else:
        exit(-1)



```

# Push Gateway


Push Gateway的主要用于Batch Job、网络隔离等场景的数据采集。

- Gateway不会Cache目标的采样数据，只是将即时数据暴露给prometheus，可以将最近一次的采样数据保存到文件中，用来在Gateway重启时恢复数据。
- 对于一些分布式计数的需求GateWay无法实现（可以使用[weaveworks/prom-aggregation-gateway](https://github.com/weaveworks/prom-aggregation-gateway)）。
- Gateway 没有实现TimeOut或者TTL机制。

Gateway 中采样的URL地址和label地址相关，格式为:
```
{IP}:{Port}/job/{job_name}/instance/{instance_value}/{label_1}/{value_1}/...
```
label相同的采样被合并为一个Group。

## timestamps

使用Gateway时，metrics中的timestamps会产生歧义，即：Client推送到Gateway的时间和Prometheus从Gateway获得sample的时间。

当Prometheus超过5min无法从target获取采样数据或者得到新采样（只时间变动的采样）时，认为target出现故障或者不存在。为了避免上述问题，Prometheus从Gateway得到的采样信息，以实际抓取的时间为timestamp，而push时间被保存在一个独立的metric中（push_time_seconds ）。


## Job 和 instance 标签

当prometheus从gateway中获取数据时会将sample中的job和instance标签配置为Gateway服务对应的值，因此需要在配置honor_labels为true。

```shell

# 使用curl发送采样到gateway

# 发送一个untyped类型的采样，对应到的Group为{job="some_job"}
echo "some_metric 3.14" | curl --data-binary @- http://pushgateway.example.org:9091/metrics/job/some_job

# 发送两个采样到gateway，Group为{job="some_job",instance="some_instance"}
  cat <<EOF | curl --data-binary @- http://pushgateway.example.org:9091/metrics/job/some_job/instance/some_instance
  # TYPE some_metric counter
  some_metric{label="val1"} 42
  # TYPE another_metric gauge
  # HELP another_metric Just an example.
  another_metric 2398.283
  EOF

# 删除指定group

curl -X DELETE http://pushgateway.example.org:9091/metrics/job/some_job/instance/some_instance

```


# 参考

[Pushgateway Java API](https://prometheus.github.io/client_java/io/prometheus/client/exporter/PushGateway.html)

[Pushgateway Python API](https://github.com/prometheus/client_python#exporting-to-a-pushgateway)

[Pushgateway Go API](https://prometheus.io/docs/instrumenting/pushing/)

[Pushgateway Ruby API](https://github.com/prometheus/client_ruby#pushgateway)