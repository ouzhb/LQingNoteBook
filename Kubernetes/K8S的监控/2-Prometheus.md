---
title: Prometheus Alarm
categories: "笔记"
date: 2019-07-31
comments: true
toc: true
tags:
	- Prometheus
---

Prometheus 调研


<!--more-->

# Prometheus告警

告警能力在Prometheus的架构中被划分成两个独立的部分：告警规则(产生告警),告警处理。

![](https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-LBdoxo9EmQ0bJP2BuUi%2F-LVMF4RtPS-2rjW9R-hG%2F-LPS9QhUbi37E1ZK8mXF%2Fprometheus-alert-artich.png?generation=1546578333144123&alt=media)

- 告警规则：通过在Prometheus中定义AlertRule（告警规则），Prometheus会周期性的对告警规则进行计算，如果满足告警触发条件就会向Alertmanager发送告警信息。
- 告警处理：

## 告警规则

一条告警规则，包括以下几个部分：

- 告警名称：告警规则命名
- 告警规则：主要由PromQL进行定义，其实际意义为当表达式（PromQL）查询结果持续多长时间（During）后出发告警
- 告警组：对一组相关的告警进行统一定义，并通过YAML文件来统一管理。

典型告警规则定义：

```yaml
groups:
- name: example
  rules:
  - alert: HighErrorRate
    expr: job:request_latency_seconds:mean5m{job="myjob"} > 0.5 # 对Prometheus来说，产生的告警条数由PromQL返回的调数决定，返回几个采样值对应就产生几个告警。
    for: 10m
    labels:
      severity: page  # 附加到告警信息上的标签，可以添加额外的标签
    annotations:
      summary: High request latency #描述告警的概要信息
      description: description info #描述告警的详细信息
```

上述告警规则中，summary和description的值可以使用模板化的值，如$labels.\<labelname\>变量可以访问当前告警实例中指定标签的值，$value则可以获取当前PromQL表达式计算的样本值


在Prometheus全局配置文件中通过rule_files指定一组告警规则文件的访问路径，Prometheus启动后会自动扫描这些路径下规则文件中定义的内容，并且根据这些规则计算是否向外部发送通知。默认情况下Prometheus会每分钟对这些告警规则进行计算。

```yaml
rule_files:
  [ - <filepath_glob> ... ]
global:
  [ evaluation_interval: <duration> | default = 1m ]

```


## Alertmanager

Alertmanager是一个独立的组件，负责接收并处理来自Prometheus Server(也可以是其它的客户端程序)的告警信息。
Alertmanager的安装部署和Prometheus类似，只包含一个可执行文件，以及对应的配置文件alertmanager.yml。

alertmanager.yml包括：global、templates、route、receivers、inhibit_rules这几个部分，如下：

```yaml
global: #用于定义一些全局的公共参数，如全局的SMTP配置，Slack配置等内容；
  [ resolve_timeout: <duration> | default = 5m ] # 该参数表示持续多长时间未接收到告警后标记告警状态为resolved
 
templates: # 定义告警通知时的模板，如HTML模板，邮件模板等；
  [ - <filepath> ... ]

route: <route> # 告警路由配置

receivers:  # 告警接收者，即邮箱、微信等
  - <receiver> ...

inhibit_rules: # 告警的抑制规则
  [ - <inhibit_rule> ... ]
```

### Router配置

上面配置文件中最关键的配置项是route和receivers，两者一一对应构成告警信息的路由拓扑网络。

所有告警在Alertmanager中从根route进入，并匹配当前节点的子route，直到找到一个最深的匹配点，并将告警信息发送个该匹配点的receivers。

Alertmanager可以对告警通知进行分组，将多条告警合合并为一个。配置文件中可以使用group_by来定义分组规则，基于告警中包含的标签，如果满足group_by中定义标签名称，那么这些告警将会合并为一个通知发送给接收器。

route 的配置模板如下：

```yaml
receiver: <string>  # 当前route节点的receivers
  group_by: '[' <labelname>, ... ']' ] # 告警分组，依据选择的label，将label取值相同的告警合并
[ continue: <boolean> | default = false ] # 告警在匹配到第一个子节点是否停止

match:      # 通过label值判断匹配
  [ <labelname>: <labelvalue>, ... ]

match_re:   # 通过label值的re表达判断匹配
  [ <labelname>: <regex>, ... ]

[ group_wait: <duration> | default = 30s ]      # 如果在等待时间内当前group接收到了新的告警，这些告警将会合并为一个通知向receiver发送。
[ group_interval: <duration> | default = 5m ]   # 定义相同的Gourp之间发送告警通知的时间间隔
[ repeat_interval: <duration> | default = 4h ]  # 表示重复发送告警的时间

routes:
  [ - <route> ... ]
```

### Receiver配置

当前官方内置的第三方receiver包括：邮件、即时通讯软件（如Slack、Hipchat）、移动应用消息推送(如Pushover)和自动化运维工具（例如：Pagerduty、Opsgenie、Victorops）以及Webhook。

下面的配置信息，展示了Alertmanager对接了企业微信的配置：

```yaml
# 是否接受告警已处理消息消息
[ send_resolved: <boolean> | default = false ]

# 企业微信使用的api_secret
[ api_secret: <secret> | default = global.wechat_api_secret ]

# 企业微信的api_url地址
[ api_url: <string> | default = global.wechat_api_url ]

# 企业微信企业id
[ corp_id: <string> | default = global.wechat_api_corp_id ]

# 告警信息文本
[ message: <tmpl_string> | default = '{{ template "wechat.default.message" . }}' ]
# 告警应用id
[ agent_id: <string> | default = '{{ template "wechat.default.agent_id" . }}' ]
# 接收告警的用户、组织、tag（三选一即可）
[ to_user: <string> | default = '{{ template "wechat.default.to_user" . }}' ]
[ to_party: <string> | default = '{{ template "wechat.default.to_party" . }}' ]
[ to_tag: <string> | default = '{{ template "wechat.default.to_tag" . }}' ]
```

### 告警抑制

告警抑制配置模板如下，当满足一下三个条件时：
- 已发送的告警匹配到target_match和target_match_re规则
- 新的告警匹配到source_match或者source_match_re规则
- 发送的告警与新产生的告警中equal定义的标签完全相同

```yaml
target_match:
  [ <labelname>: <labelvalue>, ... ]
target_match_re:
  [ <labelname>: <regex>, ... ]

source_match:
  [ <labelname>: <labelvalue>, ... ]
source_match_re:
  [ <labelname>: <regex>, ... ]

[ equal: '[' <labelname>, ... ']' ]
```

通过Alertmanager的UI临时屏蔽特定的告警通知。

通过定义标签的匹配规则(字符串或者正则表达式)，如果新的告警通知满足静默规则的设置，则不停止向receiver发送通知。

临时静默可以配置持续时间

### 配置实例

alertmanager对接wechat配置文件

```yaml

global:
  resolve_timeout: 10m
  wechat_api_url: 'https://qyapi.weixin.qq.com/cgi-bin/'
  wechat_api_corp_id: 'xxxx'

templates:
- '/opt/alertmanager/templates/*.tmpl'

inhibit_rules:
- source_match:

route:
  receiver: 'wechat'
  group_by: ['alertname']
  group_wait: 30s
  group_interval: 5m
  repeat_interval: 12h
  routes:
  - receiver: 'wechat'
    group_by: ['alertname','cluster']
    match:
      job: idata

receivers:
- name: 'wechat'
  wechat_configs:
  - send_resolved: false
    to_user: 'LinQing'
    message: '{{ template "wechat.default.message" . }}'
    agent_id: '1000002'
    api_secret: 'xxxx'
```

微信告警模板

```
{{ define "wechat.default.message" }}
{{- if gt (len .Alerts.Firing) 0 -}}
{{- range $index, $alert := .Alerts -}}
{{- if eq $index 0 -}}
告警类型: {{ $alert.Labels.alertname }}
=====================
{{ end }}
故障时间: {{ $alert.StartsAt.Format "2006-01-02 15:04:05" }}
{{ if gt (len $alert.Labels.instance) 0 }}instance: {{ $alert.Labels.instance }}{{ end }}
{{ if gt (len $alert.Labels.cluster) 0 }}cluster: {{ $alert.Labels.cluster }}{{ end }}
{{ if gt (len $alert.Labels.node) 0 }}node: {{ $alert.Labels.node }}{{ end }}
{{ end }}
{{ end }}

{{- if gt (len .Alerts.Resolved) 0 -}}
{{- range $index, $alert := .Alerts -}}
{{- if eq $index 0 -}}
告警类型: {{ $alert.Labels.alertname }}
=====================
{{ end }}
故障时间: {{ $alert.StartsAt.Format "2006-01-02 15:04:05" }}
恢复时间: {{ $alert.EndsAt.Format "2006-01-02 15:04:05" }}
{{ if gt (len $alert.Labels.instance) 0 }}instance: {{ $alert.Labels.instance }}{{ end }}
{{ if gt (len $alert.Labels.cluster) 0 }}cluster: {{ $alert.Labels.cluster }}{{ end }}
{{ if gt (len $alert.Labels.node) 0 }}node: {{ $alert.Labels.node }}{{ end }}
{{ end }}
{{ end }}
{{ end }}
```

## Recoding Rules优化

通过PromQL可以实时对Prometheus中采集到的样本数据进行查询，聚合以及其它各种运算操作。

而在某些PromQL较为复杂且计算量较大时，直接使用PromQL可能会导致Prometheus响应超时的情况。

这时需要一种能够类似于后台批处理的机制能够在后台完成这些复杂运算的计算，对于使用者而言只需要查询这些运算结果即可。

Prometheus通过Recoding Rule规则支持这种后台计算的方式，可以实现对复杂查询的性能优化，提高查询效率。