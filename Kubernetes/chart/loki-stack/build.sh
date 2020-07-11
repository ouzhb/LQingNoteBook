#!/usr/bin/env bash

rancher app install --namespace loki --version 10.4.0 --answers prometheus/answers.yaml  cattle-global-data:ali-hub-prometheus prometheus
rancher app install --namespace loki --version 4.6.3 --answers grafana/answers.yaml  cattle-global-data:ali-hub-grafana grafana
rancher app install --namespace loki --version 0.30.1 --answers loki/answers.yaml  cattle-global-data:loki-loki loki
rancher app install --namespace loki --version 0.23.2 --answers promtail/answers.yaml  cattle-global-data:loki-promtail promtail