# Install

安装前编辑kube-proxy的配置，并且重启服务

```yaml
# 安装前编辑kube-proxy的配置，并且重启服务
kubectl edit configmap -n kube-system kube-proxy

apiVersion: kubeproxy.config.k8s.io/v1alpha1
kind: KubeProxyConfiguration
mode: "ipvs"
ipvs:
  strictARP: true
```

创建密钥文件
```yaml
# Install first
kubectl create secret generic -n metallb-system memberlist --from-literal=secretkey="$(openssl rand -base64 128)"
```

# 参考

[官方配置](https://metallb.universe.tf/configuration/)