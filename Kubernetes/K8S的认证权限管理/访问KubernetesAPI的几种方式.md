# 方式一

找到相应的secrets，获取其对应的Token。

```yaml
Name:         prometheus-server-token-snzg4
Namespace:    loki
Labels:       <none>
Annotations:  kubernetes.io/service-account.name: prometheus-server
              kubernetes.io/service-account.uid: 7ef3abea-3e4f-4e3b-83a1-33277377d3bd

Type:  kubernetes.io/service-account-token

Data
====
namespace:  4 bytes
token:      <!---需要的token--->
ca.crt:     1025 bytes
```

打开PostMan指定认证方式为Bearer Token，访问指定的API。

# 方式二

```shell
curl -v -k -H ＂Authorization: Bearer WhCDvq4VPpYhrcfmF6ei7V9qlbqTubUc＂ HTTPs://10.240.122.184:443/api/v1/namespaces/default/pods/grafana
```