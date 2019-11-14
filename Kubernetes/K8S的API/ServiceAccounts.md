# Service Accounts

使用场景：让Pod中的进程能通过HTTPS的方式访问Kubernetes的API。

每个NS下包含一个名为default的ServiceAccount，创建POD时这个SA会被自动注入Pod中（通过volume的方式）。

每个ServiceAccount包含一个secret，这个Secret用来完成Kubernetes API的认证。

## Secret

Secret有三种类型：

Kubernetes提供了Secret来处理敏感信息，目前Secret的类型有3种： 

- Opaque(default): 任意字符串 
- kubernetes.io/service-account-token: 作用于ServiceAccount 
- kubernetes.io/dockercfg: 作用于Docker registry，用户下载docker镜像认证使用



# Service account 自动管理

Kubernetes一定程度上实现了Service Account的自动化管理，通过以下组件能够自动实现Pod的Serivce Account控制：

## Service account admission controller

***确保每一个Pod关联一个ServiceAccount（可能是默认的）***

- 在Pod创建或者更新时，同步配置ServiceAccount参数，如果用户没有指定，那么被设定为默认值
- 确保被引用的Service Account存在
- 自动添加ImagePullSecrets
- 为Pod添加一个用来访问API的Token
- 为Pod添加一个volume，挂载到/var/run/secrets/kubernetes.io/serviceaccount目录，其中保存Serviceaccount信息

## Token Controller


***确保每一个Service Account都有一个Token***

用来管理异步管理serviceAccount的ServiceAccountToken 信息：

- 观察serviceAccount的创建，并创建一个相应的Secret 来允许API访问。
- 观察serviceAccount的删除，并删除所有相应的ServiceAccountToken Secret
- 观察secret 添加，并确保关联的ServiceAccount存在，并在需要时向secret 中添加一个Token。
- 观察secret 删除，并在需要时对应 ServiceAccount 的关联

Token的生成规则：

需要使用--service-account-private-key-file 参数选项将Service Account 密匙（key）文件传递给controller-manager中的Token controller。

key用于 Service Account Token签名。

同样，也需要使用--service-account-key-file 参数选项将相应的（public key）公匙传递给kube-apiserver ，公钥用于在认证期间验证Token。

添加一个额外的Secret
```yaml
{
    "kind": "Secret",
    "apiVersion": "v1",
    "metadata": {
        "name": "mysecretname",
        "annotations": {
            "kubernetes.io/service-account.name": "myserviceaccount"
        }
    },
    "type": "kubernetes.io/service-account-token"
}
```

## Service Account Controller

***确保每一个命名空间有一个default ServiceAccount***

用于管理每个namespaces中的default。