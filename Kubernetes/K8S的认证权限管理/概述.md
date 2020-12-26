# API 版本

- Alpha：随时可能被删除，可能出现不兼容的问题
- Beta：不会删除该API，可能出现细微调整，如果出现Stable版本和当前版本不兼容，那么官方会提供迁移工具
- Stable：稳定的API接口
  
# API Goups

指当前Kubernetes在使用的API，通常\$GROUP_NAME/\$VERSION。

修改--runtime-config配置项可以启用或者关闭某些API组，如下：

```properties

# 修改配置文件，/etc/systemd/system/kube-apiserver.service
--runtime-config=batch/v2alpha1=true,settings.k8s.io/v1alpha1=true

# 同时也可以细致到限制某种资源

--runtime-config=extensions/v1beta1/deployments=false,extensions/v1beta1/jobs=false

```

所有API信息，参考[v1.12](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.12/)

# 配额限制

API配额限制指：在某一名字空间（namespace）中所创建的特定类型的对象的数量。

可以通过ResourceQuota对象设定配额。

```yaml
apiVersion: v1
kind: ResourceQuota
metadata:
  name: object-quota-demo
spec:
  hard:
    persistentvolumeclaims: "1"
    services.loadbalancers: "2"
    services.nodeports: "0"
```

# API 访问控制

使用客户端访问Kubernetes的API时经过以下流程：

- 认证
- 授权
- 准入控制
  
默认情况下认证只发生在HTTPS访问中，而Kubernetes集群外部访问使用HTTPS，内部访问使用HTTP。

## 认证
Kubernetes可以指定多个认证模块，只要一个模块认证成功，认证即通过。

常见认证模块包括：
- 客户端证书
- 密码
- Plain Tokens
- Bootstrap Tokens
- JWT Tokens(used for service accounts)
  
## 授权
Kubernetes可以指定多个授权模块，只要一个模块授权成功，授权即通过。

常见授权模块包括：

- ABAC模式
- RBAC模式
- Webhook模式
  
## 准入控制

准入控制模块是一个串行的插件队列，用来对请求做进一步的验证或添加默认参数。

默认情况下可以在/etc/systemd/system/kube-apiserver.servicez中配置准入控制模块：

```
--admission-control=NamespaceLifecycle,LimitRanger,ServiceAccount,DefaultStorageClass,ResourceQuota,NodeRestriction,PodPreset
```

基于Kubernetes的Webhooks(beta in 1.9)和Initializers (alpha)，用户可以自定义准入控制器（不需要重新编译kube-apiserver），并且在Kubernetes运行过程中将控制器注入环境。