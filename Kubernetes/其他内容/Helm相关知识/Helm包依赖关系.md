
# Helm包依赖关系

## 通过 requirements.yaml

基本用法：

```yaml
dependencies:
# 创建依赖，依赖apache-1.2.3
  - name: apache
    version: 1.2.3
    repository: http://example.com/charts
# 依赖本地目录的mysql-0.1.0
  - name: mysql
    version: "0.1.0"
    repository: "file://../mysql"
```
``` yaml
# 安装带依赖的chart需要先构建chart的依赖项，执行以下命令：
helm dependency build charts/myapp
# 上面的命令自动下载子chart到父chart的charts目录！
```

requirements.yaml的特点：
  - conditions条件只要设定了（同时在values中能找到相应的值），必定覆盖tags条件；
  - 如果第一个Condition生效了，会忽略后续的Condition配置；
  - 如果任何chart的tag为真，则启用chart
  - tags和conditions的值必须在顶层chart中设定
  - tags的value推荐是顶层值

## 依赖安装顺序

所有chart中的Kubernetes Object对象合并成一个Set，按照<依赖关系>-<Type>排序，之后按照顺序创建。Kubernetes对象的排序是有顺序的，通常NS、ConfigMap等资源优先排序。

```yaml
# A 包含以下对象：
namespace "A-Namespace"
statefulset "A-StatefulSet"
service "A-Service"
# B 包含以下对象：
namespace "B-Namespace"
replicaset "B-ReplicaSet"
service "B-Service"
# A 依赖 B 对象的创建顺序如下：这里有一个疑惑，创建顺序似乎与谁依赖谁无关，而是谁的name排序在前面，谁先创建！？？参考:https://github.com/helm/helm/blob/master/docs/charts.md#operational-aspects-of-using-dependencies
A-Namespace
B-Namespace
A-StatefulSet
B-ReplicaSet
A-Service
B-Service

```
## Values 导入
下面两种模式可以将子Chart中的配置导入到父Chart中，使父chart可以控制子chart的运行参数。
### exports format

如果子 chart中values顶层包含exports 字段，那么exports 字段的内容可以直接导入父chart的任意字段中。

```yaml
# 父chart在 requirements.yaml 中定义导入
    ...
    import-values:
      - data 
# 子chart values.yaml 中定义导出
...
exports:
  data:
    myint: 99

# 父 chart 实际能得到的values
...
myint: 99

```

### child-parent format

```yaml
# parent's requirements.yaml file
dependencies:
  - name: subchart1
    repository: http://localhost:10191
    version: 0.1.0
    ...
    import-values:
      - child: default.data
        parent: myimports

# parent's values.yaml file

myimports:
  myint: 0
  mybool: false
  mystring: "helm rocks!"

# subchart1's values.yaml file

default:
  data:
    myint: 999
    mybool: true

# parent's final values

myimports:
  myint: 999
  mybool: true
  mystring: "helm rocks!"
```
## 通过charts目录手工管理

- ~~在charts目录下的子Chart可以是tgz格式的压缩包，或者chart目录，但是这些文件不能以"_"或者"."开头。~~

  
