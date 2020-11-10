# Docker 容器目录结构

## 1. 日志目录

**/var/lib/docker/containers/{container-id}/**，该目录实际上是docker的默认容器日志输出目录，可以配置日志滚动限制该目录的大小。

## 2. 数据目录

**/var/lib/docker/containers/** 是容器使用Overlay2存储引擎时，运行容器的数据目录，以及本地镜像的数据目录。

 使用**docker inspect 191c86e56e8c --format="{{json .GraphDriver}}" **，获取以下容器数据目录：
 
 ```json
{
    "Data": {
        "LowerDir": "/data/var/lib/docker/overlay2/543b19c6ce4d92e3f63f91e59399a1e53771e5d97e82334a1c96bd5113f9c65f-init/diff:/data/var/lib/docker/overlay2/81a9a93c243b586e6bc02d79382e4ce07a14ef24e061cf0ee360024d33342b2d/diff",
        "MergedDir": "/data/var/lib/docker/overlay2/543b19c6ce4d92e3f63f91e59399a1e53771e5d97e82334a1c96bd5113f9c65f/merged",
        "UpperDir": "/data/var/lib/docker/overlay2/543b19c6ce4d92e3f63f91e59399a1e53771e5d97e82334a1c96bd5113f9c65f/diff",
        "WorkDir": "/data/var/lib/docker/overlay2/543b19c6ce4d92e3f63f91e59399a1e53771e5d97e82334a1c96bd5113f9c65f/work"
    },
    "Name": "overlay2"
}
```
 
 上述目录中：
 
 - UpperDir：容器运行写入目录，该目录表征了容器实际运行写入
 - MergedDir：联合挂载目录，LowerDir、UpperDir、WorkDir 联合挂载到改目录
 - WorkDir：不太清楚干啥的
 - LowerDir：底层目录，一般是只读的指向基础镜像，和UpperDir同名的**xxx-init**目录是容器启动配置目录，也是只读的