# 说明

在容器中可以使用 GlusterFS Volume 插件读写Glusterfs的数据：

- Git 地址：https://github.com/sapk/docker-volume-gluster（这个仓库作者还在维护，但是几经尝试发现根本无法使用，甚至装好plugin后无法卸载）

Gluster官方提供了一个Java客户端，可以在不mount的情况下直接向volume写入文件，这个Tools依赖[libgfapi-jni](https://github.com/gluster/libgfapi-jni)，经过尝试发现无法使用。

- Git 地址：https://github.com/gluster/glusterfs-java-filesystem


PS: 原生Docker并没有对GlusterFS提供太多支持，上面提到的插件在很久以前已经不再维护。


