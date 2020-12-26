## docker 使用指南

### 配置代理

```
mkdir -p /etc/systemd/system/docker.service.d/http-proxy.conf

[Service]
Environment="HTTP_PROXY=http://172.21.2.130:1080/"
Environment="HTTPS_PROXY=http://172.21.2.130:1080/"
Environment="NO_PROXY=172.21.128.241:5000" 

systemctl daemon-reload
systemctl restart docker 
```
### 创建私有仓库

```
docker pull registry
mkdir /opt/data/registry
docker run -d -p 5000:5000 -v /opt/data/registry:/tmp/registry registry 
```

```
修改/usr/lib/systemd/system/docker.service 
找到 ExecStar，添加
ExecStart=/usr/bin/dockerd  --insecure-registry <registry_IP>:5000
```

```
docker tag <REPOSITORY/image:tag> <registry_IP>:5000/<image_name>:<tag>
docker push <registry_IP>:5000/<image_name>:<tag>
docker pull <registry_IP>:5000/<image_name>:<tag>
```



### 容器开机重启

```
docker update --restart=always <contain_name>
docker update --restart=on-failure:10  <contain_name>
docker update --restart=no <contain_name>
```



### Docker主机模式

```
docker run -it -d -e DAEMON="true" –net=host ruijie:spark 
```

