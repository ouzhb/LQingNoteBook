# Goproxy

https://goproxy.cn/

# 远程调试

```shell
dlv exec --api-version=2 --headless --listen=:2345 ./build/bin/sdpctl 
kill -9 `ps -ef | grep "dlv" -E | awk '{print $2}'`
```

# 交叉编译

```json
GOARCH=amd64;GOOS=linux
```