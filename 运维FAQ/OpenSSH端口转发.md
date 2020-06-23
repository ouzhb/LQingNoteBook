# SSH tunneling

SSH tunneling 是一种隧道加密技术，即一种网络协议报文封装到SSH协议中进行传输，从而达到信息安全传输的作用。在实际工作过程中，SSH进程会绑定一个Port，所有targeted到这个Port的报文都将透明的加密，并转发到**远程主机**或者**本地的其他端口**。

应用场景：突破防火墙限制，翻墙！！！Vagrant等虚拟机环境下，SSH tunneling 被用来当作DNAT技术的替代，提供外网到内网的网络访问。


SSH tunneling 可以通过以下方式进行工作：

- Local Port Forwarding：在本地不能访问防火墙内的网络，但可以通过SSH连接防火墙内的某台电脑，可以利用这个SSH通道进行本地端口转发，实现穿透防火墙。

    - ssh命令在本地执行，监听端口在本地
    - 访问本地监听端口时，请求通过SSH通道抵达对端服务器，在转发到实际目的地


- Remote Port Forwarding：从防火墙内部穿透到外面，从内部向外部建一条隧道。通道建立后，实际效果和Local Port Forwarding一致
    - ssh命令在远端执行，监听端口在本地，但是监听的127.0.0.1
    - 需要在远端执行ssh命令，创建tunnel

- Dynamic Tunneling


```shell
# Local Port Forwarding，54321是本地监听端口，<DES_IP>:<DES_IP>是目的地址，<RemoteServer>是代理的用户名
ssh -N -L 54321:<DES_IP>:<DES_IP> username@<RemoteServer>
# Remote Port Forwarding
ssh -f -N -g -R 2900:<DES_IP>:<DES_IP> username@<LocalServer>

# 其他参数的含义
# -N 表示不要打开shell
# -L Local Port Forwarding
# -R Local Port Forwarding
# -D Dynamic Tunneling
# -f 后台tunnel进程后台执行
# -g 网关方式运行tunnel

```

# 参考

[图解SSH Tunneling](https://www.cnblogs.com/kidsitcn/p/11090252.html)