#  关于Network Namespace

- Linux中Namespace 可以使用 ```clone()```接口创建（即需要写C语言的接口来创建），但是Network Namespace的接口已经集成到了```ip netns```命令中；

- 进程的所有ns在/proc/<pid>/ns 中保存为一个符号链接（对应到一个打开的inode），这些文件的作用是保证ns存在（linux 保证只要这些文件时open状态，即使ns中没有任何进程，ns也不会被销毁）；

- 进程进入ns(setns接口)，进程逃离ns(unshare接口)

- 每个网络命名空间在/var/run/netns中有一个挂载点（实际操作中，发现只有使用ip netns创建的ns是这样）， docker 将容器的ns隐藏了起来，通过下面的方式可以进行管理

    ```shell
    docker inspect <docker-name>|grep Pid
    ln -s /proc/<Pid>/ns/net /var/run/netns/<docker-name>
    ```
- 对于新建的nets，其中只有一个lo设备，并且需要手工启动

    ```shell
    ip netns list
    ip netns create netns1
    ip netns delete netns1
    ip netns exec netns1 ip link list
    ip netns exec netns1 ip link set dev lo up
    
    ```
- 使用```ip netns```来管理docker的网络命名空间```ln -s /var/run/docker/netns  /var/run/netns ```

# 关于veth pair

- 成对出现的虚拟以太网卡，每一端都可以放到不同的netns中

    ```shell
    ip link add veth0 type peer name veth1
    ip link set veth1 netns netns1
    ip nethns exec netns1 ifconfig veth1 10.1.1.1/24 up
    ifconfig veth0 10.1.1.2/24 up 
    ```
- 相比而言，真实的网络设备只能放在，根命名空间中


# setns 工具

```
#define _GNU_SOURCE
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
 
#define errExit(msg) do { perror(msg); exit(EXIT_FAILURE);} while (0)
 
int main(int argc, char *argv[]) {
	
	int fd;
	if (argc < 3) {
		fprintf(stderr, "%s /proc/PID/ns/FILE cmd args...\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	
	fd = open(argv[1], O_RDONLY);   // Get descriptor for namespace
    
	if (fd == -1)
		errExit("Can't not Open");
	
	if (setns(fd, 0) == -1)         // Join that namespace
		errExit("setns");
	
	execvp(argv[2], &argv[2]);      // Execute a command in namspace
	
	errExit("execvp");
}
  
```

# VLAN

VLAN 可以将一个物理的二层网络划分成最多4093个逻辑网络，每个逻辑网络由 VLAN ID 区分。

linux中一个网卡可以同时支持 VLAN10 和 VLAN 20，即一张网卡可以收发不同网段的数据～～！！

PS：前提是这张网卡在交换上要连接到trunk口



# VXLAN

VXLAN 是一种***基于UDP工作的隧道协议***，用户可以将二层数据封装到UDP的数据包中，从而在三层提供以太网二层服务。

# 参考

Kubernetes网络权威指南