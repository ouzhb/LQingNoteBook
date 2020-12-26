# 背景

在公网暴露apiservice的6443端口被扫描出不安全 Cipher Suite，需要关闭。

Kubernetes 1.16版本 NMAP 默认扫描出以下结果：

```shell script

# 扫描命令

nmap -p 6443 --script=ssl-enum-ciphers.nse  192.168.56.104 

####################################################################
#  Starting Nmap 7.91 ( https://nmap.org ) at 2020-11-03 19:55 CST
#  Nmap scan report for 192.168.56.104
#  Host is up (0.000071s latency).
#  
#  PORT     STATE SERVICE
#  6443/tcp open  sun-sr-https
#  | ssl-enum-ciphers: 
#  |   TLSv1.2: 
#  |     ciphers: 
#  |       TLS_RSA_WITH_AES_128_GCM_SHA256 (rsa 2048) - A
#  |       TLS_RSA_WITH_AES_256_GCM_SHA384 (rsa 2048) - A
#  |       TLS_RSA_WITH_AES_128_CBC_SHA (rsa 2048) - A
#  |       TLS_RSA_WITH_AES_256_CBC_SHA (rsa 2048) - A
#  |       TLS_RSA_WITH_3DES_EDE_CBC_SHA (rsa 2048) - C
#  |     compressors: 
#  |       NULL
#  |     cipher preference: server
#  |     warnings: 
#  |       64-bit block cipher 3DES vulnerable to SWEET32 attack
#  |       Forward Secrecy not supported by any cipher
#  |_  least strength: C
#  
#  Nmap done: 1 IP address (1 host up) scanned in 0.19 seconds
###############################################################

``` 

上述输出中，TLS_RSA_WITH_3DES_EDE_CBC_SHA 需要关闭

# 解决方案

API Service 和 Kubelet 可以使用 **tls-cipher-suites** 指定 Cipher Sutie，使用 **tls-min-version**。

默认情况下，tls-cipher-suites 默认值为空，ssl服务启动使用的 Cipher Sutie 取决于go语言版本。

Kubernetes 1.16 编译的go语言版本为1.12，参考源码 tls 包中的配置（【[地址](https://github.com/golang/go/blob/dev.boringcrypto.go1.12/src/crypto/tls/cipher_suites.go)】）


``` 
var suit2 = []string{
	"TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305",
	"TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305",
	"TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256",
	"TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256",
	"TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384",
	"TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384",
	"TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256",
	"TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA",
	"TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256",
	"TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA",
	"TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA",
	"TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA",
	"TLS_RSA_WITH_AES_128_GCM_SHA256",
	"TLS_RSA_WITH_AES_256_GCM_SHA384",
	"TLS_RSA_WITH_AES_128_CBC_SHA256",
	"TLS_RSA_WITH_AES_128_CBC_SHA",
	"TLS_RSA_WITH_AES_256_CBC_SHA",
	//"TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA",
	//"TLS_RSA_WITH_3DES_EDE_CBC_SHA",
}
```
