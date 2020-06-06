# 安装部署

- 下载离线安装包 harbor-offline-installer-v1.10.2.tar.gz 解压

- 编辑 harbor.yml ，注意以下配置：

	- hostname：主机名（填：harbor）
	- http.port: http端口（填：18000）
	- https.port: https端口（填：18443）
	- https.certificate: CA签发到服务端的证书
	- https.private_key: 服务端私钥
	- data_volume: 数据目录，映射到主机上
	
- 执行prepare，生成配置

- docker-compose启动

# 创建SSL证书

注意创建时，harbor节点主机名为：harbor

配置/etc/hosts 中全域名为：<ipaddress> harbor harbor.goblin.com

```shell

# 创建根证书

openssl genrsa -out ca.key 4096

openssl req -x509 -new -nodes -sha512 -days 3650 \
 -subj "/C=CN/ST=Fujian/L=FuZhou/O=LQ/OU=Goblin/CN=harbor.goblin.com" \
 -key ca.key \
 -out ca.crt
 
# 生成服务端证书
 
## 1. 服务端私钥
openssl genrsa -out harbor.goblin.com.key 4096
 
## 2. 服务端的签发请求
openssl req -sha512 -new \
    -subj "/C=CN/ST=Fujian/L=FuZhou/O=LQ/OU=Goblin/CN=harbor.goblin.com" \
    -key harbor.goblin.com.key \
    -out harbor.goblin.com.csr
 
## 3. 生成请求配置
cat > v3.ext <<-EOF
authorityKeyIdentifier=keyid,issuer
basicConstraints=CA:FALSE
keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @alt_names

[alt_names]
DNS.1=harbor.goblin.com
DNS.2=harbor.goblin
DNS.3=harbor
EOF
 
## 4. 生成服务端证书
openssl x509 -req -sha512 -days 3650 \
    -extfile v3.ext \
    -CA ca.crt -CAkey ca.key -CAcreateserial \
    -in harbor.goblin.com.csr \
    -out harbor.goblin.com.crt

# 配置Docker客户端证书  
  
## 1. 证书转换cert格式（Docker使用这个格式的证书）
openssl x509 -inform PEM -in harbor.goblin.com.crt -out harbor.goblin.com.cert
 
## 2. 配置docker端证书
mkdir -p  /etc/docker/certs.d/harbor.goblin.com:<https_port>
cp harbor.goblin.com.cert /etc/docker/certs.d/harbor.goblin.com:<https_port>
cp harbor.goblin.com.key /etc/docker/certs.d/harbor.goblin.com:<https_port>
cp ca.crt /etc/docker/certs.d/harbor.goblin.com:<https_port>

## 3. 修改/etc/hosts,添加：<ipaddress> harbor harbor.goblin.com

## 4. 登录验证
docker login lqdocker.goblin.com:18443 -u admin

```

# 关于免费SSL证书

在[letsencrypt](https://letsencrypt.osfipin.com/)可以申请到免费的SSL证书，并且配合[certbot](https://github.com/certbot/certbot)能够自动更新，用户只要拥有一个合法域名基本上就可以获得无限期的SSL证书。

# 参考

[安装包地址](https://github.com/goharbor/harbor)

[配置SSL](https://goharbor.io/docs/1.10/install-config/configure-https/)