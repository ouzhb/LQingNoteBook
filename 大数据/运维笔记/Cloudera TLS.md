### 证书制作

#### 制作root ca证书

```shell
## 创建所有节点创建工作目录 
mkdir -p /opt/cloudera/security/pki && cd /opt/cloudera/security/pki 
export JAVA_HOME=/usr/java/default
## rootca.pem和key.pem直接拷贝到上述目录
证书见附件

## 拷贝java keystore文件，将rootca.pem写入jvm的keystore
sudo cp /usr/java/default/jre/lib/security/cacerts /usr/java/default/jre/lib/security/jssecacerts

# keystore默认密码为changeit
sudo $JAVA_HOME/bin/keytool -importcert -alias rootca \
-keystore $JAVA_HOME/jre/lib/security/jssecacerts \
-file /opt/cloudera/security/pki/rootca.pem
```
附 ： rootca证书的制作方式
``` shell
## 创建CA的自签名证书 
openssl req -newkey rsa:2048 -nodes -sha256 -keyout key.pem -x509 -days 3650 -outform PEM  -out rootca.pem -dname "CN=root_ca,OU=Bigdata,O=Ruijie,L=Fuzhou,ST=Fujian,C=CN"
```
#### 制作服务端证书
登录rjbdmater1节点，执行以下命令，完成之后拷贝cdh.vip.jks和cdh.vip.pem到rjbdmater2：/opt/cloudera/security/pki
```shell
cd /opt/cloudera/security/pki
export JAVA_HOME=/usr/java/default
## 生成私钥，证书给是Cloudera Manager的证书，官方这里CN使用的是主机名（官方Cloudera Manger部署在单独的机器上），考虑到我们使用HA，所以这里CN使用cdh.vip或者虚IP，这需要和agent配置文件中的server_host匹配
sudo $JAVA_HOME/bin/keytool -genkeypair -alias cdh.vip \
-keyalg RSA -keystore /opt/cloudera/security/pki/cdh.vip.jks \
-keysize 2048 -dname "CN=cdh.vip,OU=Bigdata,O=Ruijie,L=Fuzhou,ST=Fujian,C=CN"

## 生成证书签名请求
sudo $JAVA_HOME/bin/keytool -certreq -alias cdh.vip \
-keystore /opt/cloudera/security/pki/cdh.vip.jks \
-file /opt/cloudera/security/pki/cdh.vip.csr

## 写配置文件生成，生成双向认证的
cat > myconfig.cnf <<EOF
[mysection]
extendedKeyUsage = serverAuth,clientAuth
EOF

## 使用CA的私钥对申请进行签名
openssl x509 -extensions mysection -extfile myconfig.cnf -CA rootca.pem -CAkey key.pem -in cdh.vip.csr -req -days 3650 -outform PEM -out cdh.vip.pem -CAcreateserial -sha256

### 这里和官方有出入，官方还有一个intca.pem？？？用rootca.pem代替
cat /opt/cloudera/security/pki/rootca.pem >> /opt/cloudera/security/pki/cdh.vip.pem

sudo $JAVA_HOME/bin/keytool -importcert -alias cdh.vip \
-file /opt/cloudera/security/pki/cdh.vip.pem \
-keystore /opt/cloudera/security/pki/cdh.vip.jks

```
#### 自作客户端证书
由于所有节点都需要安装agent，因此都要制作agent的证书
```shell
### 生成本地的agent证书
sudo $JAVA_HOME/bin/keytool -genkeypair -alias $(hostname -f) -keyalg RSA -keystore \
/opt/cloudera/security/pki/$(hostname -f).jks -keysize 2048 -dname \
"CN=$(hostname -f),OU=Bigdata,O=Ruijie,L=Fuzhou,ST=Fujian,C=CN"
 
sudo $JAVA_HOME/bin/keytool -certreq -alias $(hostname -f) \
-keystore /opt/cloudera/security/pki/$(hostname -f).jks \
-file /opt/cloudera/security/pki/$(hostname -f).csr

cat > myconfig.cnf <<EOF
[mysection]
extendedKeyUsage = serverAuth,clientAuth
EOF

openssl x509 -extensions mysection -extfile myconfig.cnf -CA rootca.pem -CAkey key.pem -in $(hostname -f).csr -req -days 3650 -outform PEM -out $(hostname -f).pem -CAcreateserial -sha256

cat /opt/cloudera/security/pki/rootca.pem >> /opt/cloudera/security/pki/$(hostname -f).pem

sudo $JAVA_HOME/bin/keytool -importcert -alias $(hostname -f) \
-file /opt/cloudera/security/pki/$(hostname -f).pem \
-keystore /opt/cloudera/security/pki/$(hostname -f).jks

## 创建软连接
sudo ln -s /opt/cloudera/security/pki/$(hostname -f).pem \
/opt/cloudera/security/pki/agent.pem

```
#### PS：其他命令
``` shell
# 查看keystore当前导入的证书
keytool -list -keystore /usr/java/default/jre/lib/security/jssecacerts|grep -v "Certificate fingerprint"|grep rootca |grep 2018

keytool -list -keystore /opt/cloudera/security/pki/$(hostname -f).jks|grep -v "Certificate fingerprint"|grep 2018
```

### Cloudera相关配置

#### 配置Admin Console

修改CM配置：

```properties
Cloudera Manager TLS/SSL 服务器 JKS Keystore 文件位置 = /opt/cloudera/security/pki/cdh.vip.jks
Cloudera Manager TLS/SSL 服务器 JKS Keystore 文件密码 = 123456
对 Admin Console 使用 TLS 加密 = use
```

修改MGMT配置：

```properties
TLS/SSL 客户端 Truststore 文件位置 : /usr/java/default/jre/lib/security/jssecacerts
Cloudera Manager Server TLS/SSL 证书信任存储库密码 : changeit
```
按照上面的配置修改之后，MGMT无法正常启动。
``` 
### 解决方案一：将MGMT中所有实例的scm.url改为虚IP地址

####Service Monitor和Host Monitor
在cmon.conf的配置文件上添加：
<property>  
<name>scm.server.url</name>
<value>https://cdh.vip:57183</value>
</property>
#### Event Server
修改com.cloudera.cmf.eventcatcher.server.EventServerConfiguration#getScmServerUrl 方法，并替换Jar包
#### Alert Publisher（从代码看该实例似乎无需使用scm.url）
修改com.cloudera.enterprise.alertpublisher.AlertPublisherConfig#getURLforSCM 方法，并替换Jar包
```

```
### 解决方案二：修改对应的Cloudera Jar包(方案似乎有缺陷，有时会导致部分服务的状态无法获取)

# 修改了scm-client.jar中的以下两个方法：
# com.cloudera.cmf.BasicScmProxy#authenticate
# com.cloudera.cmf.BasicScmProxy#fetchFromUrl

#使它们校验证书中CN字段时，允许所有HOSTNAME

# 复制这两个Jar包到所有节点的/usr/share/cmf/common_jars目录
```



完成上述配置后，所有CM的API接口需要使用HTTPS来访问，需要API接口也需要使用HTTPS调用。

#### 配置Cloudera Manager Agents使用TLS

修改CM配置：

```properties
为代理使用 TLS 加密 = use
```

修改配置agent配置文件：/etc/cloudera-scm-agent/config.ini 

```properties
use_tls = 1
server_host = cdh.vip
```

#### 配置Agent使用证书校验Server

修改配置agent配置文件：/etc/cloudera-scm-agent/config.ini 

```properties
verify_cert_file=/opt/cloudera/security/pki/rootca.pem
```



#### 配置Agent证书校验

配置Agent证书校验，可以保证集群内Agent的真实性。

```shell
## 导出节点的Agent证书
sudo $JAVA_HOME/bin/keytool -importkeystore \
-srckeystore /opt/cloudera/security/pki/$(hostname -f).jks \
-destkeystore /opt/cloudera/security/pki/$(hostname -f)-key.p12 \
-deststoretype PKCS12 -srcalias $(hostname -f)

sudo openssl pkcs12 \
-in /opt/cloudera/security/pki/$(hostname -f)-key.p12 \
-nocerts \
-out /opt/cloudera/security/pki/$(hostname -f).key

sudo ln -s /opt/cloudera/security/pki/$(hostname -f).key \
/opt/cloudera/security/pki/agent.key

# 创建一个密码文件
echo "123456" > /etc/cloudera-scm-agent/agentkey.pw
chmod 440 /etc/cloudera-scm-agent/agentkey.pw
chown root:root /etc/cloudera-scm-agent/agentkey.pw
```

修改配置agent配置文件：/etc/cloudera-scm-agent/config.ini 

```properties
client_key_file=/opt/cloudera/security/pki/agent.key
client_keypw_file=/etc/cloudera-scm-agent/agentkey.pw
client_cert_file=/opt/cloudera/security/pki/agent.pem
```

修改CM配置

```properties
使用代理到服务器的 TLS 身份验证 = use
Cloudera Manager TLS/SSL 证书信任存储库文件 = /usr/java/default/jre/lib/security/jssecacerts
Cloudera Manager TLS/SSL 证书信任存储库密码 = changeit
```

