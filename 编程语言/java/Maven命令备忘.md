# 配置代理

通过HTTPS_PROXY配置代理不能再Maven中生效，Maven环境可以独立配置socks代理。

方式1： 通过环境变量export MAVEN_OPTS="${MAVEN_OPTS} -DsocksProxyHost=127.0.0.1 -DsocksProxyPort=1080"

方式2： 在settings.xml文件中配置

```xml
<proxies>
<proxy>
      <id>ss</id>
      <active>true</active>
      <protocol>socks5</protocol>
      <username></username>
      <password></password>
      <host>127.0.0.1</host>
      <port>1080</port>
      <nonProxyHosts>127.0.0.1</nonProxyHosts>
 </proxy>
</proxies>
```


# 导入本地Jar包到Maven仓库

```shell
mvn install:install-file \
-DgroupId=org.scala-lang \
-DartifactId=scala-library \
-Dversion=2.11.12 \
-Dpackaging=jar \
-Dfile=/tmp/jar_install/scala-library-2.11.12.jar
```

# 打包命令

mvn clean scala:compile compile package