# 1.背景

Tomcat中实现根据Header信息，将请求转发到不同的virtual host。

# 2. Tomcat 架构分析

tomcat 主要包括：Connector 和 Container两种类型的资源：

- Connector：负责接收tcp请求，并基于HTTP协议构造Request，发送给Container进行处理
- Container：请求处理容器，负责将Request发送到具体的servlet，以及将Response交给Connector写入到Socket。Container主要有以下几种：
    - Engine：包括多个virtual host容器
    - Host：代表一个virtual host，可以包含多个Context容器
    - Context：代表一个web应用程序，可以包含多个Wrapper容器
    - Wrapper：代表一个独立的Servlet容器
    
# 3. Valve机制
上述容器在处理Request请求时，通过Pipeline和Valve机制实现请求路由转发等功能。每种容器都有一个默认的valve，这个valve在Pipeline的最后被执行。

- StandardEngineValve
- StandardHostValve
- StandardContextValve
- StandardWrapperValve

用户可以继承org.apache.catalina.valves.ValveBase，并在server.xml中配置，实现对tomcat的扩展。

# 4. SDP 通用域名方案

- 任意域名只要带了 sdp-app-name 参数就会被路由到相应的颗粒，如果颗粒不存在则默认路由到主颗粒（构建时颗粒列表第一个）
- app1.ndaeweb.com 不带 sdp-app-name 参数会被路由到 app1，
- app1.ndaeweb.com 带 sdp-app-name=app2 会被路由到 app2
- sdp-app-name 参数的值必须是小写的

# 参考

- tomcat请求处理时序图：https://tomcat.apache.org/tomcat-8.0-doc/architecture/requestProcess/request-process.png
- 打包编译参考：https://tomcat.apache.org/tomcat-8.5-doc/building.html



