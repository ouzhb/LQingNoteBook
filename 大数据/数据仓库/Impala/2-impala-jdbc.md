---
title: Impala 安装部署
categories: "大数据"     # 类别暂时定为：大数据、DevOps、笔记、小工具
date: 2019-03-11
comments: false
toc: true
tags:
	- Cloudera
	- Impala
---

关于 Impala jdbc驱动的相关说明

<!--more-->

几个姿势点：

- impalad的jdbc/odbc端口是21050，通过--hs2_port参数可以指定；
- impala可以使用 Cloudera JDBC Connector 和 Hive 0.13 JDBC driver 两种驱动（CDH6 官方推荐使用2.5.45、2.6.2版本的impala 驱动）；
- 使用Haproxy等负载平衡工具时，应该关闭重用连接的配置，同时需要保证工具的超时时间足够大；
- 

[Cloudera 官方指引](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/impala_jdbc.html)

[Cloudera connector 驱动下载](https://www.cloudera.com/documentation/other/connectors/impala-jdbc/latest.html)

[Cloudera JDBC 官方文档](chrome-extension://ikhdkkncnoglghljlkmcimlnlhkeamad/pdf-viewer/web/viewer.html?file=https%3A%2F%2Fwww.cloudera.com%2Fdocumentation%2Fother%2Fconnectors%2Fimpala-jdbc%2Flatest%2FCloudera-JDBC-Driver-for-Impala-Install-Guide.pdf)

[ HIVE JDBC驱动安装 ](https://www.cloudera.com/documentation/enterprise/6/6.1/topics/hive_jdbc_odbc_driver_install.html#hive_installing_jdbc_odbc_drivers)