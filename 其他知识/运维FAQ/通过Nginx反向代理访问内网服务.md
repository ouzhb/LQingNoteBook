# 背景

Kubernetes 服务部署在独立机房（网络A），需要访问公司开发内网的一些服务（网络B）。

从网络A到网络B的通道是OK的，但是被限制了HTTP/HTTPS访问(不止IP Table限制，应当还有其他限制)，只允许A网络有限台机器访问B网络的指定服务器端口。


## 网络模型

- 机器A：A.A.A.A 位于A网络
    
    - k8s.registry.domain 等域名被解析到A.A.A.A
    
    - 机器A配置以下规则，
    ```shell script
    ACCEPT     tcp  --  B.B.B.B       0.0.0.0/0            tcp dpt:4507
    ```
  
    - 机器A，配置以下Nginx的反向代理：
    ```shell script
    upstream k8s.registry.domain{
    	server B.B.B.B:4507;
    }
    	
    server
    {
    	listen 80;
    	server_name k8s.registry.domain;
    	
    	   location / {
    		#wx lan
    		allow B.B.B.0/24;
    		deny all;
    	    proxy_redirect off;
    	    proxy_set_header Host $host;
    	    proxy_set_header X-Real-IP $remote_addr;
    	    proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    	    proxy_pass http://k8s.registry.domain;
    		
    	}
    	
    	access_log logs/k8s.registry.domain_access.log main;
    }
    ```

    
