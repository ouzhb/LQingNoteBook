# Jenkins 配置插件源

清华的仓库是官方的仓库地址：https://mirrors.tuna.tsinghua.edu.cn/jenkins/updates/update-center.json

update-center.json 的地址中所有插件的下载地址全是指向原有仓库的，通过以下方式配置才能正常使用:

- Jenkins启动参数添加：-Dhudson.model.DownloadService.noSignatureCheck=true
- 替换/root/.jenkins/updates/default.json文件

    - www.google.com，替换为www.baidu.com
    - http://updates.jenkins-ci.org/download/plugins/，替换为https://mirrors.tuna.tsinghua.edu.cn/jenkins/plugins/

其他备选 update-center.json 地址：[【参考】](https://jenkins-update.davidz.cn/)

https://jenkins-update.davidz.cn/update-center.json

# 参考

[墙内 Jenkins 插件下载的一种解决方案](https://blog.davidz.cn/jenkins-update-solution-inside-gfw/)

[官方Mirrors信息](http://mirrors.jenkins-ci.org/status.html)