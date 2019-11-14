# Mange命令

类似于create命令，但是区别是mange，不会在后端创建一个新的LUN，而是将一个已有的LUN纳入Cinder的管理（会对这个LUN从命名）。

**cinder manage  {HOST}  {source-name}**

- HOST ：必须是pool之一；

- source-name：必须是storage可以识别的，如netapp下可以是path，或者uuid。

# unManage命令

类似于delete命令，但是不会从后端将LUN删除。

**cinder unmanage {vol-id}**

#Manageable-list

列出HOST下所有的LUN或文件信息，这些LUN或文件不在Cinder的管理下，可以通过mange命令纳入管理。

***cinder manageable-list {host}***

