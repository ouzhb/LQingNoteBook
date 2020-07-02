# 1. 删除SPC

- 删除SPC时，需要删除该存储中的所有cvr，否则删除命令无法执行。使用过程中有发现，cStorVolume 已经删除但是相应的cvr没有删除的情况，这个时候需要手工强制删除。 
- 删除SPC后，磁盘中会遗留/dev/sdx1和/dev/sdx9这两个分区，需要使用fdisk手工删除，并重启节点的ndm（将bd的状态从release --> Unclaimed）

# 2. 挂载PVC失败

按照以下步骤检查：

1. 查看 PVC 容器日志异常
2. 查看 Pool Pod 日志
3. 查看 maya-api 日志
4. 检查 csp 资源的状态是否为Healthy，如果是 PoolCreationFailed 说明创建pool失败，需要删除SPC重建

# 3. 不小心误删 pvc pod

如果不小心误删pvc pod，openebs 不会自动重建，此时如果业务pod重建，可能导致pvc不可用

# 4. clone pod

对 cstor 创建 clone 时，有可能导致 volume 无法删除。

# 5. 回收的磁盘，不可用于lvm分区

parted /dev/sdb mklabel msdos  

