# ISCSI 相关命令

```shell
# 查看当前连接
iscsiadm -m session
# 发现iscsi设备
iscsiadm --mode discovery --type sendtargets --portal 10.102.3.226
# 连接对应的lun，连接上后可以在/dev/disk/by-path/下看到对应的lun设备
iscsiadm -d2 -m node -T iqn.2016-09.com.openebs.cstor:pvc-0baa6ee0-e063-46a5-8cad-8afe38c4403f -p 10.102.3.226 --login
# 断开lun的连接
iscsiadm -d2 -m node -T iqn.2016-09.com.openebs.cstor:pvc-62d1b9d2-da30-42bb-80b9-9bfc1e4a1673 -p 10.108.63.33 --logout
```
