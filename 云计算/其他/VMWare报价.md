# vSphere 

vSphere 是VMware 管理平台的总称包含：

- ESXi hypervisor：虚拟层套件（类似Kvm）
- vCenter management server：云平台管理软件
- 网络、存储层的相关虚拟化组件

收费模式：

- 用户购买vSphere时，VMWare针对不同规模的企业推出了不同的套餐，每种套餐包含普通版和Plus版
- Basic / Production 两种级别的技术支持，均是按年收费的

|                         | 集群规模                                               | 价格       | 技术支持价格/年 | 说明                         |
| ----------------------- | ------------------------------------------------------ | ---------- | --------------- | ---------------------------- |
| Essentials              | 支持三个节点，每个节点2个CPU                           | $ 510      | $ 67            | 经济实惠的丐版，不能升级     |
| Essentials Plus         | 支持三个节点，每个节点2个CPU                           | $4,625     | $ 971           | 多了迁移、高可用、副本等功能 |
| Standard                | 按CPU个数收费                                          | $995/CPU   | $273            | 能够部署大型集群             |
| Enterprise Plus         | 按CPU个数收费                                          | $3,595/CPU | $899            | 多了一些nb功能               |
| Platinum                | 按CPU个数收费                                          | $4,595/CPU | $1,049          | 又多了一些nb功能             |
| vCenter Server Standard | 一个集群一个就行Essentials/Essentials Plus不用额外购买 | $6,175     |                 |                              |

# 参考
[【VMware or Hyper-V? Part 3: Virtualization Licensing Costs】](https://blog.heroix.com/blog/virtualization-licensing)

[【官方报价】](https://www.vmware.com/reusable_content/vsphere_pricing.html)