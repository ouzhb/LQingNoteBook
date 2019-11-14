# Solr 节点恢复

IData项目在Solr中创建searchKeyword集合，该集合有2个shard，每个shard有2个副本，数据保存在/metadata/solr/lib目录下。

当某个节点/metadata/solr/lib的数据丢失时，可以按照以下方式恢复：

- 停止solr服务
- 删除可用节点/metadata/solr/lib/searchKeyword_shardXX_replica_XX/tlog目录下的数据
- 拷贝/metadata/solr/lib/数据到另一个节点
- 修改故障节点searchKeyword_shardXX_replica_XX目录的文件名，以及core.properties文件（如何修改参考ZK上的/solr/collections/searchKeyword/state.json）
- 启动solr服务