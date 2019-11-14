# Custom resources
## Custom controllers

custom resources使用户可以快速在K8S中创建、获取结构数据。当为custom resource创建相应的custom controller,用户即在K8S中声明了一个API。

declarative API允许声明或指定资源的所需状态，并尝试将实际状态与此期望状态相匹配。

custom controller将结构化数据解释为用户期望状态，并且不断采取行动以实现和维持该状态。

custom controller的生命周期和Cluster保持一致，能够管理集群的任意资源，但是通常他和custom resources协同工作。





