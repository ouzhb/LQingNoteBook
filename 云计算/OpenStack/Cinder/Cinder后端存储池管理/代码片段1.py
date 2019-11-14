

"""
HostManager的结构：
	backend_state_map：<backend_key:BackendState>
		cinder.scheduler.host_manager.BackendState的结构：
				封装了大部分配置信息，以及pools信息
		cinder.scheduler.host_manager.PoolState的派生自BackendState！
				最后API代码获得的是所有PoolState.capabilities构成的字典
	service_states：
	_no_capabilities_backends：
	service_states_last_update：

cinder.scheduler.host_manager.BackendState的结构：
	封装了大部分配置信息，以及pools信息

cinder.scheduler.host_manager.PoolState的派生自BackendState！

"""


#cinder.scheduler.host_manager.HostManager.get_pools代码：
    def get_pools(self, context):
        """Returns a dict of all pools on all hosts HostManager knows about."""
		
        self._update_backend_state_map(context)

        all_pools = []
        for backend_key, state in self.backend_state_map.items():#backend_key：cinder.scheduler.host_manager.BackendState
            for key in state.pools:#这里的key指的是pools的名字
				#pool为cinder.scheduler.host_manager.PoolState
                pool = state.pools[key]
                # use backend_key.pool_name to make sure key is unique
                pool_key = vol_utils.append_host(backend_key, pool.pool_name)
                new_pool = dict(name=pool_key)
                new_pool.update(dict(capabilities=pool.capabilities))
                all_pools.append(new_pool)

        return all_pools

"""
get_pools的信息来自c-sch的service_states！！！！
HostManager的service_states如何初始化，如何更新状态！？
"""
    def _update_backend_state_map(self, context):
		
        # Get resource usage across the available volume nodes:
        topic = constants.VOLUME_TOPIC
		#获取所有非disabled的volume服务
        volume_services = objects.ServiceList.get_all(context,
                                                      {'topic': topic,
                                                       'disabled': False,
                                                       'frozen': False})
        active_backends = set()
        active_hosts = set()
        no_capabilities_backends = set()
        for service in volume_services.objects:
            host = service.host#获取host信息localhost.localdomain@NetAppIscsiBackend
            if not service.is_up:
                LOG.warning(_LW("volume service is down. (host: %s)"), host)
                continue
			
			#返回cluster信息或者host信息
			
            backend_key = service.service_topic_queue
            # We only pay attention to the first up service of a cluster since
            # they all refer to the same capabilities entry in service_states
            if backend_key in active_backends:
                active_hosts.add(host)
                continue

            # Capabilities may come from the cluster or the host if the service
            # has just been converted to a cluster service.
            capabilities = (self.service_states.get(service.cluster_name, None)
                            or self.service_states.get(service.host, None))
			#疑问：开机时理应获得none？？？
            if capabilities is None:
                no_capabilities_backends.add(backend_key)
                continue

            # Since the service could have been added or remove from a cluster
            backend_state = self.backend_state_map.get(backend_key, None)
            if not backend_state:
                backend_state = self.backend_state_cls(
                    host,
                    service.cluster_name,
                    capabilities=capabilities,  #这里的capabilities必然不是none！！！从service_states是从service_states获得的
                    service=dict(service))
                self.backend_state_map[backend_key] = backend_state

            # update capabilities and attributes in backend_state
            backend_state.update_from_volume_capability(capabilities,
                                                        service=dict(service))
            active_backends.add(backend_key)
		#还无法从backend_key获得capabilities的服务
        self._no_capabilities_backends = no_capabilities_backends

        # remove non-active keys from backend_state_map
        inactive_backend_keys = set(self.backend_state_map) - active_backends
        for backend_key in inactive_backend_keys:
            # NOTE(geguileo): We don't want to log the removal of a host from
            # the map when we are removing it because it has been added to a
            # cluster.
            if backend_key not in active_hosts:
                LOG.info(_LI("Removing non-active backend: %(backend)s from "
                             "scheduler cache."), {'backend': backend_key})
            del self.backend_state_map[backend_key]
					
"""

/opt/stack/cinder/cinder/scheduler/manager.py
/opt/stack/cinder/cinder/scheduler/driver.py
update_service_capabilities
"""
			
			
			
			
			
			
			
			
			
			
			
			
			
			
