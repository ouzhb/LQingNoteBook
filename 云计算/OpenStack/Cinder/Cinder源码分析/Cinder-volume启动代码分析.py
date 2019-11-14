def main():
	#注册了cinder.object模块下的所有*.py文件？？？
    objects.register_all()
	#oslo_report的一些配置，是用来上service状态的模块
    gmr_opts.set_defaults(CONF)
	#从cinder.conf初始化配置文件
    CONF(sys.argv[1:], project='cinder',
         version=version.version_string())
	#日志配置
    logging.setup(CONF, "cinder")
    python_logging.captureWarnings(True)
	#root权限提权
    priv_context.init(root_helper=shlex.split(utils.get_root_helper()))
	#OpenStack代码热部署的开关，涉及到配置项：monkey_patch_modules，以及monkey_patch
    utils.monkey_patch()
    gmr.TextGuruMeditation.setup_autorun(version, conf=CONF)
	#lancher是用来存放Service的线程池
    launcher = service.get_launcher()
    LOG = logging.getLogger(__name__)
    service_started = False
    if CONF.enabled_backends:
		"""
		cinder会依次读取配置项enabled_backends中的后端名，根据该名称读取配置文件cinder.conf中的配置组，依次为每个back_end创建Service。
		
		cinder.service.Service是oslo_service.Service的子类，运行在独立的子线程，每个后端配置对应一个Service。
		
		Service的职责：
			1.为每个服务创建cinder.volume.manager.VolumeManager (VolumeManager包含了相关的cinder.volume.driver.BaseVD的派生类，而BaseVD又包含了相应的StorageLibrary)；
			2.创建每个服务的RPC接口；
			3.定义每个服务定时任务，包括上报数据库消息时间，以及定时任务时间。
				涉及两个配置项：
					report_interval
					periodic_interval
		Service对象在main方法中创建好后，会被传递到olso_service的ProcessLauncher中，ProcessLauncher在原进程的基础上fork出子进程，并回调Service的start（）方法进行下一步工作！
		
		"""
        for backend in filter(None, CONF.enabled_backends):
            CONF.register_opt(host_opt, group=backend)
            backend_host = getattr(CONF, backend).backend_host
            host = "%s@%s" % (backend_host or CONF.host, backend)
            # We also want to set cluster to None on empty strings, and we
            # ignore leading and trailing spaces.
            cluster = CONF.cluster and CONF.cluster.strip()
            cluster = (cluster or None) and '%s@%s' % (cluster, backend)#cluster node
            try:
                server = service.Service.create(host=host,
                                                service_name=backend,
                                                binary='cinder-volume',
                                                coordination=True,
                                                cluster=cluster)
            except Exception:
                msg = _('Volume service %s failed to start.') % host
                LOG.exception(msg)
            else:
                # Dispose of the whole DB connection pool here before
                # starting another process.  Otherwise we run into cases where
                # child processes share DB connections which results in errors.
                session.dispose_engine()#db connection
                launcher.launch_service(server)
                service_started = True
    else:
        LOG.error(_LE('Configuration for cinder-volume does not specify '
                      '"enabled_backends". Using DEFAULT section to configure '
                      'drivers is not supported since Ocata.'))

    if not service_started:
        msg = _('No volume service(s) started successfully, terminating.')
        LOG.error(msg)
        sys.exit(1)

    launcher.wait()