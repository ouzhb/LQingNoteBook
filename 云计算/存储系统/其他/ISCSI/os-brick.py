
获取当前所有可行的路径，并且在/dev/disk/by-path目录生成软连接：
	接口API：    def _get_potential_volume_paths(self, connection_properties,connect_to_portal=True,use_rescan=True):
	
	1.发现所有可能path
	
		iscsiadm -m discovery -t sendtargets -I default -p 172.24.3.168:3260 

		根据返回值，生成ips、iqns、luns:
			172.24.3.168:3260,1045 iqn.1992-08.com.netapp:sn.869ebc3e49a811e79ded00a098c364d6:vs.30
			172.24.3.167:3260,1044 iqn.1992-08.com.netapp:sn.869ebc3e49a811e79ded00a098c364d6:vs.30

			ips=[172.24.3.168:3260,1045,172.24.3.167:3260,1045]
			iqns=[iqn.1992-08.com.netapp:sn.869ebc3e49a811e79ded00a098c364d6:vs.30,iqn.1992-08.com.netapp:sn.869ebc3e49a811e79ded00a098c364d6:vs.30]

				@staticmethod
				def _get_luns(con_props, iqns=None):
					# 目前从netapp返回的target信息中不包含target_luns、target_iqns，只有target_lun和target_iqn！
					# os-brick通过discovery命令来发现target地址，能找到几个地址，就认为有几个target_iqn！并且认为这些target_iqn中的lun_id都相同
					luns = con_props.get('target_luns')
					num_luns = len(con_props['target_iqns']) if iqns is None else len(iqns)
					return luns or [con_props.get('target_lun')] * num_luns

			luns=[target_lun,target_lun]
		
		os-brick注释提到两种类型的多路径设备，netapp属于后者：

			1.所有路径iqn相同，ip地址和端口不同；
			2.所有路径iqn不相同，ip地址和端口不同；

	2.连接所有path（非multi_path只连接一个）
	
		os-brick在multi_path情形下会连接每一个可用路径，而如果不启用multi_path配置，那么只要连接上一个路径就OK了

		os-brick 建立iscsi连接全过程：
		
			os_brick.initiator.connectors.iscsi.ISCSIConnector
			接口：def _connect_to_iscsi_portal(self, connection_properties):

			# 查询数据库是否有这个target的信息，没有添加（iscsiadm -m node -T target_iqn -p target_ips --interface <interface> --op new）
			iscsiadm -m node -T target_iqn -p target_ips

			# 如果iscsi需要密码进行登录，那么会更新账户和密码到iscsiadm数据库
			iscsiadm -m node -T target_iqn -p target_ips --op update -n <key> -v <value>  key包括node.session.auth.authmethod、node.session.auth.username、node.session.auth.password

			# 检查当前iscsi连接，如果当前连接中没有要连接的target，那么连接并且更新成自动连接
			iscsiadm -m session
			iscsiadm -m node -T target_iqn -p target_ips --login
			iscsiadm -m node -T target_iqn -p target_ips --op update -n node.startup -v automatic
	
	3.rescan连接和节点：
	
		iscsiadm -m node --rescan
		iscsiadm -m session --rescan

	4.输出设备文件地址（/dev/disk/by-path目录下的软连接文件名）

此时检查/dev/disk/by-path会出现若干（每个path对应一个）软连接文件。os-brick会扫描若干次，等待文件生成。

通过scsi_id生成一个scsi设备的wwn，这里0x83指的是设备的业格式(注意：这里由于多路径的缘故可能有多个设备，但是只生成一个)
	
	/lib/udev/scsi_id --page 0x83 --whitelisted	<device_path>

针对多路径设备需要以下而外的操作：
	接口API：	def _discover_mpath_device(self, device_wwn, connection_properties,device_name)
	
	1.多路径时在以下两个路径查找设备（只要找到一个就OK）：
		/dev/disk/by-id/dm-uuid-mpath-%(wwn)
		/dev/mapper/%(wwn)

	2.获取设备的wwn和设备路径：
		multipath_id ：设备的wwn
		device_path ： 设备的文件路径“/dev/mapper/<wwn>”
		如果上面的信息无法获取相关信息，那么会调用multipath -l </dev/sdc>命令，
		从返回值中获取（os_brick.initiator.connectors.base.BaseLinuxConnector._discover_mpath_device）
		
			1.通过multipath -l </dev/sdc> 获取wwn
			2.执行校验设备状态 stat /dev/mapper/<wwn>
			
	3.如果磁盘不是ro模式的，需要对磁盘进行读写设定（os_brick.initiator.connectors.linuxscsi.LinuxSCSI.wait_for_rw）
	
		1.执行lsblk命令（该命令列出所有块设备信息，并且指出该设备是否是只读设备。参考：http://man.linuxde.net/lsblk）
			lsblk -o NAME,RO -l -n 
		  os-brick通过该命令检查磁盘是否是只读磁盘。如果是直接返回，如果不是讲磁盘文件设定为只读。
		
		
