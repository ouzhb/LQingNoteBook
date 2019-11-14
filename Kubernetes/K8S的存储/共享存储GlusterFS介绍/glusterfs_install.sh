#!/usr/bin/env bash

cat << EOF > /tmp/host_rgibns
[rgibns]
rgibns1
rgibns2
rgibns3
EOF

ansible -i /tmp/host_rgibns rgibns -m shell -a "add-apt-repository -y ppa:gluster/glusterfs-4.1"
ansible -i /tmp/host_rgibns rgibns -m shell -a "apt-get update"
ansible -i /tmp/host_rgibns rgibns -m shell -a "apt-get -y install glusterfs-server"
ansible -i /tmp/host_rgibns rgibns -m shell -a "systemctl start glusterd"
ansible -i /tmp/host_rgibns rgibns -m shell -a "systemctl enable glusterd"
ansible -i /tmp/host_rgibns rgibns -m shell -a "mkdir -p /ibnsdata/gluster/gv0"

gluster peer probe rgibns2
gluster peer probe rgibns3
gluster peer status

gluster volume create gv0 replica 3 rgibns1:/ibnsdata/gluster/gv0 rgibns2:/ibnsdata/gluster/gv0 rgibns3:/ibnsdata/gluster/gv0

gluster volume start gv0