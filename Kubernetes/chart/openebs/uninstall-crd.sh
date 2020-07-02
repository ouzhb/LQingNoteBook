#!/usr/bin/env bash

kubectl delete crd castemplates.openebs.io
kubectl delete crd cstorpools.openebs.io
kubectl delete crd cstorpoolinstances.openebs.io
kubectl delete crd cstorvolumeclaims.openebs.io
kubectl delete crd cstorvolumereplicas.openebs.io
kubectl delete crd cstorvolumepolicies.openebs.io
kubectl delete crd cstorvolumes.openebs.io
kubectl delete crd runtasks.openebs.io
kubectl delete crd storagepoolclaims.openebs.io
kubectl delete crd storagepools.openebs.io
kubectl delete crd volumesnapshotdatas.volumesnapshot.external-storage.k8s.io
kubectl delete crd volumesnapshots.volumesnapshot.external-storage.k8s.io
kubectl delete crd disks.openebs.io
kubectl delete crd blockdevices.openebs.io
kubectl delete crd blockdeviceclaims.openebs.io
kubectl delete crd cstorbackups.openebs.io
kubectl delete crd cstorrestores.openebs.io
kubectl delete crd cstorcompletedbackups.openebs.io
kubectl delete crd cstorpoolclusters.openebs.io
kubectl delete crd upgradetasks.openebs.io
