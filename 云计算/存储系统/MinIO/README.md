# 部署

在三节点环境中部署，每个节点启动一个Minio实例管理2个Brick，

```yaml
# run docker-compose -f minio.yml  up -d
version: '3.3'
services:
  minio:
    image: minio/minio:latest
    container_name: minio
    volumes:
      - "/data/minio/brick1:/data1"
      - "/data/minio/brick2:/data2"
    ports:
      - "9000:9000"
    environment:
      MINIO_ACCESS_KEY: admin
      MINIO_SECRET_KEY: omc_admin
    network_mode: host
    command: server http://node{1...3}:9000/data{1...2}
    network_mode: host
    restart: always
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:9000/minio/health/live"]
      interval: 30s
      timeout: 20s
      retries: 3
```