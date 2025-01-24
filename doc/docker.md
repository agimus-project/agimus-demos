# Docker installation

Before you begin, ensure that Docker is installed on your system. Follow the official Docker installation guide if it is not already installed: [Install Docker Engine](https://docs.docker.com/engine/install/).

```bash
# Pull the necessary Docker repositories
docker pull gitlab.laas.fr:4567/agimus-project/agimus_dev_container:humble-devel-coding-week-prague-update-base
docker pull gitlab.laas.fr:4567/agimus-project/agimus_dev_container:humble-devel-coding-week-prague-update-control
# clone repository
git clone --branch humble-devel-coding-week-prague https://gitlab.laas.fr/agimus-project/agimus_dev_container.git
```
