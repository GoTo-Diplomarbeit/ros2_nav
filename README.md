# GoTo

## Prerequisites

Install Docker Compose (rooted):

```shell
bash scripts/setup/install-docker.sh
```

Install hooks:

```shell
bash scripts/setup/install-hooks.sh
```

NVIDIA GPU only - install the NVIDIA Container Toolkit, then use `release-nvidia` instead of `release` (scripts: `SERVICE=release-nvidia bash scripts/...` or the `(nvidia)` run configs):

```shell
bash scripts/setup/install-nvidia.sh
```

## Installation

```shell
docker compose pull release
```

## Verify

```shell
docker compose up -d release
docker compose ps
```

The `release` container should show as `running`.

## Run

### Build and run project

```shell
bash scripts/build_run_project.sh
```

### Launch Gazebo

```shell
bash scripts/launch_gazebo.sh
```
