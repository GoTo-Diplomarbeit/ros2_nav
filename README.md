# GoTo

## Prerequisites

Install Docker Compose (rootless):

```shell
bash scripts/setup/install-docker.sh
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

```shell
bash scripts/run.sh
```
