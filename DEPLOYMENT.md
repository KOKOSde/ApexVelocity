# Deploying ApexVelocity to Kubernetes

This guide explains how to deploy the ApexVelocity HTTP API to a Kubernetes cluster
using the provided Helm chart.

## Prerequisites

- Kubernetes cluster (e.g., GKE, EKS, AKS, k3s, kind)
- Helm 3.x installed
- Access to the ApexVelocity container image in GHCR:
  - `ghcr.io/kokosde/apexvelocity:latest` (or a tagged version)

## Quick Start

From the repository root:

```bash
helm install apexvelocity ./charts/apexvelocity

kubectl get pods

kubectl port-forward svc/apexvelocity-apexvelocity 8080:8080

curl http://localhost:8080/health
```

By default, the chart:

- Runs 3 replicas of the ApexVelocity server.
- Exposes port 8080 internally via a ClusterIP service.
- Configures liveness and readiness probes on `/health`.
- Enables autoscaling between 2 and 10 replicas based on CPU.
- Configures an Ingress resource for `apexvelocity.example.com` (if enabled).

## Customization

Create a `my-values.yaml` file and override settings:

```yaml
image:
  repository: ghcr.io/kokosde/apexvelocity
  tag: v1.0.0

ingress:
  enabled: true
  hosts:
    - host: apexvelocity.mycompany.com
      paths:
        - path: /
          pathType: Prefix
```

Then install with:

```bash
helm install apexvelocity ./charts/apexvelocity -f my-values.yaml
```


