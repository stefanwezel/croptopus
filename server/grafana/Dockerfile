# Grafana image with the Timescale datasource + dashboards baked in.
#
# Same reason as db/Dockerfile: Coolify doesn't reliably mount the repo's
# ./grafana/provisioning at runtime, so provisioning silently did nothing
# ("can't read ... no such file or directory" in the logs). Baking it into the
# image is Coolify-proof.
#
# Grafana re-reads /etc/grafana/provisioning on every start. The ${POSTGRES_*}
# placeholders in datasources/timescale.yml are expanded at runtime from the
# container's environment (set on the grafana service in docker-compose.yml).
FROM grafana/grafana:latest

COPY provisioning/ /etc/grafana/provisioning/
