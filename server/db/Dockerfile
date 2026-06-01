# Timescale image with the croptopus schema baked in.
#
# Why a custom image instead of a bind mount: Coolify doesn't reliably mount the
# repo's ./db/init into the container at runtime (the repo checkout path is
# ephemeral), so `- ./db/init:/docker-entrypoint-initdb.d` silently became an
# empty dir and the schema never got created. Baking the SQL into the image is
# Coolify-proof and keeps the repo the source of truth.
#
# Postgres runs everything in /docker-entrypoint-initdb.d ONCE, on first boot
# against an empty data volume.
FROM timescale/timescaledb:latest-pg16

COPY init/ /docker-entrypoint-initdb.d/
