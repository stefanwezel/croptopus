# Croptopus manifest-builder image (the control plane).
#
# Pattern mirrors server/ingester: python:3.12-slim + uv, dependencies installed
# system-wide from pyproject, run as a non-root user. Served by gunicorn — NOT
# the Flask dev server.
#
# The applier's runtime state (Grafana service-account tokens + the apply audit
# DB) lives in /app/instance. Mount a PERSISTENT volume there in Coolify, or
# tokens and history are lost on every redeploy.
FROM python:3.12-slim

# uv: fast installer/resolver (single static binary from the official image).
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app

# Install deps first for layer caching. `-r pyproject.toml` installs exactly the
# [project].dependencies (no test deps, no project packaging step).
COPY pyproject.toml ./
RUN uv pip install --system --no-cache -r pyproject.toml

# App code (see .dockerignore — .env, instance/, .venv are never copied in).
COPY . .

# Non-root user that owns the writable instance dir.
RUN useradd --create-home --uid 10001 appuser \
    && mkdir -p /app/instance \
    && chown -R appuser:appuser /app/instance
USER appuser

EXPOSE 8000

# Single worker on purpose: the working manifest is held in-process (this is a
# single-user internal tool), so multiple workers would see divergent state.
# A generous timeout covers slow Plan/Apply calls against Timescale/Grafana.
CMD ["gunicorn", "--bind", "0.0.0.0:8000", "--workers", "1", "--timeout", "120", "app:app"]
