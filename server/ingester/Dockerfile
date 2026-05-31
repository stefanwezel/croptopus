# Croptopus ingester image.
# Pattern: python:3.12-slim + uv, install deps system-wide from pyproject, copy code,
# run uvicorn as a non-root user.
FROM python:3.12-slim

# uv: fast installer/resolver (single static binary copied from the official image).
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app

# Install dependencies first for layer caching. uv reads [project].dependencies.
COPY pyproject.toml ./
RUN uv pip install --system --no-cache .

# App code.
COPY app ./app

# Run as non-root.
RUN useradd --create-home --uid 10001 appuser
USER appuser

EXPOSE 8000

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
