"""Bearer-token authentication + schema routing.

Requests must send `Authorization: Bearer <token>`. The token identifies the
project and resolves to the Postgres schema its data lands in:

1. The legacy shared INGEST_TOKEN env var maps to the original `croptopus`
   schema (override with INGEST_LEGACY_SCHEMA) — keeps existing gateways
   working unchanged.
2. Otherwise the token is looked up in registry.projects, which the
   manifest-builder applier maintains: one token per project, generated on
   first Apply.

Used as a FastAPI dependency that returns the target schema name.
"""

from __future__ import annotations

import hmac
import os

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from . import db

# auto_error=False so we can return a uniform 401 (instead of FastAPI's 403)
# whether the header is missing or malformed.
_bearer = HTTPBearer(auto_error=False)

_LEGACY_SCHEMA = "croptopus"


async def require_token(
    credentials: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> str:
    """Return the target schema for the presented token, or reject with 401.

    Env vars are read at request time so tests can set INGEST_TOKEN via
    monkeypatch. compare_digest avoids leaking length/content via timing.
    """
    presented = credentials.credentials if credentials else ""

    if presented:
        legacy = os.environ.get("INGEST_TOKEN", "")
        if legacy and hmac.compare_digest(presented, legacy):
            return os.environ.get("INGEST_LEGACY_SCHEMA", _LEGACY_SCHEMA)

        schema = await db.resolve_token(presented)
        if schema:
            return schema

    raise HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="invalid or missing bearer token",
        headers={"WWW-Authenticate": "Bearer"},
    )
