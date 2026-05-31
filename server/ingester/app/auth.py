"""Shared bearer-token authentication.

The token comes from the INGEST_TOKEN env var. Requests must send
`Authorization: Bearer <token>`. Used as a FastAPI dependency so endpoints stay clean.
"""

from __future__ import annotations

import hmac
import os

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

# auto_error=False so we can return a uniform 401 (instead of FastAPI's 403)
# whether the header is missing or malformed.
_bearer = HTTPBearer(auto_error=False)


def require_token(
    credentials: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> None:
    """Reject (401) unless a correct bearer token is present.

    The server's token is read at request time so tests can set INGEST_TOKEN
    via monkeypatch. compare_digest avoids leaking length/content via timing.
    """
    expected = os.environ.get("INGEST_TOKEN", "")
    presented = credentials.credentials if credentials else ""

    if not expected or not presented or not hmac.compare_digest(presented, expected):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="invalid or missing bearer token",
            headers={"WWW-Authenticate": "Bearer"},
        )
