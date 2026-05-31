"""Exceptions for the applier layer."""


class ApplierError(Exception):
    """Base class for all applier errors."""


class RefusedDestructive(ApplierError):
    """Raised whenever a destructive operation is requested.

    In v1 the applier never deletes infrastructure. Orphaned resources are
    surfaced in plans but their removal is refused until the decommission
    flow is designed.
    """

    def __init__(self, message="not yet supported, see decommission flow in roadmap"):
        super().__init__(message)


class StateReadError(ApplierError):
    """Raised when live state cannot be read (connection refused, auth, ...)."""
