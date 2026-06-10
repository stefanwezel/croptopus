"""Plain data structures shared across the applier.

These are intentionally framework-free (no Flask, no DB driver) so the whole
applier package can be lifted out into a standalone ``projectctl`` later.
"""

from dataclasses import dataclass, field, fields, asdict
from enum import Enum
from typing import Any, Optional


class Action(str, Enum):
    """Classification of a single resource in a plan."""

    CREATE = "CREATE"        # in manifest, not live
    UPDATE = "UPDATE"        # in both, differs
    NO_CHANGE = "NO_CHANGE"  # in both, identical
    ORPHAN = "ORPHAN"        # in live, not in manifest (never deleted in v1)
    SKIP = "SKIP"            # referenced by manifest but not actionable (e.g. unknown dashboard)


# Actions that cause apply to actually touch infrastructure.
ACTIONABLE = {Action.CREATE, Action.UPDATE}


@dataclass
class ResourceChange:
    """One row in a plan: what we intend to do to one resource."""

    section: str                       # "timescale" | "grafana"
    kind: str                          # schema|table|retention_policy|compression_policy|folder|datasource|dashboard
    name: str                          # human identity of the resource
    action: str                        # one of Action values
    detail: str = ""                   # human-readable description
    sql: Optional[str] = None          # the SQL we'd run, where useful for transparency
    destructive: bool = False          # true => apply will refuse it
    warning: Optional[str] = None      # surfaced on the plan page
    op: Optional[str] = None           # applier method to invoke on apply
    params: dict = field(default_factory=dict)  # arguments for that method

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


@dataclass
class Plan:
    """The full diff between a manifest and live infrastructure."""

    manifest_id: str
    manifest_hash: str
    customer_id: str
    project_id: str
    schema: str
    folder_name: str = ""
    changes: list = field(default_factory=list)   # list[ResourceChange]
    warnings: list = field(default_factory=list)   # list[str]
    stubs: list = field(default_factory=list)      # sections deferred ("coming soon")
    plan_hash: str = ""

    # --- convenience accessors used by templates --------------------------
    def section(self, name):
        return [c for c in self.changes if c.section == name]

    @property
    def has_destructive(self):
        return any(c.destructive for c in self.changes)

    @property
    def actionable_count(self):
        return sum(1 for c in self.changes if Action(c.action) in ACTIONABLE)

    def summary(self):
        counts = {}
        for c in self.changes:
            counts[c.action] = counts.get(c.action, 0) + 1
        parts = [f"{v} {k.lower()}" for k, v in sorted(counts.items())]
        return ", ".join(parts) if parts else "no changes"

    # --- (de)serialisation for session storage ----------------------------
    def to_dict(self):
        d = asdict(self)
        d["changes"] = [c.to_dict() for c in self.changes]
        return d

    @classmethod
    def from_dict(cls, d):
        # drop unknown keys so a session-stashed plan from an older schema
        # (e.g. pre-rename org_name) degrades to "stale plan" instead of a 500
        known = {f.name for f in fields(cls)}
        d = {k: v for k, v in d.items() if k in known}
        d["changes"] = [ResourceChange.from_dict(c) for c in d.get("changes", [])]
        return cls(**d)


@dataclass
class ResourceResult:
    """Outcome of applying one ResourceChange."""

    name: str
    kind: str
    action: str
    status: str           # "ok" | "error" | "skipped"
    message: str = ""

    def to_dict(self):
        return asdict(self)


@dataclass
class ApplyResult:
    """Outcome of applying a whole plan."""

    manifest_id: str
    manifest_hash: str
    results: list = field(default_factory=list)  # list[ResourceResult]

    @property
    def success(self):
        return all(r.status != "error" for r in self.results)

    def to_dict(self):
        return {
            "manifest_id": self.manifest_id,
            "manifest_hash": self.manifest_hash,
            "success": self.success,
            "results": [r.to_dict() for r in self.results],
        }
