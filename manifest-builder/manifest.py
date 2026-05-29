"""Manifest schema, YAML I/O, validation, and form-to-dict parsing.

The manifest is a single nested dict. We keep YAML key order stable on dump
(top-level: customer, project, gateways, nodes, database, grafana, alerts)
so round-tripping a manifest produces a byte-stable file.
"""

from __future__ import annotations

import re
from collections import OrderedDict
from typing import Any

import yaml

TOP_LEVEL_ORDER = [
    "customer",
    "project",
    "gateways",
    "nodes",
    "database",
    "grafana",
    "alerts",
]


def empty_manifest() -> dict[str, Any]:
    """A blank-but-well-shaped manifest for 'start from scratch'."""
    return {
        "customer": {"id": "", "name": ""},
        "project": {
            "id": "",
            "name": "",
            "environment": "dev",
            "timezone": "Europe/Berlin",
        },
        "gateways": [],
        "nodes": [],
        "database": {
            "schema": "croptopus",
            "retention_days": 365,
            "compression_after_days": 30,
        },
        "grafana": {
            "folder": "Croptopus",
            "datasource_name": "croptopus_tsdb",
            "dashboards": [],
        },
        "alerts": {},
    }


# --- YAML I/O ---------------------------------------------------------------

def _ordered_top_level(manifest: dict[str, Any]) -> "OrderedDict[str, Any]":
    """Return a copy with top-level keys in TOP_LEVEL_ORDER (extras appended)."""
    out: "OrderedDict[str, Any]" = OrderedDict()
    for key in TOP_LEVEL_ORDER:
        if key in manifest:
            out[key] = manifest[key]
    for key, value in manifest.items():
        if key not in out:
            out[key] = value
    return out


class _OrderedDumper(yaml.SafeDumper):
    pass


def _ordered_dict_representer(dumper: yaml.SafeDumper, data: "OrderedDict[str, Any]"):
    return dumper.represent_mapping("tag:yaml.org,2002:map", data.items())


_OrderedDumper.add_representer(OrderedDict, _ordered_dict_representer)


def dump_yaml(manifest: dict[str, Any]) -> str:
    """Serialize the manifest to a clean, stably-ordered YAML string."""
    ordered = _ordered_top_level(manifest)
    return yaml.dump(
        ordered,
        Dumper=_OrderedDumper,
        sort_keys=False,
        default_flow_style=False,
        allow_unicode=True,
        indent=2,
    )


def load_yaml(text: str) -> dict[str, Any]:
    """Parse a YAML manifest. Raises ValueError on bad YAML / wrong shape."""
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ValueError(f"YAML parse error: {e}") from e
    if not isinstance(data, dict):
        raise ValueError("Manifest root must be a mapping.")
    return data


# --- Validation -------------------------------------------------------------

def validate(manifest: dict[str, Any]) -> list[str]:
    """Return a list of human-readable validation errors (empty if OK).

    We don't try to be exhaustive — surface the obvious problems clearly.
    """
    errs: list[str] = []

    cust = manifest.get("customer") or {}
    if not cust.get("id"):
        errs.append("customer.id is required")
    if not cust.get("name"):
        errs.append("customer.name is required")

    proj = manifest.get("project") or {}
    if not proj.get("id"):
        errs.append("project.id is required")
    if not proj.get("name"):
        errs.append("project.name is required")
    if proj.get("environment") not in ("prod", "staging", "dev"):
        errs.append("project.environment must be one of prod/staging/dev")

    gw_ids: set[str] = set()
    for i, gw in enumerate(manifest.get("gateways") or []):
        if not gw.get("id"):
            errs.append(f"gateways[{i}].id is required")
        elif gw["id"] in gw_ids:
            errs.append(f"gateways[{i}].id '{gw['id']}' is duplicated")
        else:
            gw_ids.add(gw["id"])
        loc = gw.get("location") or {}
        for k in ("lat", "lon"):
            if loc.get(k) in (None, "") or not isinstance(loc.get(k), (int, float)):
                errs.append(f"gateways[{i}].location.{k} must be a number")

    node_ids: set[str] = set()
    for i, n in enumerate(manifest.get("nodes") or []):
        if not n.get("device_id"):
            errs.append(f"nodes[{i}].device_id is required")
        elif n["device_id"] in node_ids:
            errs.append(f"nodes[{i}].device_id '{n['device_id']}' is duplicated")
        else:
            node_ids.add(n["device_id"])

        gid = n.get("gateway_id")
        if not gid:
            errs.append(f"nodes[{i}].gateway_id is required")
        elif gid not in gw_ids:
            errs.append(
                f"nodes[{i}].gateway_id '{gid}' does not match any gateway.id"
            )

        samp = n.get("sampling") or {}
        for k in ("interval_sec", "transmit_every_n_samples"):
            v = samp.get(k)
            if not isinstance(v, int) or v <= 0:
                errs.append(f"nodes[{i}].sampling.{k} must be a positive integer")

        for j, s in enumerate(n.get("sensors") or []):
            if not s.get("type"):
                errs.append(f"nodes[{i}].sensors[{j}].type is required")

    return errs


# --- Form-to-manifest parsing ----------------------------------------------
# Forms use bracket-notation names, e.g. gateways[0][location][lat]. We pull
# values by explicit lookup rather than a generic bracket parser — keeps the
# code obvious and makes it easy to coerce types per field.

_INT_RE = re.compile(r"^-?\d+$")
_FLOAT_RE = re.compile(r"^-?\d+(\.\d+)?$")


def _str_or_none(v: str | None) -> str | None:
    if v is None:
        return None
    v = v.strip()
    return v or None


def _int_or_none(v: str | None) -> int | None:
    if v is None:
        return None
    v = v.strip()
    if not v:
        return None
    if not _INT_RE.match(v):
        raise ValueError(f"expected integer, got {v!r}")
    return int(v)


def _float_or_none(v: str | None) -> float | int | None:
    """Parse a numeric field. Returns int if the input has no decimal point,
    else float — keeps `threshold: 10` from drifting to `10.0` on round-trip."""
    if v is None:
        return None
    v = v.strip()
    if not v:
        return None
    if not _FLOAT_RE.match(v):
        raise ValueError(f"expected number, got {v!r}")
    return int(v) if "." not in v else float(v)


def _bool_from_checkbox(form, key: str) -> bool:
    return form.get(key, "") == "on"


def _indices_for(form, prefix: str) -> list[int]:
    """Find all numeric indices used for keys like 'prefix[N]...'."""
    pat = re.compile(rf"^{re.escape(prefix)}\[(\d+)\]")
    seen: set[int] = set()
    for k in form.keys():
        m = pat.match(k)
        if m:
            seen.add(int(m.group(1)))
    return sorted(seen)


def parse_form(form) -> dict[str, Any]:
    """Convert a submitted editor form into a manifest dict.

    Raises ValueError with a friendly message on any coercion failure.
    """
    m: dict[str, Any] = {
        "customer": {
            "id": form.get("customer[id]", "").strip(),
            "name": form.get("customer[name]", "").strip(),
        },
        "project": {
            "id": form.get("project[id]", "").strip(),
            "name": form.get("project[name]", "").strip(),
            "environment": form.get("project[environment]", "dev").strip(),
            "timezone": form.get("project[timezone]", "").strip(),
        },
        "gateways": [],
        "nodes": [],
        "database": {
            "schema": form.get("database[schema]", "").strip(),
            "retention_days": _int_or_none(form.get("database[retention_days]")) or 0,
            "compression_after_days": _int_or_none(
                form.get("database[compression_after_days]")
            )
            or 0,
        },
        "grafana": {
            "folder": form.get("grafana[folder]", "").strip(),
            "datasource_name": form.get("grafana[datasource_name]", "").strip(),
            "dashboards": [],
        },
        "alerts": {},
    }

    for i in _indices_for(form, "gateways"):
        m["gateways"].append(
            {
                "id": form.get(f"gateways[{i}][id]", "").strip(),
                "ip": _str_or_none(form.get(f"gateways[{i}][ip]")),
                "lorawan_region": form.get(
                    f"gateways[{i}][lorawan_region]", ""
                ).strip(),
                "site_name": form.get(f"gateways[{i}][site_name]", "").strip(),
                "location": {
                    "lat": _float_or_none(form.get(f"gateways[{i}][location][lat]")),
                    "lon": _float_or_none(form.get(f"gateways[{i}][location][lon]")),
                },
            }
        )

    for i in _indices_for(form, "nodes"):
        sensors = []
        for j in _indices_for(form, f"nodes[{i}][sensors]"):
            sensors.append(
                {
                    "type": form.get(f"nodes[{i}][sensors][{j}][type]", "").strip(),
                    "channel": form.get(
                        f"nodes[{i}][sensors][{j}][channel]", ""
                    ).strip(),
                }
            )
        m["nodes"].append(
            {
                "device_id": form.get(f"nodes[{i}][device_id]", "").strip(),
                "hw_id": _str_or_none(form.get(f"nodes[{i}][hw_id]")),
                "gateway_id": form.get(f"nodes[{i}][gateway_id]", "").strip(),
                "role": form.get(f"nodes[{i}][role]", "").strip(),
                "location": {
                    "lat": _float_or_none(form.get(f"nodes[{i}][location][lat]")),
                    "lon": _float_or_none(form.get(f"nodes[{i}][location][lon]")),
                    "description": form.get(
                        f"nodes[{i}][location][description]", ""
                    ).strip(),
                },
                "sensors": sensors,
                "sampling": {
                    "interval_sec": _int_or_none(
                        form.get(f"nodes[{i}][sampling][interval_sec]")
                    ),
                    "transmit_every_n_samples": _int_or_none(
                        form.get(f"nodes[{i}][sampling][transmit_every_n_samples]")
                    ),
                },
            }
        )

    for i in _indices_for(form, "grafana[dashboards]"):
        m["grafana"]["dashboards"].append(
            {
                "id": form.get(f"grafana[dashboards][{i}][id]", "").strip(),
                "enabled": _bool_from_checkbox(
                    form, f"grafana[dashboards][{i}][enabled]"
                ),
            }
        )

    for i in _indices_for(form, "alerts"):
        name = form.get(f"alerts[{i}][name]", "").strip()
        if not name:
            continue
        alert: dict[str, Any] = {
            "enabled": _bool_from_checkbox(form, f"alerts[{i}][enabled]"),
        }
        threshold = _float_or_none(form.get(f"alerts[{i}][threshold]"))
        if threshold is not None:
            alert["threshold"] = threshold
        duration = _int_or_none(form.get(f"alerts[{i}][duration_min]"))
        if duration is not None:
            alert["duration_min"] = duration
        m["alerts"][name] = alert

    return m
