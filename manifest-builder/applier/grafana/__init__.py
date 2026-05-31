"""Grafana applier package.

Kept import-light on purpose: the concrete appliers (org_applier,
dashboard_applier) pull in ``requests``, so they're imported lazily by
applier.build_appliers rather than at package import. This lets the pure plan
logic and its unit tests run without the HTTP/DB drivers installed.
"""
