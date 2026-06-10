"""Grafana applier package.

Kept import-light on purpose: the concrete applier (dashboard_applier) pulls
in ``requests``, so it's imported lazily by applier.build_appliers rather
than at package import. This lets the pure plan logic and its unit tests run
without the HTTP/DB drivers installed.
"""
