"""Geometry check failures. Still ValueErrors, but with a stable code so the simple-mode
solver can react to a failed check without matching message text.

Checks report through geometry_issue(): it raises, unless the caller is collecting issues
(collecting_issues()), in which case the issue is recorded and the build carries on, so a
broken design can still be built and looked at."""
import contextvars
from contextlib import contextmanager


class GeometryError(ValueError):
    def __init__(self, code: str, msg: str):
        super().__init__(msg)
        self.code = code


_collector: contextvars.ContextVar = contextvars.ContextVar("geometry_issues", default=None)


def geometry_issue(code: str, msg: str) -> None:
    issues = _collector.get()
    if issues is None:
        raise GeometryError(code, msg)
    issues.append((code, msg))


@contextmanager
def collecting_issues():
    """Within this block geometry checks record (code, msg) into the yielded list."""
    issues: list[tuple[str, str]] = []
    token = _collector.set(issues)
    try:
        yield issues
    finally:
        _collector.reset(token)
