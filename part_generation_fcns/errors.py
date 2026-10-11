"""Geometry check failures. Still ValueErrors, but with a stable code so the simple-mode
solver can react to a failed check without matching message text."""


class GeometryError(ValueError):
    def __init__(self, code: str, msg: str):
        super().__init__(msg)
        self.code = code
