"""Parameter registry: every number the simulator uses, with where it came from.

    status "verified"  manufacturer specification
           "measured"  this vehicle: its config, or our own tests
           "assumed"   temporary placeholder for an UNKNOWN value (has a range;
                       sweep it, don't trust it; replace by measuring)

The simulator code never hard-codes a physical number: it reads them from here.
"""
from __future__ import annotations

import copy
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PROFILE = os.path.join(HERE, "profile.json")
STATUSES = ("verified", "measured", "assumed", "estimated")  # estimated = fitted/derived from real data, not measured


class Profile:
    def __init__(self, entries: dict, path: str | None = None):
        self._e = {k: dict(v) for k, v in entries.items() if not k.startswith("_") and isinstance(v, dict)}
        self.path = path

    # -------------------------------------------------- load / save
    @classmethod
    def load(cls, path: str | None = None) -> "Profile":
        path = path or DEFAULT_PROFILE
        with open(path) as f:
            return cls(json.load(f), path)

    def save(self, path: str | None = None):
        path = path or self.path
        with open(path) as f:
            raw = json.load(f)
        for k, e in self._e.items():
            raw[k] = e
        with open(path, "w") as f:
            json.dump(raw, f, indent=2)

    # -------------------------------------------------- access
    def __getattr__(self, name):
        e = self.__dict__.get("_e", {}).get(name)
        if e is None:
            raise AttributeError(f"profile has no parameter '{name}'")
        return e["value"]

    def __contains__(self, name):
        return name in self._e

    def entry(self, name) -> dict:
        return self._e[name]

    def names(self, status=None):
        return [k for k, e in self._e.items() if status is None or e.get("status") == status]

    def with_values(self, **values) -> "Profile":
        """Copy with some values replaced (status kept: an assumed value stays assumed)."""
        p = Profile(copy.deepcopy(self._e), self.path)
        for k, v in values.items():
            if k not in p._e:
                raise KeyError(f"unknown parameter '{k}'")
            p._e[k]["value"] = v
            p._e[k]["overridden"] = True
        return p

    def set_measured(self, name, value, source):
        """Record a real measurement: value + how it was measured; status becomes 'measured'."""
        e = self._e[name]
        e.update(value=value, status="measured", source=source)
        e.pop("overridden", None)

    # -------------------------------------------------- reporting
    def assumed_in_use(self):
        return self.names("assumed")

    def report(self) -> str:
        lines = []
        for st in STATUSES:
            ks = self.names(st)
            if not ks:
                continue
            lines.append(f"[{st.upper()}] ({len(ks)})")
            for k in ks:
                e = self._e[k]
                rng = f"  range {e['range']}" if "range" in e else ""
                unit = f" {e['unit']}" if "unit" in e else ""
                lines.append(f"  {k:<20} {json.dumps(e['value']) if not isinstance(e['value'], str) else e['value']}{unit}{rng}")
                lines.append(f"  {'':<20} source: {e.get('source', '?')}")
        return "\n".join(lines)

    def as_dict(self):
        return copy.deepcopy(self._e)


if __name__ == "__main__":
    print(Profile.load().report())
