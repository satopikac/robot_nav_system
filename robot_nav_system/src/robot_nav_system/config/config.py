"""Configuration management with deployment profile support."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

from ..exceptions import ConfigError
from .defaults import DEFAULT_CONFIG


def _merge_dict(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Deep-merge *override* into *base*; override wins on conflict."""
    merged = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(merged.get(k), dict):
            merged[k] = _merge_dict(merged[k], v)
        else:
            merged[k] = v
    return merged


# Standard directory for profile JSON files
_PROFILES_DIR = Path(__file__).resolve().parent.parent.parent.parent / "config" / "profiles"


class Config:
    """Hierarchical configuration: DEFAULT_CONFIG ← profile.json ← overrides."""

    def __init__(self, data: Dict[str, Any]):
        self._data = data

    # ---- constructors ----

    @classmethod
    def from_profile(
        cls,
        profile_name: str,
        profiles_dir: Optional[Path] = None,
        overrides: Optional[Dict[str, Any]] = None,
    ) -> "Config":
        """Load a named deployment profile and merge with defaults.

        Lookup order: *profiles_dir* / ``<profile_name>.json``.
        """
        search_dir = Path(profiles_dir) if profiles_dir else _PROFILES_DIR
        profile_path = search_dir / f"{profile_name}.json"

        if not profile_path.exists():
            raise ConfigError(
                f"Profile '{profile_name}' not found at {profile_path}"
            )

        try:
            with profile_path.open("r", encoding="utf-8") as f:
                profile_data = json.load(f)
        except json.JSONDecodeError as e:
            raise ConfigError(f"Invalid profile JSON ({profile_name}): {e}") from e
        except OSError as e:
            raise ConfigError(f"Failed to read profile: {e}") from e

        merged = _merge_dict(DEFAULT_CONFIG, profile_data)
        if overrides:
            merged = _merge_dict(merged, overrides)
        return cls(merged)

    @classmethod
    def from_json(cls, path: str | Path) -> "Config":
        """Load a single JSON file and merge with defaults (legacy compat)."""
        path_obj = Path(path)
        if not path_obj.exists():
            raise ConfigError(f"Config file not found: {path_obj}")
        try:
            with path_obj.open("r", encoding="utf-8") as f:
                user_data = json.load(f)
        except json.JSONDecodeError as e:
            raise ConfigError(f"Invalid config JSON: {e}") from e
        except OSError as e:
            raise ConfigError(f"Failed to read config file: {e}") from e

        merged = _merge_dict(DEFAULT_CONFIG, user_data)
        return cls(merged)

    @classmethod
    def from_defaults(cls) -> "Config":
        """Create a Config with only the default values."""
        return cls(dict(DEFAULT_CONFIG))

    # ---- access ----

    def get(self, dotted_key: str, default: Any = None) -> Any:
        """Retrieve a value by dot-separated key path.

        Example: ``config.get("robots.global_frame")``
        """
        if not dotted_key:
            return default
        cur: Any = self._data
        for key in dotted_key.split("."):
            if not isinstance(cur, dict) or key not in cur:
                return default
            cur = cur[key]
        return cur

    def require(self, dotted_key: str) -> Any:
        """Like :meth:`get` but raises :class:`ConfigError` if missing."""
        value = self.get(dotted_key, None)
        if value in (None, ""):
            raise ConfigError(f"Required config missing: {dotted_key}")
        return value

    @property
    def data(self) -> Dict[str, Any]:
        return self._data

    @property
    def profile_name(self) -> str:
        return self._data.get("profile", "unknown")

    def __repr__(self) -> str:
        return f"Config(profile={self.profile_name!r})"
