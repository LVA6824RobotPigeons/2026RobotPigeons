"""Schema loader utilities for BCNP autonomous planner contract."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Iterable
import zlib


_TYPE_SIZES = {
    "int8": 1,
    "uint8": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "float32": 4,
    "int64": 8,
    "uint64": 8,
    "float64": 8,
}


@dataclass(frozen=True)
class SchemaMessage:
    """A single message entry from the canonical BCNP schema."""

    id: int
    name: str
    field_types: tuple[str, ...]

    @property
    def wire_size_bytes(self) -> int:
        return sum(_TYPE_SIZES[field_type] for field_type in self.field_types)


@dataclass(frozen=True)
class SchemaContract:
    """Loaded schema with message lookups by name and id."""

    version: str
    namespace: str
    by_name: Dict[str, SchemaMessage]
    by_id: Dict[int, SchemaMessage]

    def wire_sizes_by_id(self) -> Dict[int, int]:
        return {message_id: message.wire_size_bytes for message_id, message in self.by_id.items()}


def canonical_schema_bytes(raw: bytes) -> bytes:
    """Mirror BCNP codegen canonical schema serialization for hash compatibility."""

    doc = json.loads(raw.decode("utf-8"))
    canonical = {
        "version": doc["version"],
        "messages": [],
    }
    for message in sorted(doc["messages"], key=lambda m: m["id"]):
        canonical_message = {
            "id": message["id"],
            "name": message["name"],
            "fields": [],
        }
        for field in message.get("fields", []):
            canonical_field = {
                "name": field["name"],
                "type": field["type"],
            }
            if "scale" in field:
                canonical_field["scale"] = field["scale"]
            canonical_message["fields"].append(canonical_field)
        canonical["messages"].append(canonical_message)

    return json.dumps(canonical, separators=(",", ":"), sort_keys=True).encode("utf-8")


def schema_crc32(raw: bytes) -> int:
    """Compute unsigned 32-bit CRC for BCNP canonicalized schema payload."""

    return zlib.crc32(canonical_schema_bytes(raw)) & 0xFFFFFFFF


def load_contract(path: Path | str) -> SchemaContract:
    raw = Path(path).read_bytes()
    doc = json.loads(raw.decode("utf-8"))
    messages = tuple(_load_messages(doc["messages"]))
    by_name = {message.name: message for message in messages}
    by_id = {message.id: message for message in messages}
    return SchemaContract(
        version=str(doc.get("version", "")),
        namespace=str(doc.get("namespace", "")),
        by_name=by_name,
        by_id=by_id,
    )


def _load_messages(messages: Iterable[dict]) -> Iterable[SchemaMessage]:
    for message in messages:
        field_types = []
        for field in message.get("fields", []):
            field_type = str(field["type"])
            if field_type not in _TYPE_SIZES:
                raise ValueError(f"Unsupported schema type '{field_type}' in message '{message.get('name')}'")
            field_types.append(field_type)

        yield SchemaMessage(
            id=int(message["id"]),
            name=str(message["name"]),
            field_types=tuple(field_types),
        )
