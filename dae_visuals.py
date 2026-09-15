"""Split Nero Collada visuals by material so MuJoCo can keep the original colors.

STL collision meshes have no color. The official DAE files do: mostly near-black
plastic, silver bands, white logos, and a red accent. This writes one STL per
material into models/visual/.
"""

from __future__ import annotations

import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
DAE_DIR = Path(__file__).resolve().parent.parent / "agx_arm_urdf" / "nero" / "meshes" / "dae"
VISUAL_DIR = Path(__file__).resolve().parent / "models" / "visual"

LINKS = [
    "base_link",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "link7",
]


def _local(tag: str) -> str:
    return tag.split("}", 1)[-1]


def _text(el: ET.Element | None) -> str:
    return (el.text or "").strip() if el is not None else ""


def _parse_effects(root: ET.Element) -> dict[str, tuple[float, float, float, float]]:
    colors: dict[str, tuple[float, float, float, float]] = {}
    for effect in root.findall(".//c:library_effects/c:effect", NS):
        eid = effect.get("id") or ""
        color_el = effect.find(".//c:diffuse/c:color", NS)
        vals = [float(x) for x in _text(color_el).split()] if color_el is not None else []
        if len(vals) == 3:
            vals.append(1.0)
        if len(vals) != 4:
            vals = [0.5, 0.5, 0.5, 1.0]
        colors[eid] = (vals[0], vals[1], vals[2], vals[3])
        colors["#" + eid] = colors[eid]
    return colors


def _parse_materials(root: ET.Element, effects: dict) -> dict[str, tuple[float, float, float, float]]:
    mats: dict[str, tuple[float, float, float, float]] = {}
    for material in root.findall(".//c:library_materials/c:material", NS):
        mid = material.get("id") or ""
        inst = material.find("c:instance_effect", NS)
        url = (inst.get("url") if inst is not None else "") or ""
        rgba = effects.get(url, effects.get(url.lstrip("#"), (0.5, 0.5, 0.5, 1.0)))
        mats[mid] = rgba
        mats[material.get("name") or mid] = rgba
    return mats


def _floats(text: str) -> list[float]:
    return [float(x) for x in text.split()]


def _write_stl(path: Path, tris: list[tuple[tuple[float, float, float], ...]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    buf = bytearray(80)
    buf.extend(struct.pack("<I", len(tris)))
    for a, b, c in tris:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        nlen = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
        buf.extend(struct.pack("<12fH", nx / nlen, ny / nlen, nz / nlen, *a, *b, *c, 0))
    path.write_bytes(buf)


def convert_dae(link: str) -> list[dict]:
    dae_path = DAE_DIR / f"{link}.dae"
    if not dae_path.exists():
        raise FileNotFoundError(dae_path)
    root = ET.parse(dae_path).getroot()
    effects = _parse_effects(root)
    materials = _parse_materials(root, effects)

    groups: dict[tuple[float, float, float, float], list] = {}
    for geom in root.findall(".//c:library_geometries/c:geometry", NS):
        mesh = geom.find("c:mesh", NS)
        if mesh is None:
            continue
        sources: dict[str, list[float]] = {}
        for source in mesh.findall("c:source", NS):
            sid = source.get("id") or ""
            arr = source.find("c:float_array", NS)
            sources[sid] = _floats(_text(arr))
            sources["#" + sid] = sources[sid]
        vertices = mesh.find("c:vertices", NS)
        pos_url = ""
        if vertices is not None:
            for inp in vertices:
                if _local(inp.tag) == "input" and inp.get("semantic") == "POSITION":
                    pos_url = inp.get("source") or ""
                    break
        positions = sources.get(pos_url, sources.get(pos_url.lstrip("#"), []))
        if not positions:
            continue

        for tri in mesh.findall("c:triangles", NS):
            mat_id = tri.get("material") or ""
            rgba = materials.get(mat_id, (0.5, 0.5, 0.5, 1.0))
            key = tuple(round(c, 4) for c in rgba)
            inputs = tri.findall("c:input", NS)
            stride = max((int(i.get("offset") or 0) for i in inputs), default=0) + 1
            v_off = 0
            for inp in inputs:
                if inp.get("semantic") == "VERTEX":
                    v_off = int(inp.get("offset") or 0)
            idx = [int(x) for x in _text(tri.find("c:p", NS)).split()]
            tris = groups.setdefault(key, [])
            for i in range(0, len(idx), stride * 3):
                verts = []
                for k in range(3):
                    vi = idx[i + k * stride + v_off]
                    verts.append(
                        (
                            positions[3 * vi],
                            positions[3 * vi + 1],
                            positions[3 * vi + 2],
                        )
                    )
                tris.append(tuple(verts))

    out: list[dict] = []
    for i, (rgba, tris) in enumerate(groups.items()):
        if not tris:
            continue
        name = f"{link}_visual_{i}"
        rel = Path("visual") / f"{name}.stl"
        _write_stl(VISUAL_DIR / f"{name}.stl", tris)
        out.append({"name": name, "file": rel.as_posix(), "rgba": rgba, "ntri": len(tris)})
    return out


def convert_all() -> dict[str, list[dict]]:
    VISUAL_DIR.mkdir(parents=True, exist_ok=True)
    result = {}
    for link in LINKS:
        parts = convert_dae(link)
        result[link] = parts
        summary = ", ".join(f"{p['rgba']} x{p['ntri']}" for p in parts)
        print(f"{link}: {len(parts)} materials ({summary})")
    return result


if __name__ == "__main__":
    convert_all()
