"""The robot's URDF as what the operator page needs to draw it: links with their meshes
and joints with their origins and axes."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET


def _floats(text: str | None, default):
    return [float(v) for v in text.split()] if text else list(default)


def _origin(element) -> dict:
    origin = element.find("origin") if element is not None else None
    if origin is None:
        return {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
    return {"xyz": _floats(origin.get("xyz"), (0, 0, 0)), "rpy": _floats(origin.get("rpy"), (0, 0, 0))}


def load(path: str) -> dict:
    """Parse a URDF. Mesh file names are returned relative to the URDF's directory."""
    root = ET.parse(path).getroot()
    materials = {}
    for material in root.iter("material"):
        color = material.find("color")
        if material.get("name") and color is not None:
            materials[material.get("name")] = _floats(color.get("rgba"), (0.7, 0.7, 0.7, 1))
    links = {}
    for link in root.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename", "")
            if filename.startswith("package://"):
                filename = filename.split("/", 3)[-1]
            material = visual.find("material")
            color = materials.get(material.get("name")) if material is not None else None
            visuals.append({"mesh": filename, **_origin(visual), "color": color or [0.7, 0.7, 0.7, 1],
                            "scale": _floats(mesh.get("scale"), (1, 1, 1))})
        links[link.get("name")] = visuals
    joints = []
    for joint in root.findall("joint"):
        axis = joint.find("axis")
        joints.append({
            "name": joint.get("name"), "type": joint.get("type"),
            "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
            **_origin(joint), "axis": _floats(axis.get("xyz") if axis is not None else None, (1, 0, 0)),
        })
    children = {j["child"] for j in joints}
    roots = [name for name in links if name not in children]
    return {"name": root.get("name"), "root": roots[0] if roots else None, "links": links, "joints": joints}


def mesh_path(urdf_path: str, relative: str) -> str | None:
    """The file a mesh name refers to, or None if it would leave the URDF's directory."""
    base = os.path.dirname(os.path.abspath(urdf_path))
    full = os.path.normpath(os.path.join(base, relative))
    return full if full.startswith(base + os.sep) and os.path.isfile(full) else None
