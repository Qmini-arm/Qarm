from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np
import xacro
from qmini_arm_motion import ArmModel, CollisionChecker


def test_urdf_chain_and_zero_pose(model: ArmModel) -> None:
    assert model.base_link == "base_link"
    assert model.tip_link == "tool0"
    assert model.root_link == "world"
    assert model.joint_names == tuple(f"joint_{index}" for index in range(1, 7))
    expected_soft_limits_deg = [
        170.0,
        85.94366927,
        115.0,
        114.59155903,
        115.0,
        85.94366927,
    ]
    assert np.allclose(np.degrees(model.lower), -np.asarray(expected_soft_limits_deg), atol=1e-6)
    assert np.allclose(np.degrees(model.upper), expected_soft_limits_deg, atol=1e-6)
    assert np.allclose(
        model.fk(np.zeros(6))[:3, 3],
        [0.68206318, 0.04781823, -0.16252715],
        atol=1e-8,
    )
    base_collisions = model.links["base_link"].collisions
    assert tuple(geometry.kind for geometry in base_collisions) == ("cylinder", "box")


def test_visual_mesh_references_are_complete(model: ArmModel) -> None:
    root = ET.parse(model.urdf_path).getroot()
    filenames = {
        mesh.get("filename")
        for mesh in root.findall("./link/visual/geometry/mesh")
        if mesh.get("filename")
    }
    assert "meshes/visual/base_pair.stl" in filenames
    assert all((model.urdf_path.parent / filename).is_file() for filename in filenames)


def test_runtime_urdf_matches_xacro_source(model: ArmModel) -> None:
    xacro_path = model.urdf_path.with_suffix(".urdf.xacro")
    expanded = ET.fromstring(xacro.process_file(str(xacro_path)).toxml())
    runtime = ET.parse(model.urdf_path).getroot()

    def signature(node: ET.Element) -> tuple[object, ...]:
        text = " ".join((node.text or "").split())
        return (
            node.tag,
            tuple(sorted(node.attrib.items())),
            text,
            tuple(signature(child) for child in node),
        )

    assert signature(runtime) == signature(expanded)


def test_analytic_position_jacobian_matches_finite_difference(model: ArmModel) -> None:
    q = np.array([0.2, -0.3, 0.4, -0.25, 0.15, -0.1])
    numeric = np.zeros((3, model.dof))
    step = 1e-7
    for index in range(model.dof):
        delta = np.zeros(model.dof)
        delta[index] = step
        numeric[:, index] = (model.fk(q + delta)[:3, 3] - model.fk(q - delta)[:3, 3]) / (2.0 * step)
    assert np.allclose(model.jacobian(q)[:3], numeric, atol=1e-7)


def test_collision_checker_accepts_zero_and_rejects_known_overlap(
    collision: CollisionChecker,
) -> None:
    assert collision.is_free(np.zeros(6))
    colliding = np.array(
        [1.92408836, -0.14987336, 0.96931087, 0.06803653, 1.71572270, -0.84180829]
    )
    pairs = collision.check(colliding)
    assert pairs
    assert any({pair.link_a, pair.link_b} == {"link_3", "link_5"} for pair in pairs)


def test_edge_check_includes_the_full_segment(collision: CollisionChecker) -> None:
    colliding = np.array(
        [1.92408836, -0.14987336, 0.96931087, 0.06803653, 1.71572270, -0.84180829]
    )
    assert not collision.segment_is_free(np.zeros(6), colliding)
