"""Forward kinematics and the offset fit.

The URDF these arms actually use is CC BY-NC-SA and lives outside this
repository, so everything here is checked against a synthetic description
whose answers can be worked out by hand.
"""

import math
import struct

import numpy as np
import pytest

import kinematics
from kinematics import (
    Chain, MeshCache, Robot, Sample, axis_rotation, load_stl, matrix_to_rpy,
    resolve_mesh, rotation_vector, rpy_to_matrix, solve_ik, solve_offsets,
    unit_correction,
)

# A planar two-link arm: both joints turn about +Z, each link is 1 m of +X.
PLANAR = """<?xml version="1.0"?>
<robot name="planar">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="link2"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture
def planar(tmp_path):
    path = tmp_path / "planar.urdf"
    path.write_text(PLANAR, encoding="utf-8")
    return Robot.from_urdf(path)


# -- rotation helpers -----------------------------------------------------

def test_rpy_matches_the_urdf_convention():
    """URDF rpy is fixed-axis: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = 0.3, -0.7, 1.1
    rx = axis_rotation(np.array([1.0, 0, 0]), r)
    ry = axis_rotation(np.array([0, 1.0, 0]), p)
    rz = axis_rotation(np.array([0, 0, 1.0]), y)
    assert np.allclose(rpy_to_matrix(r, p, y), rz @ ry @ rx)


def test_rpy_round_trips():
    for rpy in [(0, 0, 0), (0.3, -0.7, 1.1), (-1.2, 0.4, -2.0)]:
        assert np.allclose(matrix_to_rpy(rpy_to_matrix(*rpy)), rpy, atol=1e-9)


def test_quarter_turn_about_z():
    rotated = axis_rotation(np.array([0, 0, 1.0]), math.pi / 2) @ [1.0, 0, 0]
    assert np.allclose(rotated, [0, 1, 0], atol=1e-12)


# -- structure ------------------------------------------------------------

def test_root_and_tips(planar):
    assert planar.root == "base"
    assert planar.tips() == ["tip"]


def test_chain_skips_fixed_joints_in_the_count(planar):
    chain = planar.chain("tip")
    assert chain.names == ["j1", "j2"]     # the fixed tip joint is not actuated
    assert len(chain) == 2
    assert len(chain.joints) == 3          # but it still contributes geometry


def test_chain_rejects_a_disconnected_tip(planar):
    with pytest.raises(ValueError):
        planar.chain("base", base="tip")


def test_wrong_joint_count_is_rejected(planar):
    with pytest.raises(ValueError, match="needs 2 joint values"):
        planar.chain("tip").position([0.0])


# -- forward kinematics ---------------------------------------------------

def test_straight_out(planar):
    """Both joints at zero: the arm is 2 m along +X."""
    assert np.allclose(planar.chain("tip").position([0, 0]), [2, 0, 0])


def test_folded_back(planar):
    """Elbow at 180 deg folds the forearm onto the upper arm."""
    assert np.allclose(planar.chain("tip").position([0, math.pi]),
                       [0, 0, 0], atol=1e-12)


def test_shoulder_quarter_turn(planar):
    assert np.allclose(planar.chain("tip").position([math.pi / 2, 0]),
                       [0, 2, 0], atol=1e-12)


def test_right_angle_elbow(planar):
    """Shoulder 0, elbow +90: out 1 m, then 1 m to the left."""
    assert np.allclose(planar.chain("tip").position([0, math.pi / 2]),
                       [1, 1, 0], atol=1e-12)


def test_joint_values_may_be_named(planar):
    chain = planar.chain("tip")
    assert np.allclose(chain.position({"j1": 0.0, "j2": math.pi / 2}),
                       chain.position([0, math.pi / 2]))


def test_frames_cover_every_link(planar):
    frames = planar.chain("tip").frames([0, 0])
    assert [name for name, _ in frames] == ["base", "link1", "link2", "tip"]
    # The drawn frames must agree with the reported tip.
    assert np.allclose(frames[-1][1][:3, 3], planar.chain("tip").position([0, 0]))


def test_prismatic_joint_translates(tmp_path):
    urdf = """<?xml version="1.0"?>
    <robot name="slider">
      <link name="base"/><link name="tip"/>
      <joint name="s" type="prismatic">
        <parent link="base"/><child link="tip"/>
        <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
        <limit lower="0" upper="0.5"/>
      </joint>
    </robot>"""
    path = tmp_path / "slider.urdf"
    path.write_text(urdf, encoding="utf-8")
    chain = Robot.from_urdf(path).chain("tip")
    assert np.allclose(chain.position([0.25]), [0, 0, 0.25])


def test_origin_rotation_is_applied(tmp_path):
    """A joint origin's rpy has to rotate the axis it carries."""
    urdf = """<?xml version="1.0"?>
    <robot name="tilted">
      <link name="base"/><link name="tip"/>
      <joint name="j" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="0 0 0" rpy="0 1.5707963267948966 0"/>
        <axis xyz="0 0 1"/><limit lower="-3" upper="3"/>
      </joint>
    </robot>"""
    path = tmp_path / "tilted.urdf"
    path.write_text(urdf, encoding="utf-8")
    chain = Robot.from_urdf(path).chain("tip")
    # The origin pitches +90 deg, so the joint's local +Z points along -X.
    axis = chain.fk([0.0])[:3, :3] @ [0, 0, 1]
    assert np.allclose(axis, [1, 0, 0], atol=1e-12) or \
        np.allclose(axis, [-1, 0, 0], atol=1e-12)


# -- the jacobian ---------------------------------------------------------

def test_jacobian_matches_a_hand_derivative(planar):
    """At full stretch, turning the shoulder sweeps the tip 2 m per radian."""
    jac = planar.chain("tip").jacobian([0, 0])
    assert jac.shape == (3, 2)
    assert np.allclose(jac[:, 0], [0, 2, 0], atol=1e-5)
    assert np.allclose(jac[:, 1], [0, 1, 0], atol=1e-5)


# -- the offset fit -------------------------------------------------------

def _synthetic(chain: Chain, poses, truth) -> list[Sample]:
    """Measurements a perfectly-built arm would produce with `truth` offsets."""
    return [Sample(q=np.array(q),
                   measured=chain.position(np.array(q) + truth))
            for q in poses]


def test_offsets_are_recovered_from_clean_measurements(planar):
    chain = planar.chain("tip")
    truth = np.array([0.05, -0.03])
    poses = [(0.0, 0.0), (0.6, -0.4), (-0.5, 0.9), (1.2, 0.3)]

    fit = solve_offsets(chain, _synthetic(chain, poses, truth))

    assert fit.converged
    assert np.allclose(fit.offsets, truth, atol=1e-6)
    assert fit.rms < 1e-9
    assert fit.rms_before > 1e-3          # the fit actually did something


def test_fit_reports_the_improvement(planar):
    chain = planar.chain("tip")
    truth = np.array([0.2, -0.15])
    poses = [(0.0, 0.0), (0.7, -0.5), (-0.6, 1.0)]

    fit = solve_offsets(chain, _synthetic(chain, poses, truth))

    assert fit.rms_before > fit.rms
    assert fit.residuals.size == 3 * 3    # three samples, xyz each


def test_fit_tolerates_measurement_noise(planar):
    """A 1 mm measurement error must not throw the offsets off."""
    chain = planar.chain("tip")
    truth = np.array([0.08, -0.06])
    poses = [(0.0, 0.0), (0.6, -0.4), (-0.5, 0.9), (1.2, 0.3), (-1.0, -0.7)]
    rng = np.random.default_rng(0)

    samples = _synthetic(chain, poses, truth)
    for sample in samples:
        sample.measured = sample.measured + rng.normal(0, 0.001, 3)

    fit = solve_offsets(chain, samples)
    assert np.allclose(fit.offsets, truth, atol=0.01)


def test_no_samples_is_an_error(planar):
    with pytest.raises(ValueError, match="no samples"):
        solve_offsets(planar.chain("tip"), [])


# -- rotation vectors -----------------------------------------------------

def test_rotation_vector_of_identity_is_zero():
    assert np.allclose(rotation_vector(np.eye(3)), np.zeros(3))


def test_rotation_vector_recovers_axis_and_angle():
    for axis, angle in [([0, 0, 1.0], 0.7), ([1.0, 0, 0], -1.2),
                        ([1, 1, 1.0], 2.0)]:
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        recovered = rotation_vector(axis_rotation(axis, angle))
        assert np.allclose(recovered, axis * angle, atol=1e-9)


def test_rotation_vector_survives_a_half_turn():
    """At pi the antisymmetric part vanishes and the naive formula divides by zero."""
    for axis in ([0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0], [1, 1, 0.0]):
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        recovered = rotation_vector(axis_rotation(axis, math.pi))
        assert np.isclose(np.linalg.norm(recovered), math.pi, atol=1e-4)
        # The sign of a half-turn axis is arbitrary; the line is not.
        assert np.allclose(np.abs(recovered / math.pi), np.abs(axis), atol=1e-4)


# -- inverse kinematics ---------------------------------------------------

SPATIAL = """<?xml version="1.0"?>
<robot name="spatial">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <link name="l4"/><link name="l5"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0"/></joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/></joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/></joint>
  <joint name="j4" type="revolute">
    <parent link="l3"/><child link="l4"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0"/></joint>
  <joint name="j5" type="revolute">
    <parent link="l4"/><child link="l5"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0"/></joint>
  <joint name="j6" type="revolute">
    <parent link="l5"/><child link="tip"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0"/></joint>
</robot>
"""


@pytest.fixture
def spatial(tmp_path):
    path = tmp_path / "spatial.urdf"
    path.write_text(SPATIAL, encoding="utf-8")
    return Robot.from_urdf(path).chain("tip")


def _reachable(chain, rng):
    """A pose generated from real joint values, so it is reachable by construction."""
    lower, upper = chain.limits()
    q = rng.uniform(np.maximum(lower, -2.0), np.minimum(upper, 2.0))
    return q, chain.fk(q)


def test_ik_reaches_a_reachable_full_pose(spatial):
    rng = np.random.default_rng(3)
    for _ in range(8):
        truth, target = _reachable(spatial, rng)
        result = solve_ik(spatial, target, seed=truth * 0.5)
        assert result.converged
        assert result.position_error < 1e-3
        assert result.orientation_error < 1e-2


def test_ik_position_only_ignores_orientation(spatial):
    rng = np.random.default_rng(4)
    _, target = _reachable(spatial, rng)
    result = solve_ik(spatial, target, orientation=False)
    assert result.converged
    assert result.position_error < 1e-3


def test_ik_respects_joint_limits(spatial):
    rng = np.random.default_rng(5)
    lower, upper = spatial.limits()
    for _ in range(6):
        _, target = _reachable(spatial, rng)
        result = solve_ik(spatial, target)
        assert np.all(result.q >= lower - 1e-9)
        assert np.all(result.q <= upper + 1e-9)


def test_ik_fails_gracefully_on_an_unreachable_target(spatial):
    """Damping is what stops this throwing the arm somewhere unrelated."""
    target = kinematics.pose([5.0, 5.0, 5.0], [0, 0, 0])
    result = solve_ik(spatial, target)

    assert not result.converged
    assert result.position_error > 1.0            # honestly reported
    assert np.all(np.isfinite(result.q))          # not a blow-up
    lower, upper = spatial.limits()
    assert np.all(result.q >= lower - 1e-9) and np.all(result.q <= upper + 1e-9)


def test_ik_is_deterministic(spatial):
    """Anything that drives hardware must give the same answer every time."""
    target = kinematics.pose([0.3, 0.1, 0.6], [0.2, 0.3, -0.1])
    first = solve_ik(spatial, target)
    second = solve_ik(spatial, target)
    assert np.allclose(first.q, second.q)


def test_ik_prefers_to_stay_near_the_seed(spatial):
    """Redundancy resolved by least motion - the safe choice near hardware."""
    rng = np.random.default_rng(6)
    truth, target = _reachable(spatial, rng)
    result = solve_ik(spatial, target, seed=truth)
    assert result.converged
    # Seeded at the answer, it should barely move at all.
    assert np.max(np.abs(result.q - truth)) < 0.05


def test_ik_reports_joints_resting_on_a_limit(tmp_path):
    """A joint pinned by its URDF limit has to be named, not silently clamped.

    Reaching for a far target is not enough to prove this - a chain simply
    stretches out straight, which is well inside its limits. It takes a
    target that can only be served by exceeding one.
    """
    # The link length has to come AFTER the revolute joint. Putting the
    # translation in the joint's own origin rotates the tip in place and
    # moves it nowhere, which makes the position Jacobian identically zero.
    urdf = """<?xml version="1.0"?>
    <robot name="narrow">
      <link name="base"/><link name="arm"/><link name="tip"/>
      <joint name="j" type="revolute">
        <parent link="base"/><child link="arm"/>
        <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
        <limit lower="0.0" upper="0.4"/>
      </joint>
      <joint name="reach" type="fixed">
        <parent link="arm"/><child link="tip"/>
        <origin xyz="1 0 0"/>
      </joint>
    </robot>"""
    path = tmp_path / "narrow.urdf"
    path.write_text(urdf, encoding="utf-8")
    chain = Robot.from_urdf(path).chain("tip")

    # Straight up the +Y axis would need 90 deg; the joint stops at 0.4 rad.
    result = solve_ik(chain, kinematics.pose([0.0, 1.0, 0.0], [0, 0, 0]),
                      orientation=False)

    assert result.at_limit == ["j"]
    assert result.q[0] == pytest.approx(0.4)
    assert not result.converged


def test_pose_helper_round_trips():
    transform = kinematics.pose([1, 2, 3], [0.1, -0.2, 0.3])
    assert np.allclose(transform[:3, 3], [1, 2, 3])
    assert np.allclose(matrix_to_rpy(transform), [0.1, -0.2, 0.3])


# -- meshes ---------------------------------------------------------------

def _write_stl(path, triangles) -> None:
    """A minimal binary STL, the format the description packages ship."""
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            fh.write(struct.pack("<3f", 0.0, 0.0, 1.0))     # normal
            for vertex in triangle:
                fh.write(struct.pack("<3f", *vertex))
            fh.write(b"\0\0")                                # attribute word


TRIANGLES = [[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
             [(0, 0, 1), (1, 0, 1), (0, 1, 1)]]


def test_binary_stl_round_trips(tmp_path):
    path = tmp_path / "block.stl"
    _write_stl(path, TRIANGLES)
    loaded = load_stl(path)
    assert loaded.shape == (2, 3, 3)
    assert np.allclose(loaded[0], TRIANGLES[0])


def test_ascii_stl_is_read(tmp_path):
    path = tmp_path / "ascii.stl"
    path.write_text(
        "solid s\n"
        "facet normal 0 0 1\n outer loop\n"
        "  vertex 0 0 0\n  vertex 1 0 0\n  vertex 0 1 0\n"
        " endloop\nendfacet\nendsolid s\n", encoding="utf-8")
    loaded = load_stl(path)
    assert loaded.shape == (1, 3, 3)
    assert np.allclose(loaded[0][1], [1, 0, 0])


def test_package_uri_resolves_against_the_package_folder(tmp_path):
    """`package://pkg/meshes/x.stl` under a root that IS the package."""
    package = tmp_path / "my_description"
    (package / "meshes").mkdir(parents=True)
    _write_stl(package / "meshes" / "x.stl", TRIANGLES)

    found = resolve_mesh("package://my_description/meshes/x.stl", [package])
    assert found == package / "meshes" / "x.stl"


def test_package_uri_resolves_against_the_parent_folder(tmp_path):
    """The same URI under a root that CONTAINS the package."""
    package = tmp_path / "my_description"
    (package / "meshes").mkdir(parents=True)
    _write_stl(package / "meshes" / "x.stl", TRIANGLES)

    found = resolve_mesh("package://my_description/meshes/x.stl", [tmp_path])
    assert found == package / "meshes" / "x.stl"


def test_mesh_is_found_by_name_in_a_flat_folder(tmp_path):
    """A folder of loose STLs is a normal thing to be handed."""
    _write_stl(tmp_path / "x.stl", TRIANGLES)
    found = resolve_mesh("package://whatever/deep/path/x.stl", [tmp_path])
    assert found == tmp_path / "x.stl"


def test_unresolvable_mesh_returns_none(tmp_path):
    assert resolve_mesh("package://p/missing.stl", [tmp_path]) is None


def test_cache_loads_once_and_records_misses(tmp_path):
    _write_stl(tmp_path / "x.stl", TRIANGLES)
    cache = MeshCache()

    first = cache.triangles("package://p/x.stl", [tmp_path])
    second = cache.triangles("package://p/x.stl", [tmp_path])
    assert first is second                      # same object, loaded once
    assert not cache.missing

    assert cache.triangles("package://p/nope.stl", [tmp_path]) is None
    assert "package://p/nope.stl" in cache.missing


def test_link_geometry_is_parsed(tmp_path):
    urdf = """<?xml version="1.0"?>
    <robot name="withmesh">
      <link name="base">
        <collision>
          <origin xyz="0 0 0.5" rpy="0 0 0"/>
          <geometry><mesh filename="package://p/meshes/base.stl"/></geometry>
        </collision>
        <visual>
          <geometry><mesh filename="package://p/meshes/base.dae"
                          scale="0.001 0.001 0.001"/></geometry>
        </visual>
      </link>
    </robot>"""
    path = tmp_path / "withmesh.urdf"
    path.write_text(urdf, encoding="utf-8")
    link = Robot.from_urdf(path).links["base"]

    assert len(link.collisions) == 1 and len(link.visuals) == 1
    assert link.collisions[0].mesh.endswith("base.stl")
    assert link.collisions[0].origin[2, 3] == pytest.approx(0.5)
    assert np.allclose(link.visuals[0].scale, [0.001, 0.001, 0.001])
    # Collision is preferred: only STL is readable here, visuals are DAE.
    assert link.geometry("collision") == link.collisions


# -- unit sanity ----------------------------------------------------------

def _box(size: float) -> np.ndarray:
    """One triangle spanning a cube of the given edge length."""
    return np.array([[(0, 0, 0), (size, 0, 0), (0, size, size)]], dtype=float)


def test_millimetre_mesh_declared_as_metres_is_caught():
    """OpenArmX's own body mesh: 773 units tall with scale 1.0."""
    assert unit_correction(_box(773.0), [1, 1, 1], reference=1.3) == 0.001


def test_correctly_scaled_meshes_are_left_alone():
    # Arm links: metres, declared 1.0.
    assert unit_correction(_box(0.22), [1, 1, 1], reference=1.3) is None
    # Hand: millimetres, correctly declared 0.001.
    assert unit_correction(_box(168.0), [0.001, 0.001, 0.001],
                           reference=1.3) is None


def test_a_negative_scale_still_measures_by_magnitude():
    """Mirrored links carry scale -1; that is not a unit error."""
    assert unit_correction(_box(0.22), [1, -1, 1], reference=1.3) is None


def test_absurd_after_conversion_is_not_corrected():
    """If millimetres do not explain it, leave it and let the user see it."""
    assert unit_correction(_box(1e9), [1, 1, 1], reference=1.3) is None


def test_no_reference_means_no_guessing():
    assert unit_correction(_box(773.0), [1, 1, 1], reference=0.0) is None


def test_empty_mesh_is_ignored():
    assert unit_correction(np.zeros((0, 3, 3)), [1, 1, 1], 1.3) is None


def test_geometry_falls_back_when_one_kind_is_absent(tmp_path):
    urdf = """<?xml version="1.0"?>
    <robot name="visualonly">
      <link name="base">
        <visual><geometry><mesh filename="v.stl"/></geometry></visual>
      </link>
    </robot>"""
    path = tmp_path / "visualonly.urdf"
    path.write_text(urdf, encoding="utf-8")
    link = Robot.from_urdf(path).links["base"]
    assert link.geometry("collision") == link.visuals
