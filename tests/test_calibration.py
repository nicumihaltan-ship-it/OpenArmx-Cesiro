"""Fixed-point calibration: the fit, what it can identify, and pose choice.

Checked against a synthetic arm, for the same reason as test_kinematics: the
description these arms actually use is CC BY-NC-SA and lives outside this
repository. The arm below is deliberately a proper 3D one - a planar chain
would hide exactly the rank problems this module exists to report.
"""

import math

import numpy as np
import pytest

import calibration as cal
import kinematics as kin

# Six revolute joints, mixed axes, with the tip deliberately off the last
# joint's axis - put it on the axis and that joint's offset stops being
# observable, which is a real effect worth not tripping over by accident.
ARM = """<?xml version="1.0"?>
<robot name="arm6">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <link name="l4"/><link name="l5"/><link name="l6"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.10"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.08"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0 0 0.30"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="j4" type="revolute">
    <parent link="l3"/><child link="l4"/>
    <origin xyz="0 0 0.06"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/>
  </joint>
  <joint name="j5" type="revolute">
    <parent link="l4"/><child link="l5"/>
    <origin xyz="0 0 0.26"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="j6" type="revolute">
    <parent link="l5"/><child link="l6"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="l6"/><child link="tip"/>
    <origin xyz="0.04 0 0.09"/>
  </joint>
</robot>
"""


@pytest.fixture
def chain(tmp_path):
    path = tmp_path / "arm6.urdf"
    path.write_text(ARM, encoding="utf-8")
    return kin.Robot.from_urdf(path).chain("tip")


SEED = np.array([0.30, -0.60, 0.45, 0.90, -0.35, 0.55])


def reachable(chain, seed, count, *, tool=(0.0, 0.0, 0.0), rng_seed=1):
    """Joint vectors that all place the tool tip exactly where seed does."""
    poses = cal.candidates(chain, seed, tool=tool, count=count,
                           attempts=count * 40, rng_seed=rng_seed)
    assert len(poses) >= count, f"only generated {len(poses)} of {count}"
    return poses[:count]


# -- the tool on a chain ---------------------------------------------------


def test_tool_offsets_the_whole_chain(chain):
    tooled = chain.with_tool(kin.pose((0.0, 0.0, 0.12), (0, 0, 0)))
    q = np.zeros(len(chain))
    # Everything is stacked along +Z at zero, so the tool lands 120 mm above.
    assert np.allclose(tooled.position(q) - chain.position(q),
                       [0.0, 0.0, 0.12])
    # And the derived quantities follow it rather than the flange.
    assert not np.allclose(tooled.jacobian(q), chain.jacobian(q))
    assert chain.tool.shape == (4, 4)        # the plain chain is unchanged


def test_solve_ik_aims_the_tool_not_the_flange(chain):
    tooled = chain.with_tool(kin.pose((0.03, 0.0, 0.11), (0, 0, 0)))
    target = np.eye(4)
    target[:3, 3] = tooled.position(SEED)
    result = kin.solve_ik(tooled, target, seed=SEED * 0.5, orientation=False)
    assert result.position_error < 1e-4


# -- scatter ---------------------------------------------------------------


def test_spread_reports_centre_rms_and_span():
    points = np.array([[0.0, 0, 0], [0.01, 0, 0], [0.02, 0, 0]])
    result = cal.spread(points)
    assert np.allclose(result.centre, [0.01, 0, 0])
    assert result.worst == pytest.approx(0.01)
    assert result.span == pytest.approx(0.02)     # the two extremes
    assert result.rms == pytest.approx(math.sqrt((1e-4 + 0 + 1e-4) / 3))


def test_spread_about_a_given_centre():
    result = cal.spread([[0.0, 0, 0]], centre=[0.003, 0, 0])
    assert result.rms == pytest.approx(0.003)


# -- the fit ---------------------------------------------------------------


def test_recovers_planted_offsets(chain):
    """The whole point: plant offsets, capture, get them back."""
    truth = np.array([0.0, 0.020, -0.035, 0.012, 0.028, -0.017])
    truth_poses = reachable(chain, SEED, 12)
    # The motors report a joint value short by the offset, which is what the
    # operator captures; the fit has to add it back.
    poses = [cal.Pose(q - truth) for q in truth_poses]

    fit = cal.solve_fixed_point(chain, poses, locked=["j1"])

    assert fit.after.rms < 1e-6
    assert fit.before.rms > 1e-3          # the planted error was visible
    assert np.allclose(fit.offsets, truth, atol=2e-4)


def test_fit_reports_the_scatter_it_removed(chain):
    truth = np.array([0.0, 0.03, -0.02, 0.0, 0.015, 0.0])
    poses = [cal.Pose(q - truth) for q in reachable(chain, SEED, 10)]
    fit = cal.solve_fixed_point(chain, poses, locked=["j1"])
    assert fit.improvement > 100
    assert fit.before.span > fit.after.span
    assert fit.named()["j3"] == pytest.approx(-0.02, abs=2e-4)


def test_a_perfect_arm_is_left_alone(chain):
    poses = [cal.Pose(q) for q in reachable(chain, SEED, 8)]
    fit = cal.solve_fixed_point(chain, poses, locked=["j1"])
    assert fit.before.rms < 1e-9
    assert np.max(np.abs(fit.offsets)) < 1e-4


def test_known_fixture_point_makes_the_first_joint_observable(chain):
    truth = np.array([0.025, 0.020, -0.035, 0.012, 0.028, -0.017])
    truth_poses = reachable(chain, SEED, 14)
    point = chain.position(truth_poses[0])
    poses = [cal.Pose(q - truth) for q in truth_poses]

    # Measuring the fixture into the base frame is the only thing that pins
    # the base rotation down, so j1 comes back only in this variant.
    fit = cal.solve_fixed_point(chain, poses, point=point)
    assert np.allclose(fit.offsets, truth, atol=5e-4)

    free = cal.solve_fixed_point(chain, poses)
    assert free.after.rms < 1e-5          # fits the data just as well...
    assert abs(free.offsets[0] - truth[0]) > 1e-3    # ...on a different j1


def test_fitting_the_tool_recovers_where_the_tip_is(chain):
    tool = np.array([0.05, -0.02, 0.11])
    truth = np.array([0.0, 0.02, -0.03, 0.0, 0.0, 0.0])
    poses = [cal.Pose(q - truth)
             for q in reachable(chain, SEED, 14, tool=tool)]

    fit = cal.solve_fixed_point(chain, poses, tool=(0.0, 0.0, 0.10),
                                fit_tool=True, locked=["j1", "j6"])
    assert fit.after.rms < 1e-5
    assert np.allclose(fit.tool, tool, atol=1e-3)


def test_empty_capture_is_an_error(chain):
    with pytest.raises(ValueError):
        cal.solve_fixed_point(chain, [])


# -- observability ---------------------------------------------------------


def test_first_joint_is_never_identified_by_a_free_point(chain):
    poses = [cal.Pose(q) for q in reachable(chain, SEED, 12)]
    report = cal.observability(chain, poses)
    assert "j1" in report.weak()
    assert not np.isfinite(report.by_name()["j1"])
    # The joints in between are perfectly ordinary.
    assert report.by_name()["j3"] < cal.WEAK


def test_measuring_the_point_identifies_the_first_joint(chain):
    poses = [cal.Pose(q) for q in reachable(chain, SEED, 12)]
    report = cal.observability(chain, poses, free_point=False)
    assert "j1" not in report.weak()
    assert math.isfinite(report.condition)


def test_a_single_pose_identifies_almost_nothing(chain):
    one = cal.observability(chain, [cal.Pose(SEED)], locked=["j1"])
    many = cal.observability(chain, [cal.Pose(q) for q in reachable(
        chain, SEED, 12)], locked=["j1"])
    assert len(one.weak()) > len(many.weak())
    assert many.worst_joint() < one.worst_joint()


def test_no_poses_at_all_is_total_ignorance(chain):
    report = cal.observability(chain, [])
    assert not np.isfinite(report.uncertainty).any()
    assert report.weak() == report.names


def test_locked_joints_are_absent_from_the_report(chain):
    report = cal.observability(chain, [cal.Pose(SEED)], locked=["j1", "j2"])
    assert "j1" not in report.names and "j2" not in report.names
    assert "j3" in report.names


# -- pose generation -------------------------------------------------------


def test_variants_all_reach_the_same_point(chain):
    poses = cal.variants(chain, SEED, count=6, locked=["j1"])
    assert len(poses) == 6
    target = chain.position(SEED)
    for q in poses:
        assert np.linalg.norm(chain.position(q) - target) < 1e-3


def test_variants_are_distinct_from_each_other(chain):
    poses = [SEED, *cal.variants(chain, SEED, count=6, locked=["j1"])]
    for i, a in enumerate(poses):
        for b in poses[i + 1:]:
            assert np.max(np.abs(a - b)) >= cal.SEPARATION * 0.9


def test_variants_respect_joint_limits(chain):
    lower, upper = chain.limits()
    for q in cal.variants(chain, SEED, count=6, locked=["j1"]):
        assert np.all(q >= lower - 1e-6) and np.all(q <= upper + 1e-6)


def test_variants_honour_a_tool_offset(chain):
    tool = (0.04, 0.0, 0.13)
    arm = chain.with_tool(kin.pose(tool, (0, 0, 0)))
    target = arm.position(SEED)
    for q in cal.variants(chain, SEED, count=4, tool=tool, locked=["j1"]):
        assert np.linalg.norm(arm.position(q) - target) < 1e-3


def test_chosen_poses_beat_the_ones_they_were_chosen_from(chain):
    """The selection has to earn its keep against just taking the first N."""
    pool = cal.candidates(chain, SEED, count=40, attempts=600, rng_seed=3)
    assert len(pool) >= 12
    picked = cal.choose(chain, SEED, pool, 6, locked=["j1"])
    arbitrary = pool[:6]

    def worst(poses):
        return cal.observability(
            chain, [cal.Pose(SEED)] + [cal.Pose(q) for q in poses],
            locked=["j1"]).worst_joint()

    assert worst(picked) < worst(arbitrary)


def test_generation_is_deterministic(chain):
    first = cal.variants(chain, SEED, count=5, locked=["j1"])
    second = cal.variants(chain, SEED, count=5, locked=["j1"])
    assert all(np.allclose(a, b) for a, b in zip(first, second))


def test_choose_handles_an_empty_pool(chain):
    assert cal.choose(chain, SEED, [], 5) == []
    assert cal.variants(chain, SEED, count=0) == []
