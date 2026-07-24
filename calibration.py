"""Joint-zero identification from a tool tip pinned to one point in space.

The arm holds a tool whose tip is captured by a fixture, so the tip cannot
move while the joints do. Nothing about *where* that fixture is has to be
known: if the model were perfect, every configuration the arm can reach with
the tip still seated would put the computed tip in exactly the same place.
It does not, and that scatter is the error the joint offsets have to explain.

Worth stating plainly, because it is what makes the procedure practical on a
bench: no external metrology, no measuring the fixture into the base frame,
no touching off against a known coordinate. A rigid tool and a machined seat
are the whole apparatus.

What a fixed point cannot do is fix the arm in space, and pretending
otherwise is how a calibration produces confident nonsense:

- Add a constant to the **first joint's** offset and every computed tip
  position rotates about the base axis - but so does the fitted fixture
  point, leaving every residual exactly as it was. The first joint's offset
  is unobservable from a fixed point alone. It always is, no matter how many
  poses are captured.
- The **last joint's** offset goes the same way whenever the tool tip sits on
  its axis, and whenever the tool offset is being fitted at the same time:
  turning the tool and rotating the tool vector are the same motion.

The fit damps those directions instead of inventing a number for them, and
:func:`observability` names them and says how badly each offset is pinned
down. An offset nothing constrains should read as "not identified", not as
0.004 degrees.

Everything here is radians and metres. Nothing in this module talks to
hardware or to Qt.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

import kinematics as kin

log = logging.getLogger(__name__)

#: Step used for the numerical Jacobian, matching :mod:`kinematics`.
EPS = 1e-6

#: The measurement noise identifiability is quoted against. One millimetre is
#: about what a hand-guided capture into a machined seat is worth, so the
#: numbers read directly as "degrees of offset error per millimetre of slop".
NOISE = 1e-3

#: An offset the captured poses pin down no better than this many degrees per
#: millimetre of noise is reported as not identified.
WEAK = 2.0

#: Joint vectors closer than this in max-norm (radians) are the same pose as
#: far as new information is concerned.
SEPARATION = 0.30


@dataclass
class Pose:
    """One capture: the joint vector the model believes, plus a label."""

    q: np.ndarray
    label: str = ""

    def __post_init__(self) -> None:
        self.q = np.asarray(self.q, dtype=float).ravel()


# --------------------------------------------------------------------------
# Forward evaluation
# --------------------------------------------------------------------------


def tip_frames(chain: kin.Chain, poses, offsets) -> np.ndarray:
    """Tip-*link* frame in every pose, as an ``(N, 4, 4)`` array.

    Deliberately stops at the last link rather than going through
    :meth:`kinematics.Chain.fk`: here the tool is a parameter being fitted,
    not a fixed property of the chain, so a tool already attached to the
    chain would be counted twice.
    """
    offsets = np.asarray(offsets, dtype=float)
    out = np.empty((len(poses), 4, 4))
    for i, pose in enumerate(poses):
        out[i] = chain.frames(pose.q + offsets)[-1][1]
    return out


def tool_points(chain: kin.Chain, poses, offsets, tool) -> np.ndarray:
    """Where the model puts the tool tip in each pose, ``(N, 3)``."""
    frames = tip_frames(chain, poses, offsets)
    tool = np.asarray(tool, dtype=float)
    return frames[:, :3, :3] @ tool + frames[:, :3, 3]


@dataclass
class Spread:
    """How badly a set of supposedly-identical points disagrees."""

    centre: np.ndarray
    distances: np.ndarray               # per point, metres from the centre

    @property
    def rms(self) -> float:
        return float(np.sqrt(np.mean(self.distances ** 2))) \
            if self.distances.size else 0.0

    @property
    def worst(self) -> float:
        return float(self.distances.max()) if self.distances.size else 0.0

    @property
    def span(self) -> float:
        """Distance between the two points that disagree most.

        The headline number for an operator: "these two poses think the tip
        is 14 mm apart" needs no explaining, where an RMS about a centroid
        nobody chose does.
        """
        if self.distances.size < 2:
            return 0.0
        points = self._points
        diff = points[:, None, :] - points[None, :, :]
        return float(np.linalg.norm(diff, axis=2).max())

    _points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)),
                                repr=False)


def spread(points, centre=None) -> Spread:
    """Scatter of ``points`` about ``centre``, or about their centroid."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if centre is None:
        centre = points.mean(axis=0) if len(points) else np.zeros(3)
    centre = np.asarray(centre, dtype=float)
    return Spread(centre=centre,
                  distances=np.linalg.norm(points - centre, axis=1),
                  _points=points)


# --------------------------------------------------------------------------
# Parameter layout
#
# One vector holds every unknown: the joint offsets, the tool tip in the tip
# frame, and the fixture point in the base frame. A boolean mask says which
# of them this particular fit is allowed to move, so locking a joint, using a
# measured fixture point and fitting the tool are all the same mechanism.
# --------------------------------------------------------------------------

TOOL_NAMES = ("tool X", "tool Y", "tool Z")
POINT_NAMES = ("fixture X", "fixture Y", "fixture Z")


@dataclass
class Layout:
    names: list[str]
    free: np.ndarray                    # bool, one per parameter
    joints: int                         # how many leading entries are joints

    @property
    def size(self) -> int:
        return len(self.names)

    @property
    def free_names(self) -> list[str]:
        return [n for n, f in zip(self.names, self.free) if f]

    def tool_slice(self) -> slice:
        return slice(self.joints, self.joints + 3)

    def point_slice(self) -> slice:
        return slice(self.joints + 3, self.joints + 6)


def layout(chain: kin.Chain, *, fit_tool: bool = False,
           free_point: bool = True, locked=()) -> Layout:
    """Which unknowns a fit with these choices is allowed to move."""
    joint_names = [joint.name for joint in chain.actuated]
    locked = set(locked)
    free = np.array(
        [name not in locked for name in joint_names]
        + [bool(fit_tool)] * 3
        + [bool(free_point)] * 3)
    return Layout(names=joint_names + list(TOOL_NAMES) + list(POINT_NAMES),
                  free=free, joints=len(joint_names))


def _jacobian(chain: kin.Chain, poses, offsets, tool,
              plan: Layout) -> np.ndarray:
    """``(3N, P)`` derivative of every tip position by every parameter.

    Numerical in the joint columns for the same reason
    :meth:`kinematics.Chain.jacobian` is - it cannot silently disagree with
    the forward kinematics it is differentiating. The tool and fixture
    columns are exact and free: the tip position depends on them through one
    rotation and one subtraction.
    """
    n = plan.joints
    offsets = np.asarray(offsets, dtype=float)
    tool = np.asarray(tool, dtype=float)

    frames = tip_frames(chain, poses, offsets)
    base = (frames[:, :3, :3] @ tool + frames[:, :3, 3]).ravel()

    out = np.zeros((3 * len(poses), plan.size))
    for k in range(n):
        if not plan.free[k]:
            continue                    # a locked joint never moves
        shifted = offsets.copy()
        shifted[k] += EPS
        out[:, k] = (tool_points(chain, poses, shifted, tool).ravel()
                     - base) / EPS
    for i in range(len(poses)):
        rows = slice(3 * i, 3 * i + 3)
        out[rows, plan.tool_slice()] = frames[i, :3, :3]
        out[rows, plan.point_slice()] = -np.eye(3)
    return out


# --------------------------------------------------------------------------
# The fit
# --------------------------------------------------------------------------


@dataclass
class Fit:
    offsets: np.ndarray                 # rad, to be added to the joint values
    tool: np.ndarray                    # tool tip in the tip frame, m
    point: np.ndarray                   # fixture point in the base frame, m
    before: Spread                      # tip scatter with the offsets at zero
    after: Spread                       # tip scatter with them applied
    plan: Layout
    iterations: int
    converged: bool

    @property
    def improvement(self) -> float:
        """Factor by which the RMS scatter shrank. 1.0 means no help."""
        return self.before.rms / self.after.rms if self.after.rms > 0 else \
            float("inf")

    def named(self) -> dict[str, float]:
        """Joint name -> fitted offset, radians."""
        return {name: float(value) for name, value
                in zip(self.plan.names[:self.plan.joints], self.offsets)}


def solve_fixed_point(chain: kin.Chain, poses, *, tool=(0.0, 0.0, 0.0),
                      fit_tool: bool = False, point=None, locked=(),
                      iterations: int = 120, tolerance: float = 1e-11) -> Fit:
    """Joint offsets that make every pose agree on where the tool tip is.

    ``point`` is the fixture position in the base frame if it was measured;
    leave it ``None`` - the usual case - and it becomes another unknown,
    which costs three parameters and buys freedom from ever having to measure
    the fixture.

    Levenberg-Marquardt rather than the plain Gauss-Newton in
    :func:`kinematics.solve_offsets`, because this problem is *structurally*
    rank-deficient (see the module docstring) rather than merely
    under-determined when samples are few. Adaptive damping keeps the
    unobservable directions parked near zero instead of letting a fixed
    damping factor trade accuracy against stability for the whole run.
    """
    poses = list(poses)
    if not poses:
        raise ValueError("no poses captured")
    plan = layout(chain, fit_tool=fit_tool, free_point=point is None,
                  locked=locked)
    n = plan.joints

    x = np.zeros(plan.size)
    x[plan.tool_slice()] = np.asarray(tool, dtype=float)
    start = tool_points(chain, poses, x[:n], x[plan.tool_slice()])
    x[plan.point_slice()] = (np.asarray(point, dtype=float) if point is not None
                             else start.mean(axis=0))
    before = spread(start, x[plan.point_slice()])

    free = plan.free

    def residual(vector: np.ndarray) -> np.ndarray:
        pts = tool_points(chain, poses, vector[:n], vector[plan.tool_slice()])
        return (pts - vector[plan.point_slice()]).ravel()

    error = residual(x)
    cost = float(error @ error)
    damping = 1e-6
    used = 0
    converged = False

    for used in range(1, iterations + 1):
        jac = _jacobian(chain, poses, x[:n], x[plan.tool_slice()], plan)[:, free]
        gradient = jac.T @ error
        normal = jac.T @ jac
        # Marquardt's scaling: damp each parameter in proportion to its own
        # curvature. A flat damping term is measured in whatever units the
        # column happens to have, and here the columns are radians and metres
        # at once, so a flat term silently favours one over the other.
        scale = np.diag(np.maximum(np.diag(normal), 1e-12))
        for _ in range(12):
            try:
                step = np.linalg.solve(normal + damping * scale, -gradient)
            except np.linalg.LinAlgError:
                damping *= 10.0
                continue
            candidate = x.copy()
            candidate[free] += step
            trial = residual(candidate)
            trial_cost = float(trial @ trial)
            if trial_cost < cost:
                x, error, cost = candidate, trial, trial_cost
                damping = max(damping / 3.0, 1e-12)
                break
            damping *= 5.0
        else:
            # No damping in that range improved anything: this is as far as
            # the data goes.
            converged = True
            break
        if np.max(np.abs(step)) < tolerance:
            converged = True
            break

    after = spread(
        tool_points(chain, poses, x[:n], x[plan.tool_slice()]),
        x[plan.point_slice()])
    return Fit(offsets=x[:n], tool=x[plan.tool_slice()].copy(),
               point=x[plan.point_slice()].copy(), before=before, after=after,
               plan=plan, iterations=used, converged=converged)


# --------------------------------------------------------------------------
# Observability
#
# The question this answers is not "did the fit converge" - it always does -
# but "which of these numbers did the data actually determine". Without it a
# fixed-point calibration reports seven confident offsets when the poses
# only ever constrained five.
# --------------------------------------------------------------------------


@dataclass
class Observability:
    names: list[str]                    # the free parameters, in order
    singular: np.ndarray                # singular values of the stacked J
    uncertainty: np.ndarray             # per parameter, deg per mm of noise
    joints: int                         # leading entries that are joints

    @property
    def condition(self) -> float:
        if self.singular.size == 0 or self.singular[-1] <= 0:
            return float("inf")
        return float(self.singular[0] / self.singular[-1])

    def weak(self, limit: float = WEAK) -> list[str]:
        """Parameters the captured poses do not pin down."""
        return [name for name, value in zip(self.names, self.uncertainty)
                if not np.isfinite(value) or value > limit]

    def worst_joint(self) -> float:
        """Uncertainty of the least-determined joint offset, deg per mm."""
        values = self.uncertainty[:self.joints]
        return float(values.max()) if values.size else 0.0

    def by_name(self) -> dict[str, float]:
        return dict(zip(self.names, (float(v) for v in self.uncertainty)))


def observability(chain: kin.Chain, poses, *, offsets=None,
                  tool=(0.0, 0.0, 0.0), fit_tool: bool = False,
                  free_point: bool = True, locked=(),
                  noise: float = NOISE) -> Observability:
    """How well the captured poses determine each unknown.

    The figure per parameter is its standard deviation given ``noise`` of
    measurement error, which reads directly: 0.05 deg/mm means a millimetre
    of slop in the fixture moves that offset by five hundredths of a degree,
    and 40 deg/mm means the poses say essentially nothing about it.

    Derived from the pseudo-inverse of ``J^T J``, so the structurally
    unobservable directions come back as astronomically large rather than as
    an exception.
    """
    poses = list(poses)
    plan = layout(chain, fit_tool=fit_tool, free_point=free_point,
                  locked=locked)
    if offsets is None:
        offsets = np.zeros(plan.joints)
    if not poses:
        return Observability(names=plan.free_names,
                             singular=np.zeros(0),
                             uncertainty=np.full(int(plan.free.sum()),
                                                 np.inf),
                             joints=int(plan.free[:plan.joints].sum()))

    jac = _jacobian(chain, poses, offsets, tool, plan)[:, plan.free]
    singular = np.linalg.svd(jac, compute_uv=False)
    # rcond relative to the largest singular value: anything below it is a
    # null direction, and pinv reports it as unconstrained rather than
    # inverting numerical dust into a huge but finite number.
    covariance = np.linalg.pinv(jac.T @ jac, rcond=1e-12, hermitian=True)
    variance = np.clip(np.diag(covariance), 0.0, None)
    with np.errstate(divide="ignore", over="ignore"):
        sigma = np.degrees(noise * np.sqrt(variance))
    # A direction pinv zeroed out is unconstrained, not perfectly known.
    rank = int((singular > singular[0] * 1e-9).sum()) if singular.size else 0
    if rank < jac.shape[1]:
        # Parameters living mostly in the null space get called out as
        # infinite rather than as whatever the pseudo-inverse rounded to.
        _, _, vt = np.linalg.svd(jac, full_matrices=True)
        null = vt[rank:]
        if null.size:
            share = (null ** 2).sum(axis=0)
            sigma = np.where(share > 0.5, np.inf, sigma)
    return Observability(names=plan.free_names, singular=singular,
                         uncertainty=sigma,
                         joints=int(plan.free[:plan.joints].sum()))


# --------------------------------------------------------------------------
# Pose generation
#
# A seven-joint arm holding its tip at one point has a four-dimensional space
# of motions that keep it there. Those are exactly the poses this procedure
# wants: same tip, different arm. Random ones work; chosen ones work far
# better, so the candidates are generated by a null-space walk and then
# picked for how much they actually add.
# --------------------------------------------------------------------------


def _nullspace_step(chain: kin.Chain, q, rng, size: float) -> np.ndarray:
    """A joint-space move that, to first order, does not move the tip."""
    jac = chain.jacobian(q)
    projector = np.eye(len(q)) - np.linalg.pinv(jac) @ jac
    direction = projector @ rng.normal(size=len(q))
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return np.zeros(len(q))
    return direction / norm * size


def candidates(chain: kin.Chain, seed, *, tool=(0.0, 0.0, 0.0), count: int = 60,
               attempts: int = 400, separation: float = SEPARATION,
               tolerance: float = 5e-4, rng_seed: int = 0) -> list[np.ndarray]:
    """Distinct joint vectors that all put the tool tip where ``seed`` does.

    Null-space walk, then a position-only IK snap to undo the drift the walk
    accumulates - the null space is only tangent, so a finite step off it
    always moves the tip a little.
    """
    seed = np.asarray(seed, dtype=float).ravel()
    arm = chain.with_tool(kin.pose(tool, (0.0, 0.0, 0.0)))
    target = np.eye(4)
    target[:3, 3] = arm.position(seed)
    lower, upper = arm.limits()
    rng = np.random.default_rng(rng_seed)

    found: list[np.ndarray] = []
    for _ in range(attempts):
        if len(found) >= count:
            break
        start = found[rng.integers(len(found))] if found and rng.random() < 0.4 \
            else seed
        walked = np.clip(
            start + _nullspace_step(arm, start, rng, rng.uniform(0.3, 1.5)),
            lower, upper)
        result = kin.solve_ik(arm, target, seed=walked, orientation=False,
                              restarts=1, position_tolerance=tolerance)
        if result.position_error > tolerance:
            continue
        if any(np.max(np.abs(result.q - other)) < separation
               for other in [seed, *found]):
            continue
        found.append(result.q)
    return found


def _logdet(matrix: np.ndarray) -> float:
    sign, value = np.linalg.slogdet(matrix)
    return value if sign > 0 else -np.inf


def choose(chain: kin.Chain, seed, pool, count: int, *,
           tool=(0.0, 0.0, 0.0), fit_tool: bool = False,
           free_point: bool = True, locked=()) -> list[np.ndarray]:
    """Pick the ``count`` poses from ``pool`` that pin the offsets down best.

    Greedy D-optimality: at each step take the candidate that most increases
    the determinant of the information matrix, restricted to the subspace the
    whole pool can constrain at all. Restricting matters - the determinant
    over the full parameter set is zero for every subset, because the first
    joint is unobservable no matter what is captured, and a criterion that is
    always zero ranks nothing.
    """
    pool = [np.asarray(q, dtype=float) for q in pool]
    if not pool or count <= 0:
        return []
    plan = layout(chain, fit_tool=fit_tool, free_point=free_point,
                  locked=locked)
    offsets = np.zeros(plan.joints)

    def block(q) -> np.ndarray:
        return _jacobian(chain, [Pose(q)], offsets, tool, plan)[:, plan.free]

    blocks = [block(q) for q in pool]
    stacked = np.vstack(blocks)
    _, singular, vt = np.linalg.svd(stacked, full_matrices=False)
    keep = singular > singular[0] * 1e-6 if singular.size else np.zeros(0, bool)
    basis = vt[keep].T
    if basis.size == 0:
        return pool[:count]

    projected = [basis.T @ (b.T @ b) @ basis for b in blocks]
    information = basis.T @ (block(seed).T @ block(seed)) @ basis
    information += np.eye(basis.shape[1]) * 1e-12

    chosen: list[int] = []
    for _ in range(min(count, len(pool))):
        scores = [(-np.inf if i in chosen
                   else _logdet(information + projected[i]), i)
                  for i in range(len(pool))]
        best, index = max(scores)
        if not np.isfinite(best):
            break
        information = information + projected[index]
        chosen.append(index)
    return [pool[i] for i in chosen]


def variants(chain: kin.Chain, seed, count: int = 8, *,
             tool=(0.0, 0.0, 0.0), fit_tool: bool = False,
             free_point: bool = True, locked=(), separation: float = SEPARATION,
             rng_seed: int = 0) -> list[np.ndarray]:
    """``count`` alternative arm poses reaching the same tool-tip point."""
    pool = candidates(chain, seed, tool=tool, count=max(count * 8, 40),
                      separation=separation, rng_seed=rng_seed)
    return choose(chain, seed, pool, count, tool=tool, fit_tool=fit_tool,
                  free_point=free_point, locked=locked)
