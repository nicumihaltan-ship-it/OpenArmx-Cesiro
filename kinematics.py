"""Forward kinematics from a URDF.

Enough of URDF to answer one question: given the joint angles the motors are
reporting, where is the tool tip relative to the base? That is the basis for
checking a calibration - command a pose, compute where the tip should be,
measure where it actually is, and the difference identifies the joint zero
offsets.

Deliberately dependency-free beyond numpy. ``urdfpy`` is unmaintained and
pins an old numpy, ``yourdfpy`` drags in trimesh, and neither survives a
PyInstaller freeze without a fight. A serial chain of revolute joints needs
less code than either integration.

The description file itself is not shipped with this tool - OpenArmX's is
CC BY-NC-SA, so the path to a local copy is configuration, not content.
"""

from __future__ import annotations

import logging
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

log = logging.getLogger(__name__)

#: Joint types that consume a value from the joint vector.
ACTUATED = ("revolute", "continuous", "prismatic")


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw, i.e. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def matrix_to_rpy(matrix: np.ndarray) -> tuple[float, float, float]:
    """Inverse of :func:`rpy_to_matrix`, for reporting an orientation."""
    pitch = np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
    if np.isclose(np.cos(pitch), 0.0, atol=1e-9):
        # Gimbal lock: roll and yaw are no longer separable, so pin yaw.
        return float(np.arctan2(-matrix[1, 2], matrix[1, 1])), float(pitch), 0.0
    return (float(np.arctan2(matrix[2, 1], matrix[2, 2])), float(pitch),
            float(np.arctan2(matrix[1, 0], matrix[0, 0])))


def rotation_vector(matrix: np.ndarray) -> np.ndarray:
    """SO(3) log map: the axis-angle vector of a rotation matrix.

    This is how an orientation error is expressed as three numbers the
    solver can drive to zero, the rotational counterpart of a position
    difference.
    """
    cosine = np.clip((np.trace(matrix[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    if angle < 1e-9:
        return np.zeros(3)
    if angle > np.pi - 1e-6:
        # Near half a turn the antisymmetric part vanishes and its axis is
        # numerically worthless; recover the axis from the symmetric part.
        symmetric = (matrix[:3, :3] + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(symmetric), 0.0, None))
        dominant = int(np.argmax(axis))
        if axis[dominant] > 1e-9:
            axis = symmetric[:, dominant] / axis[dominant]
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return np.zeros(3)
        return axis / norm * angle
    axis = np.array([matrix[2, 1] - matrix[1, 2],
                     matrix[0, 2] - matrix[2, 0],
                     matrix[1, 0] - matrix[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def pose(xyz, rpy) -> np.ndarray:
    """A 4x4 transform from a position and a roll-pitch-yaw triple."""
    return _transform(np.asarray(xyz, dtype=float), rpy_to_matrix(*rpy))


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about an arbitrary unit axis."""
    x, y, z = axis
    c, s, t = np.cos(angle), np.sin(angle), 1.0 - np.cos(angle)
    return np.array([
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def _transform(translation, rotation) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


def _floats(text: str | None, default: tuple) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in text.split()], dtype=float)


@dataclass
class Geometry:
    """One ``<visual>`` or ``<collision>`` mesh hanging off a link."""

    mesh: str                           # the filename exactly as the URDF has it
    origin: np.ndarray                  # 4x4, link frame -> mesh frame
    scale: np.ndarray                   # per-axis mesh scale


@dataclass
class Link:
    name: str
    visuals: list[Geometry] = field(default_factory=list)
    collisions: list[Geometry] = field(default_factory=list)

    def geometry(self, prefer: str = "collision") -> list[Geometry]:
        """Meshes for this link, falling back to the other kind."""
        first = self.collisions if prefer == "collision" else self.visuals
        second = self.visuals if prefer == "collision" else self.collisions
        return first or second


@dataclass
class Joint:
    """One URDF joint: a fixed origin followed by the motion it allows."""

    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray                  # 4x4, parent frame -> joint frame
    axis: np.ndarray                    # unit vector in the joint frame
    lower: float | None = None
    upper: float | None = None

    @property
    def actuated(self) -> bool:
        return self.type in ACTUATED

    def transform(self, value: float = 0.0) -> np.ndarray:
        """Parent frame -> child frame at this joint value."""
        if self.type in ("revolute", "continuous"):
            return self.origin @ _transform((0, 0, 0),
                                            axis_rotation(self.axis, value))
        if self.type == "prismatic":
            return self.origin @ _transform(self.axis * value, np.eye(3))
        return self.origin

    def clamp(self, value: float) -> float:
        if self.lower is not None and value < self.lower:
            return self.lower
        if self.upper is not None and value > self.upper:
            return self.upper
        return value


class Robot:
    """A parsed URDF tree."""

    def __init__(self, name: str, joints: list[Joint], links: dict[str, Link]):
        self.name = name
        self.links = links
        self.joints = {j.name: j for j in joints}
        self._by_child = {j.child: j for j in joints}
        self._children: dict[str, list[str]] = {}
        for joint in joints:
            self._children.setdefault(joint.parent, []).append(joint.child)

    # -- parsing ----------------------------------------------------------

    @classmethod
    def from_urdf(cls, path: str | Path) -> "Robot":
        root = ET.parse(Path(path)).getroot()
        if root.tag != "robot":
            raise ValueError(f"{path} is not a URDF (root tag is <{root.tag}>)")

        joints = []
        for element in root.findall("joint"):
            origin = element.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None,
                          (0, 0, 0))
            rpy = _floats(origin.get("rpy") if origin is not None else None,
                          (0, 0, 0))
            axis_el = element.find("axis")
            axis = _floats(axis_el.get("xyz") if axis_el is not None else None,
                           (1, 0, 0))
            norm = np.linalg.norm(axis)
            if norm > 0:
                axis = axis / norm
            limit = element.find("limit")
            joints.append(Joint(
                name=element.get("name"),
                type=element.get("type"),
                parent=element.find("parent").get("link"),
                child=element.find("child").get("link"),
                origin=_transform(xyz, rpy_to_matrix(*rpy)),
                axis=axis,
                lower=float(limit.get("lower")) if limit is not None
                and limit.get("lower") is not None else None,
                upper=float(limit.get("upper")) if limit is not None
                and limit.get("upper") is not None else None,
            ))
        links: dict[str, Link] = {}
        for element in root.findall("link"):
            link = Link(element.get("name"))
            for kind, bucket in (("visual", link.visuals),
                                 ("collision", link.collisions)):
                for el in element.findall(kind):
                    mesh = el.find("geometry/mesh")
                    if mesh is None or not mesh.get("filename"):
                        continue        # boxes/cylinders are not drawn
                    origin = el.find("origin")
                    bucket.append(Geometry(
                        mesh=mesh.get("filename"),
                        origin=_transform(
                            _floats(origin.get("xyz") if origin is not None
                                    else None, (0, 0, 0)),
                            rpy_to_matrix(*_floats(
                                origin.get("rpy") if origin is not None
                                else None, (0, 0, 0)))),
                        scale=_floats(mesh.get("scale"), (1, 1, 1)),
                    ))
            links[link.name] = link
        return cls(root.get("name", "robot"), joints, links)

    # -- structure --------------------------------------------------------

    @property
    def root(self) -> str:
        """The only link that is nobody's child."""
        for link in self.links:
            if link not in self._by_child:
                return link
        raise ValueError("no root link - the URDF tree has a cycle")

    def chain(self, tip: str, base: str | None = None) -> "Chain":
        """The joints from ``base`` down to ``tip``, in that order."""
        if tip not in self.links:
            raise KeyError(f"no link named {tip!r}")
        if base is None:
            base = self.root
        path: list[Joint] = []
        link = tip
        while link != base:
            joint = self._by_child.get(link)
            if joint is None:
                raise ValueError(f"{tip!r} is not a descendant of {base!r}")
            path.append(joint)
            link = joint.parent
        path.reverse()
        return Chain(base, tip, path)

    def tips(self) -> list[str]:
        """Leaf links - the candidates for a tool frame."""
        return [link for link in self.links if link not in self._children]

    def gripper(self, chain: "Chain") -> "Gripper | None":
        """Prismatic joints branching off ``chain`` - i.e. the fingers.

        A gripper is not part of the arm's kinematics: the fingers hang off
        a link on the way to the tool frame rather than contributing to
        where that frame is, so the IK chain rightly ignores them. They
        still need placing in the 3D view and driving from the panel, which
        is what this finds.
        """
        on_chain = {joint.child for joint in chain.joints} | {chain.base}
        in_chain = {joint.name for joint in chain.joints}
        fingers = [joint for joint in self.joints.values()
                   if joint.type == "prismatic"
                   and joint.parent in on_chain
                   and joint.name not in in_chain]
        if not fingers:
            return None
        return Gripper(joints=fingers, parent=fingers[0].parent)


@dataclass
class Gripper:
    """Finger joints driven together by one motor.

    The URDF declares the fingers as independent prismatic joints because
    URDF has no way to say "one actuator, two slides" without a ``mimic``
    tag the description does not use. On this hardware they share a motor,
    so one stroke value drives them all - their opposing axes make that
    symmetric without any extra bookkeeping.
    """

    joints: list[Joint]
    parent: str                         # the link they hang off

    @property
    def lower(self) -> float:
        return max((j.lower for j in self.joints if j.lower is not None),
                   default=0.0)

    @property
    def upper(self) -> float:
        return min((j.upper for j in self.joints if j.upper is not None),
                   default=0.0)

    def frames(self, parent_transform: np.ndarray,
               stroke: float) -> list[tuple[str, np.ndarray]]:
        """World transform of each finger link at the given stroke."""
        return [(joint.child, parent_transform @ joint.transform(stroke))
                for joint in self.joints]

    def separation(self, stroke: float) -> float:
        """Distance between the two finger frames, for a two-finger hand.

        Reported alongside the stroke because the stroke is per-finger and
        reads as half of what the hand visibly does.
        """
        if len(self.joints) != 2:
            return float("nan")
        first, second = self.joints
        one = (first.origin @ _transform(first.axis * stroke, np.eye(3)))[:3, 3]
        two = (second.origin @ _transform(second.axis * stroke, np.eye(3)))[:3, 3]
        return float(np.linalg.norm(one - two))


class Chain:
    """A serial run of joints, base to tip."""

    def __init__(self, base: str, tip: str, joints: list[Joint], tool=None):
        self.base = base
        self.tip = tip
        self.joints = joints
        self.actuated = [j for j in joints if j.actuated]
        #: Tip frame -> tool frame, 4x4. Applied by :meth:`fk`, and so by
        #: everything built on it - :meth:`position`, both Jacobians and
        #: :func:`solve_ik` - which is what lets the solver aim a tool tip
        #: rather than the flange it is bolted to. :meth:`frames` is
        #: deliberately left alone: it enumerates *link* frames for drawing,
        #: and a tool is not a link.
        self.tool = np.eye(4) if tool is None else np.asarray(tool, dtype=float)

    def with_tool(self, tool) -> "Chain":
        """This chain measured at ``tool`` instead of at the tip link."""
        return Chain(self.base, self.tip, self.joints, tool)

    def __len__(self) -> int:
        return len(self.actuated)

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.actuated]

    # -- kinematics -------------------------------------------------------

    def fk(self, q) -> np.ndarray:
        """Base -> tool transform for the actuated joint values ``q``."""
        return self.frames(q)[-1][1] @ self.tool

    def frames(self, q) -> list[tuple[str, np.ndarray]]:
        """Every link frame along the chain, base first.

        The intermediate frames are what the 3D view draws, so they come out
        of the same walk that produces the tip - a separate drawing path
        would be free to disagree with the number on screen.
        """
        values = self._vector(q)
        transform = np.eye(4)
        out = [(self.base, transform)]
        index = 0
        for joint in self.joints:
            value = 0.0
            if joint.actuated:
                value = values[index]
                index += 1
            transform = transform @ joint.transform(value)
            out.append((joint.child, transform))
        return out

    def position(self, q) -> np.ndarray:
        return self.fk(q)[:3, 3]

    def pose_jacobian(self, q, eps: float = 1e-6) -> np.ndarray:
        """Numerical 6xN Jacobian: three position rows, three rotation rows."""
        values = self._vector(q)
        base = self.fk(values)
        out = np.zeros((6, len(values)))
        for i in range(len(values)):
            shifted = values.copy()
            shifted[i] += eps
            moved = self.fk(shifted)
            out[:3, i] = (moved[:3, 3] - base[:3, 3]) / eps
            out[3:, i] = rotation_vector(
                moved[:3, :3] @ base[:3, :3].T) / eps
        return out

    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Lower and upper bounds of the actuated joints, unbounded as inf."""
        lower = np.array([-np.inf if j.lower is None else j.lower
                          for j in self.actuated])
        upper = np.array([np.inf if j.upper is None else j.upper
                          for j in self.actuated])
        return lower, upper

    def jacobian(self, q, eps: float = 1e-6) -> np.ndarray:
        """Numerical 3xN position Jacobian.

        Numerical rather than analytic on purpose: the chain is short, this
        runs once per solver iteration, and an analytic Jacobian is one more
        thing that can silently disagree with :meth:`fk`.
        """
        values = self._vector(q)
        base = self.position(values)
        out = np.zeros((3, len(values)))
        for i in range(len(values)):
            shifted = values.copy()
            shifted[i] += eps
            out[:, i] = (self.position(shifted) - base) / eps
        return out

    def _vector(self, q) -> np.ndarray:
        if isinstance(q, dict):
            return np.array([float(q.get(j.name, 0.0)) for j in self.actuated])
        values = np.asarray(q, dtype=float).ravel()
        if values.size != len(self.actuated):
            raise ValueError(
                f"{self.tip} needs {len(self.actuated)} joint values, got "
                f"{values.size}")
        return values


# --------------------------------------------------------------------------
# Meshes
#
# URDF mesh references are ROS-flavoured: `package://<pkg>/<path>`, which only
# resolves against a ROS workspace. There is no workspace here, so resolution
# is a search over folders the user points at.
# --------------------------------------------------------------------------

def resolve_mesh(filename: str, roots) -> Path | None:
    """Find ``filename`` under one of ``roots``, or return ``None``.

    Handles the three spellings that turn up in practice: a ``package://``
    URI, a ``file://`` URI and a plain relative path. For ``package://`` both
    interpretations of a root are tried - the root may be the package folder
    itself, or the folder that contains it.
    """
    if not filename:
        return None
    text = filename.replace("\\", "/")
    candidates: list[str] = []

    if text.startswith("package://"):
        remainder = text[len("package://"):]
        package, _, rest = remainder.partition("/")
        candidates += [rest, f"{package}/{rest}", remainder]
    elif text.startswith("file://"):
        direct = Path(text[len("file://"):].lstrip("/"))
        if direct.exists():
            return direct
        candidates.append(text[len("file://"):].lstrip("/"))
    else:
        direct = Path(text)
        if direct.is_absolute() and direct.exists():
            return direct
        candidates.append(text)

    # The bare filename is the last resort: descriptions get reorganised and
    # a flat folder of STLs is a perfectly normal thing to be handed.
    leaf = text.rsplit("/", 1)[-1]

    for root in roots:
        if not root:
            continue
        root = Path(root)
        for candidate in candidates:
            path = root / candidate
            if path.is_file():
                return path
        matches = sorted(root.rglob(leaf))
        if matches:
            return matches[0]
    return None


def load_stl(path) -> np.ndarray:
    """Triangles from a binary or ASCII STL, as an ``(N, 3, 3)`` array."""
    path = Path(path)
    size = path.stat().st_size
    with open(path, "rb") as handle:
        header = handle.read(84)
        if len(header) == 84:
            count = struct.unpack("<I", header[80:84])[0]
            # File size is the only reliable discriminator. A leading "solid"
            # does not mean ASCII - plenty of binary writers emit it - and in
            # an ASCII file bytes 80..84 are ordinary text that decodes to an
            # enormous count. Sizing the read off that count before checking
            # it asks for gigabytes and dies with MemoryError.
            if size == 84 + count * 50:
                return _binary_stl(handle.read(count * 50), count)
    return _ascii_stl(path)


def _binary_stl(body: bytes, count: int) -> np.ndarray:
    records = np.frombuffer(body, dtype=np.uint8).reshape(count, 50)
    # Bytes 0-11 are the facet normal, 12-47 the three vertices, 48-49 the
    # attribute word. Only the vertices are wanted.
    vertices = records[:, 12:48].copy().view("<f4").reshape(count, 3, 3)
    return vertices.astype(np.float64)


def _ascii_stl(path: Path) -> np.ndarray:
    points = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            parts = line.split()
            if parts and parts[0] == "vertex" and len(parts) >= 4:
                points.append([float(v) for v in parts[1:4]])
    array = np.array(points, dtype=np.float64)
    if array.size == 0 or len(array) % 3:
        raise ValueError(f"{path.name} is not a well-formed ASCII STL")
    return array.reshape(-1, 3, 3)


#: A mesh whose extent exceeds this many times the robot's own kinematic
#: reach is assumed to be declaring the wrong units.
UNIT_MISMATCH_RATIO = 20.0


def unit_correction(triangles: np.ndarray, scale, reference: float,
                    ratio: float = UNIT_MISMATCH_RATIO) -> float | None:
    """``0.001`` when a mesh looks like millimetres declared as metres.

    Descriptions mix units more often than they admit. OpenArmX's own URDF
    declares ``scale="1.0 1.0 1.0"`` on a body mesh authored in millimetres,
    which draws a 773-metre column and swallows the whole scene, while the
    arm meshes beside it really are metres and the hand meshes correctly
    declare 0.001.

    Guessing is only safe because the alternative is so obviously broken: a
    single link cannot be twenty times the reach of the whole robot. Anything
    less clear-cut is left alone, and the caller is expected to say out loud
    that a correction was applied.
    """
    if triangles is None or len(triangles) == 0 or reference <= 0:
        return None
    scaled = triangles.reshape(-1, 3) * np.asarray(scale, dtype=float)
    extent = float(np.linalg.norm(scaled.max(axis=0) - scaled.min(axis=0)))
    if extent <= ratio * reference:
        return None
    # Only correct when millimetres actually explain it - a mesh that is
    # still absurd after the conversion is a different problem.
    return 0.001 if extent * 0.001 <= ratio * reference else None


class MeshCache:
    """Loads each mesh file once and remembers what could not be found."""

    def __init__(self):
        self._loaded: dict[Path, np.ndarray] = {}
        self.missing: set[str] = set()

    def triangles(self, filename: str, roots) -> np.ndarray | None:
        path = resolve_mesh(filename, roots)
        if path is None:
            self.missing.add(filename)
            return None
        if path not in self._loaded:
            try:
                self._loaded[path] = load_stl(path)
            except Exception as exc:
                log.debug("could not load %s: %s", path, exc)
                self.missing.add(filename)
                self._loaded[path] = np.zeros((0, 3, 3))
        return self._loaded[path]

    def clear(self) -> None:
        self._loaded.clear()
        self.missing.clear()


# --------------------------------------------------------------------------
# Inverse kinematics
# --------------------------------------------------------------------------

#: Metres of position error considered as costly as one radian of orientation
#: error. Position and orientation are not the same physical quantity, so the
#: 6-vector needs a length scale before least squares can weigh the two.
ROTATION_SCALE = 0.15


@dataclass
class IKResult:
    q: np.ndarray                       # solved joint values, rad
    position_error: float               # metres
    orientation_error: float            # radians
    converged: bool
    iterations: int
    at_limit: list[str]                 # joints resting on a URDF limit
    reached: np.ndarray                 # the pose actually achieved, 4x4


def solve_ik(chain: Chain, target: np.ndarray, seed=None,
             orientation: bool = True, iterations: int = 300,
             position_tolerance: float = 2e-4,
             orientation_tolerance: float = 2e-3,
             damping: float = 0.02, max_step: float = 0.2,
             rotation_scale: float = ROTATION_SCALE,
             restarts: int = 6) -> IKResult:
    """Joint values putting the tip at ``target``, respecting URDF limits.

    Damped least squares, seeded from the current pose. The damping is what
    makes it safe to point at an unreachable target: an undamped pseudo-
    inverse blows up near a singularity and throws the arm somewhere
    unrelated, whereas this one just stops short and reports how far off it
    ended up.

    On a 7-joint chain with a full 6-DOF target there is still a null space,
    and the minimum-norm step DLS takes resolves it by moving as little as
    possible from the seed. That is also the safest choice, since the arm
    stays near where it already is.

    Restarts are deterministic - a seeded generator, not the global one - so
    the same target always produces the same solution. A solver that quietly
    picked a different pose each time would be unusable for anything that
    drives hardware.
    """
    target = np.asarray(target, dtype=float)
    lower, upper = chain.limits()
    n = len(chain)
    rows = 6 if orientation else 3

    if seed is None:
        seed = np.zeros(n)
    seed = np.clip(np.asarray(seed, dtype=float).ravel(), lower, upper)

    def error(q: np.ndarray) -> np.ndarray:
        current = chain.fk(q)
        out = np.zeros(rows)
        out[:3] = target[:3, 3] - current[:3, 3]
        if orientation:
            out[3:] = rotation_vector(
                target[:3, :3] @ current[:3, :3].T) * rotation_scale
        return out

    generator = np.random.default_rng(0)
    best: IKResult | None = None

    for attempt in range(max(1, restarts)):
        if attempt == 0:
            q = seed.copy()
        else:
            # Perturb around the seed, then fall back to sampling the whole
            # box - near targets want a nearby branch, far ones need width.
            span = np.where(np.isfinite(upper - lower), upper - lower, 2 * np.pi)
            spread = 0.25 if attempt < restarts // 2 else 1.0
            q = np.clip(seed + generator.uniform(-spread, spread, n) * span / 2,
                        lower, upper)

        residual = error(q)
        used = 0
        for used in range(1, iterations + 1):
            jac = chain.pose_jacobian(q)[:rows]
            if orientation:
                jac = jac.copy()
                jac[3:] *= rotation_scale
            # dq = J^T (J J^T + lambda^2 I)^-1 e - the 3x3/6x6 form, which is
            # the cheap one when the chain is redundant.
            lhs = jac @ jac.T + (damping ** 2) * np.eye(rows)
            step = jac.T @ np.linalg.solve(lhs, residual)
            longest = np.max(np.abs(step)) if step.size else 0.0
            if longest > max_step:
                step *= max_step / longest
            q = np.clip(q + step, lower, upper)
            residual = error(q)
            if np.max(np.abs(step)) < 1e-10:
                break

        reached = chain.fk(q)
        position_error = float(np.linalg.norm(target[:3, 3] - reached[:3, 3]))
        orientation_error = float(np.linalg.norm(rotation_vector(
            target[:3, :3] @ reached[:3, :3].T))) if orientation else 0.0
        converged = (position_error <= position_tolerance
                     and orientation_error <= orientation_tolerance)
        at_limit = [joint.name for joint, value, low, high
                    in zip(chain.actuated, q, lower, upper)
                    if (np.isfinite(low) and abs(value - low) < 1e-6)
                    or (np.isfinite(high) and abs(value - high) < 1e-6)]
        candidate = IKResult(q=q, position_error=position_error,
                             orientation_error=orientation_error,
                             converged=converged, iterations=used,
                             at_limit=at_limit, reached=reached)
        if best is None or _ik_score(candidate, rotation_scale) < \
                _ik_score(best, rotation_scale):
            best = candidate
        if converged:
            break

    return best


def _ik_score(result: IKResult, rotation_scale: float) -> float:
    return result.position_error + result.orientation_error * rotation_scale


@dataclass
class Sample:
    """One calibration observation: what the motors said, where the tip was."""

    q: np.ndarray                       # joint readings, rad
    measured: np.ndarray                # tip position in the base frame, m

    def __post_init__(self):
        self.q = np.asarray(self.q, dtype=float).ravel()
        self.measured = np.asarray(self.measured, dtype=float).ravel()


@dataclass
class OffsetFit:
    offsets: np.ndarray                 # rad, added to each joint reading
    residuals: np.ndarray               # per-sample tip error after the fit, m
    initial: np.ndarray                 # per-sample tip error before, m
    iterations: int
    converged: bool

    @property
    def rms(self) -> float:
        return float(np.sqrt(np.mean(self.residuals ** 2))) if \
            self.residuals.size else 0.0

    @property
    def rms_before(self) -> float:
        return float(np.sqrt(np.mean(self.initial ** 2))) if \
            self.initial.size else 0.0


def solve_offsets(chain: Chain, samples: list[Sample], iterations: int = 60,
                  damping: float = 1e-4, tolerance: float = 1e-9) -> OffsetFit:
    """Least-squares joint zero offsets from measured tip positions.

    Finds the per-joint constant ``d`` minimising the tip error over every
    sample, by damped Gauss-Newton. Damping matters: with few samples the
    problem is under-determined - a 7-joint arm needs at least three
    well-spread poses before all seven offsets are observable - and an
    undamped step would chase an arbitrary direction in the null space.

    The caller is responsible for spreading the poses out. Three samples in
    nearly the same pose will report a tiny residual and offsets that mean
    nothing.
    """
    if not samples:
        raise ValueError("no samples to fit")
    n = len(chain)

    def errors(offsets: np.ndarray) -> np.ndarray:
        return np.concatenate([
            chain.position(s.q + offsets) - s.measured for s in samples])

    offsets = np.zeros(n)
    initial = errors(offsets)
    residual = initial
    converged = False
    used = 0
    for used in range(1, iterations + 1):
        jac = np.vstack([chain.jacobian(s.q + offsets) for s in samples])
        # (JtJ + lambda I) step = -Jt r
        lhs = jac.T @ jac + damping * np.eye(n)
        step = np.linalg.solve(lhs, -jac.T @ residual)
        offsets = offsets + step
        residual = errors(offsets)
        if np.max(np.abs(step)) < tolerance:
            converged = True
            break
    return OffsetFit(offsets=offsets, residuals=residual, initial=initial,
                     iterations=used, converged=converged)
