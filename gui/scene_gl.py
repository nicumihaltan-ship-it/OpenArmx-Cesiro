"""Hardware-accelerated 3D view of the arm.

Built on ``pyqtgraph.opengl``, which brings orbit/pan/zoom and a shaded mesh
item for free. The alternative - projecting 43k triangles per frame in
software - measured about 660 ms a frame, so the meshes need a GPU.

Mesh items are created once and then only re-transformed as the joints move.
Rebuilding vertex buffers every tick would throw away most of the benefit of
being on the GPU at all.
"""

from __future__ import annotations

import logging

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from OpenGL.GL import (
    GL_ALPHA_TEST, GL_BLEND, GL_CULL_FACE, GL_DEPTH_TEST,
)
from PySide6.QtGui import QMatrix4x4

log = logging.getLogger(__name__)

LINK_COLOR = (0.62, 0.67, 0.72, 1.0)
SKELETON_COLOR = (0.20, 0.24, 0.28, 1.0)
TCP_COLOR = (0.84, 0.15, 0.16, 1.0)
MEASURED_COLOR = (1.00, 0.60, 0.00, 1.0)
ORIGIN_COLOR = (1.00, 0.85, 0.10, 1.0)
PREVIEW_COLOR = (0.10, 0.65, 0.45, 1.0)
TARGET_COLOR = (0.55, 0.25, 0.75, 1.0)
#: X/Y/Z axis colours for the little frame triads.
AXIS_COLORS = np.array([[0.84, 0.15, 0.16, 1.0],
                        [0.17, 0.63, 0.17, 1.0],
                        [0.12, 0.47, 0.71, 1.0]])


def _qmatrix(matrix: np.ndarray) -> QMatrix4x4:
    return QMatrix4x4(*np.asarray(matrix, dtype=float).ravel())


def _translation(position: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, 3] = position
    return out


class SceneGL(gl.GLViewWidget):
    """Solid arm meshes plus the skeleton, joint frames and sample markers."""

    def __init__(self, parent=None):
        # The oscilloscope sets pyqtgraph's global background to None, which
        # means "use the widget palette" for a 2D plot but is simply not a
        # colour as far as GLViewWidget is concerned - it raises during
        # construction. Swap in a real colour just for this call rather than
        # changing what the 2D plots look like.
        previous = pg.getConfigOption("background")
        if previous is None:
            pg.setConfigOption("background", "#fbfbfb")
        try:
            super().__init__(parent)
        finally:
            pg.setConfigOption("background", previous)

        self.setMinimumSize(320, 260)
        # Explicit rather than inherited: the global option is whatever the
        # oscilloscope last set it to.
        self.setBackgroundColor("#eef1f4")
        self.setCameraPosition(distance=2.0, elevation=18, azimuth=45)

        self._grid = gl.GLGridItem()
        self._grid.setSize(2.0, 2.0)
        self._grid.setSpacing(0.1, 0.1)
        self.addItem(self._grid)

        # The base frame - everything the tip pose is reported relative to.
        # It sits at the foot of the robot, usually right inside the base
        # mesh, so it is drawn without depth testing and last of all:
        # a landmark you cannot see is not a landmark.
        self._origin_radius = 0.02
        self._origin = gl.GLMeshItem(
            meshdata=gl.MeshData.sphere(rows=16, cols=32,
                                        radius=self._origin_radius),
            smooth=True, shader="shaded", color=ORIGIN_COLOR)
        self._origin.setGLOptions(self._overlay_options())
        self._origin.setDepthValue(10)
        self.addItem(self._origin)

        # The tool tip gets the same always-visible treatment: it is the one
        # thing you are usually looking for, and a scatter point of it
        # disappears behind the arm's own geometry from half the angles.
        self._tip_radius = 0.018
        self._tip = gl.GLMeshItem(
            meshdata=gl.MeshData.sphere(rows=16, cols=32,
                                        radius=self._tip_radius),
            smooth=True, shader="shaded", color=TCP_COLOR)
        self._tip.setGLOptions(self._overlay_options())
        self._tip.setDepthValue(10)
        self._tip.setVisible(False)
        self.addItem(self._tip)

        # Origin -> tip broken into its three orthogonal components, so the
        # tip's position can be read off the scene rather than only off the
        # numbers below it.
        self._components = gl.GLLinePlotItem(
            pos=np.zeros((0, 3)), color=np.zeros((0, 4)), width=2.5,
            antialias=True, mode="lines")
        self._components.setGLOptions(self._overlay_options())
        self._components.setDepthValue(9)
        self.addItem(self._components)

        #: link name -> [(item, mesh-local origin transform)]
        self._meshes: dict[str, list[tuple[gl.GLMeshItem, np.ndarray]]] = {}

        self._skeleton = gl.GLLinePlotItem(
            pos=np.zeros((0, 3)), color=SKELETON_COLOR, width=3.0,
            antialias=True, mode="line_strip")
        self.addItem(self._skeleton)

        self._axes = gl.GLLinePlotItem(
            pos=np.zeros((0, 3)), color=np.zeros((0, 4)), width=2.0,
            antialias=True, mode="lines")
        self.addItem(self._axes)

        self._points = gl.GLScatterPlotItem(
            pos=np.zeros((0, 3)), color=np.zeros((0, 4)), size=9.0)
        self.addItem(self._points)

        # The pose an IK solution would move to, drawn alongside the live
        # one so the two can be compared before anything is commanded.
        self._preview = gl.GLLinePlotItem(
            pos=np.zeros((0, 3)), color=PREVIEW_COLOR, width=3.0,
            antialias=True, mode="line_strip")
        self.addItem(self._preview)

    # -- markers ----------------------------------------------------------

    @staticmethod
    def _overlay_options() -> dict:
        """Draw through solid geometry instead of being hidden by it.

        Back-face culling stands in for the depth test on the spheres: they
        are convex, so discarding back faces leaves exactly the visible
        hemisphere and the shading still reads. Without it the far side
        paints over the near side and a sphere looks like a flat disc.
        """
        return {
            GL_DEPTH_TEST: False,
            GL_BLEND: False,
            GL_ALPHA_TEST: False,
            GL_CULL_FACE: True,
        }

    def set_tip(self, position) -> None:
        """Place the tool-tip marker, or hide it with ``None``."""
        if position is None:
            self._tip.setVisible(False)
            return
        self._tip.setVisible(True)
        self._tip.setTransform(_qmatrix(
            _translation(np.asarray(position, dtype=float))))

    def set_components(self, position) -> None:
        """Draw origin -> tip as three orthogonal X, Y, Z legs."""
        if position is None:
            self._components.setData(pos=np.zeros((0, 3)),
                                     color=np.zeros((0, 4)))
            return
        x, y, z = (float(v) for v in position)
        corners = np.array([
            [0.0, 0.0, 0.0],        # origin
            [x, 0.0, 0.0],          # along X
            [x, y, 0.0],            # then Y
            [x, y, z],              # then Z, landing on the tip
        ])
        # 'lines' mode takes vertex pairs, one pair per leg.
        pos = np.repeat(corners, 2, axis=0)[1:-1]
        color = np.repeat(AXIS_COLORS, 2, axis=0)
        self._components.setData(pos=pos, color=color, width=2.5)

    def set_marker_radius(self, radius: float) -> None:
        """Resize both spheres to suit the robot they sit under."""
        radius = max(float(radius), 1e-4)
        if abs(radius - self._origin_radius) > 1e-9:
            self._origin_radius = radius
            self._origin.setMeshData(
                meshdata=gl.MeshData.sphere(rows=16, cols=32, radius=radius))
        tip = radius * 0.8
        if abs(tip - self._tip_radius) > 1e-9:
            self._tip_radius = tip
            self._tip.setMeshData(
                meshdata=gl.MeshData.sphere(rows=16, cols=32, radius=tip))

    # -- origin marker ----------------------------------------------------

    def set_origin_visible(self, visible: bool) -> None:
        self._origin.setVisible(bool(visible))

    # -- meshes -----------------------------------------------------------

    def clear_meshes(self) -> None:
        for entries in self._meshes.values():
            for item, _ in entries:
                self.removeItem(item)
        self._meshes = {}

    def add_mesh(self, link: str, triangles: np.ndarray,
                 origin: np.ndarray, scale=None) -> None:
        """Register one link's mesh, in the link's own frame."""
        if triangles is None or len(triangles) == 0:
            return
        vertices = triangles
        if scale is not None and not np.allclose(scale, 1.0):
            vertices = vertices * np.asarray(scale, dtype=float)
        data = gl.MeshData(vertexes=np.ascontiguousarray(vertices,
                                                        dtype=np.float32))
        item = gl.GLMeshItem(meshdata=data, smooth=False, shader="shaded",
                             color=LINK_COLOR, drawEdges=False)
        item.setTransform(_qmatrix(origin))
        self.addItem(item)
        self._meshes.setdefault(link, []).append((item, np.asarray(origin)))

    @property
    def mesh_count(self) -> int:
        return sum(len(v) for v in self._meshes.values())

    # -- per-frame update -------------------------------------------------

    def set_pose(self, transforms: dict[str, np.ndarray]) -> None:
        """Place every registered mesh using its link's world transform."""
        for link, entries in self._meshes.items():
            world = transforms.get(link)
            for item, origin in entries:
                if world is None:
                    item.setVisible(False)
                    continue
                item.setVisible(True)
                item.setTransform(_qmatrix(world @ origin))

    def set_skeleton(self, points: np.ndarray) -> None:
        points = np.asarray(points, dtype=float)
        self._skeleton.setData(pos=points, color=SKELETON_COLOR, width=3.0)

    def set_frames(self, frames) -> None:
        """``frames`` is a sequence of (4x4 transform, axis length)."""
        starts, ends, colors = [], [], []
        for transform, length in frames:
            origin = transform[:3, 3]
            for column in range(3):
                starts.append(origin)
                ends.append(origin + transform[:3, column] * length)
                colors.append(AXIS_COLORS[column])
        if not starts:
            self._axes.setData(pos=np.zeros((0, 3)), color=np.zeros((0, 4)))
            return
        # 'lines' mode consumes vertex pairs, so interleave start/end.
        pos = np.empty((len(starts) * 2, 3))
        pos[0::2], pos[1::2] = np.array(starts), np.array(ends)
        color = np.repeat(np.array(colors), 2, axis=0)
        self._axes.setData(pos=pos, color=color, width=2.0)

    def set_preview(self, points) -> None:
        """Draw a candidate arm pose, or clear it with ``None``."""
        if points is None or len(points) == 0:
            self._preview.setData(pos=np.zeros((0, 3)), color=PREVIEW_COLOR)
            return
        self._preview.setData(pos=np.asarray(points, dtype=float),
                              color=PREVIEW_COLOR, width=3.0)

    def set_points(self, points) -> None:
        """``points`` is a sequence of (xyz, rgba, size)."""
        if not points:
            self._points.setData(pos=np.zeros((0, 3)), color=np.zeros((0, 4)),
                                 size=np.zeros(0))
            return
        self._points.setData(
            pos=np.array([p for p, _, _ in points], dtype=float),
            color=np.array([c for _, c, _ in points], dtype=float),
            size=np.array([s for _, _, s in points], dtype=float))

    # -- framing ----------------------------------------------------------

    def look_at(self, centre, span: float) -> None:
        from pyqtgraph import Vector
        self.opts["center"] = Vector(*[float(v) for v in centre])
        self.setCameraPosition(distance=max(span * 1.7, 0.4))

    def fit(self, points) -> None:
        """Frame the camera on a cloud of world points."""
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return
        low, high = points.min(axis=0), points.max(axis=0)
        self.look_at((low + high) / 2.0, float(np.linalg.norm(high - low)))

    def reset_view(self) -> None:
        self.setCameraPosition(elevation=18, azimuth=45)
