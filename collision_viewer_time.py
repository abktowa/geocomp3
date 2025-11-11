#!/usr/bin/env python3
"""
collision_viewer_time.py

Author: ChatGPT 5.0 - prompts by Christian Duncan (see discussion at end)
Interactive viewer for Project 3 (2D Collision Detection) inputs.

Features
--------
- Displays all objects at a chosen time t.
- Supports panning and zooming.
- Two time-range sliders: t0 and t1.
- Play/Pause animation from t0 to t1.
- Adjustable time step dt for animation.
- Culls objects whose bounding boxes are outside the current view
  (for better performance with large inputs).
- Focus & highlight two objects for collision inspection.
- Optional camera follow of the focus object.

New in this version
-------------------
- Command-line options to set:
    * --t0          : initial start time
    * --t1          : initial end time
    * --dt          : initial time step (can be very small, e.g., 1e-6 or 1e-9)
- '[' / ']' adjust dt by +/- 0.001 (with a tiny lower bound 1e-9).

Controls
--------
Keyboard:
  + or =       : zoom in
  - or _       : zoom out
  Arrow keys   : pan (← → ↑ ↓)
  0            : reset view to full scene
  p or r       : play / pause animation
  [            : decrease dt by 0.001 (min 1e-9)
  ]            : increase dt by 0.001
  t            : reset current time to t0
  f            : toggle following the focus object (if any)
  q            : quit

GUI:
  t0 slider    : set start time for animation
  t1 slider    : set end time for animation
  Play button  : play/pause from t0 to t1

Usage
-----
  python3 collision_viewer_time.py input.txt
  python3 collision_viewer_time.py input.txt --t0 3.5 --t1 4.0 --dt 0.0001
  python3 collision_viewer_time.py input.txt --focus-id 0 --highlight-id 4
"""

import argparse
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.widgets import Slider, Button


# ------------------------- Data structures -------------------------

@dataclass
class PathSegment:
    vx: float
    vy: float
    dt: float
    t_start: float
    t_end: float
    cum_dx: float  # displacement accumulated up to t_start
    cum_dy: float


@dataclass
class MovingObject:
    poly0: np.ndarray          # shape (k, 2), initial vertices at t=0
    bbox0: Tuple[float,float,float,float]  # (minx,maxx,miny,maxy) at t=0
    path: List[PathSegment]    # piecewise constant velocity segments
    total_time: float          # total motion time (then object stops)


# ------------------------- Parsing input -------------------------

def parse_input(path: str) -> Tuple[List[MovingObject], float]:
    """
    Parse Project 3 input file.

    Returns:
        objects: list of MovingObject
        global_tmax: max total_time over all objects
    """
    with open(path, "r") as f:
        lines = [line.strip() for line in f if line.strip()]

    if not lines:
        raise ValueError("Empty input file")

    N = int(lines[0])
    expected_lines = 1 + 2 * N
    if len(lines) < expected_lines:
        raise ValueError(
            f"Expected at least {expected_lines} non-empty lines, got {len(lines)}"
        )

    objects: List[MovingObject] = []
    idx = 1
    global_tmax = 0.0

    for obj_id in range(N):
        poly_line = lines[idx]; idx += 1
        path_line = lines[idx]; idx += 1

        # Polygon line: x1 y1 x2 y2 ... xk yk
        coords = list(map(float, poly_line.split()))
        if len(coords) % 2 != 0:
            raise ValueError(f"Polygon line for object {obj_id} has odd number of values")
        pts = np.array(coords, dtype=float).reshape(-1, 2)

        xs = pts[:, 0]
        ys = pts[:, 1]
        minx = float(xs.min())
        maxx = float(xs.max())
        miny = float(ys.min())
        maxy = float(ys.max())
        bbox0 = (minx, maxx, miny, maxy)

        # Path line: vx1 vy1 dt1 vx2 vy2 dt2 ...
        vals = list(map(float, path_line.split()))
        if len(vals) % 3 != 0:
            raise ValueError(f"Path line for object {obj_id} has length not multiple of 3")

        path: List[PathSegment] = []
        t = 0.0
        cum_dx = 0.0
        cum_dy = 0.0

        for i in range(0, len(vals), 3):
            vx, vy, dt = vals[i], vals[i+1], vals[i+2]
            if dt < 0:
                raise ValueError(f"Negative duration in object {obj_id}")
            if dt == 0:
                continue
            seg = PathSegment(
                vx=vx, vy=vy, dt=dt,
                t_start=t,
                t_end=t + dt,
                cum_dx=cum_dx,
                cum_dy=cum_dy,
            )
            path.append(seg)
            t += dt
            cum_dx += vx * dt
            cum_dy += vy * dt

        total_time = t
        global_tmax = max(global_tmax, total_time)

        objects.append(MovingObject(
            poly0=pts,
            bbox0=bbox0,
            path=path,
            total_time=total_time
        ))

    # Cap as per problem statement (though inputs should already respect this)
    global_tmax = min(global_tmax, 1_000_000.0)
    return objects, global_tmax


# ------------------------- Motion evaluation -------------------------

def displacement_at_time(obj: MovingObject, t: float) -> Tuple[float, float]:
    """
    Return the translational displacement (dx, dy) of obj at global time t.
    After obj.total_time, object stops and stays at its last position.
    """
    if not obj.path or t <= 0.0:
        return 0.0, 0.0

    if t >= obj.total_time:
        last = obj.path[-1]
        dx = last.cum_dx + last.vx * last.dt
        dy = last.cum_dy + last.vy * last.dt
        return dx, dy

    # Path length <= 100, linear scan is fine
    for seg in obj.path:
        if t < seg.t_end:
            tau = t - seg.t_start
            dx = seg.cum_dx + seg.vx * tau
            dy = seg.cum_dy + seg.vy * tau
            return dx, dy

    last = obj.path[-1]
    dx = last.cum_dx + last.vx * last.dt
    dy = last.cum_dy + last.vy * last.dt
    return dx, dy


# ------------------------- Viewer class -------------------------

class CollisionViewer:
    def __init__(
        self,
        objects: List[MovingObject],
        tmax: float,
        t0_init: float = 0.0,
        t1_init: Optional[float] = None,
        dt: float = 0.1,
        cull_margin: float = 50.0,
        focus_id: Optional[int] = None,
        highlight_id: Optional[int] = None,
    ):
        self.objects = objects
        self.tmax = max(tmax, 0.0)
        self.cull_margin = float(cull_margin)

        # Focus/highlight settings
        self.focus_id = focus_id if (focus_id is not None and 0 <= focus_id < len(objects)) else None
        if focus_id is not None and self.focus_id is None:
            print(f"Warning: focus-id {focus_id} out of range; ignoring.")
        self.highlight_id = highlight_id if (highlight_id is not None and 0 <= highlight_id < len(objects)) else None
        if highlight_id is not None and self.highlight_id is None:
            print(f"Warning: highlight-id {highlight_id} out of range; ignoring.")

        # Follow mode: if True and focus_id is set, camera centers on focus object as it moves.
        self.follow_focus = False

        # Time range initialization from CLI
        tmax_slider = self.tmax if self.tmax > 0 else 1.0
        t0_clamped = max(0.0, min(t0_init, tmax_slider))
        if t1_init is None:
            t1_clamped = tmax_slider
        else:
            t1_clamped = max(0.0, min(t1_init, tmax_slider))
        if t1_clamped < t0_clamped:
            # Enforce t0 <= t1
            t1_clamped = t0_clamped

        self.t0 = t0_clamped
        self.t1 = t1_clamped
        self.current_t = self.t0
        self.playing = False

        # dt and its bounds
        self.dt_min = 1e-9
        self.dt_max = max(self.dt_min, self.tmax if self.tmax > 0 else 1.0)
        self.dt = max(self.dt_min, min(float(dt), self.dt_max))

        # Global bounding box at t=0 for initial view
        if objects:
            all_x = np.concatenate([obj.poly0[:, 0] for obj in objects])
            all_y = np.concatenate([obj.poly0[:, 1] for obj in objects])
        else:
            all_x = np.array([0.0])
            all_y = np.array([0.0])
        self.global_minx = float(all_x.min())
        self.global_maxx = float(all_x.max())
        self.global_miny = float(all_y.min())
        self.global_maxy = float(all_y.max())

        # Matplotlib setup
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.subplots_adjust(bottom=0.25)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title("Project 3 Viewer (t = 0.0)")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")

        # LineCollections:
        self.base_lc = LineCollection([], linewidths=0.7)
        self.focus_lc = LineCollection([], linewidths=1.8)
        self.hl_lc = LineCollection([], linewidths=1.4)
        self.ax.add_collection(self.base_lc)
        self.ax.add_collection(self.focus_lc)
        self.ax.add_collection(self.hl_lc)

        # Point scatters
        self.base_points = self.ax.scatter([], [], s=10, zorder=3)
        self.focus_points = self.ax.scatter([], [], s=30, marker="x", zorder=4)
        self.hl_points = self.ax.scatter([], [], s=25, marker="s", zorder=4)

        # Last view limits for change detection
        self._last_xlim = None
        self._last_ylim = None

        # Init view
        self._init_view()

        # Sliders for t0, t1
        slider_ax_t0 = self.fig.add_axes([0.15, 0.12, 0.7, 0.03])
        slider_ax_t1 = self.fig.add_axes([0.15, 0.08, 0.7, 0.03])
        self.slider_t0 = Slider(
            ax=slider_ax_t0,
            label="t0",
            valmin=0.0,
            valmax=tmax_slider,
            valinit=self.t0,
        )
        self.slider_t1 = Slider(
            ax=slider_ax_t1,
            label="t1",
            valmin=0.0,
            valmax=tmax_slider,
            valinit=self.t1,
        )
        self.slider_t0.on_changed(self._on_slider_t0)
        self.slider_t1.on_changed(self._on_slider_t1)

        # Play/Pause button
        play_ax = self.fig.add_axes([0.88, 0.12, 0.08, 0.08])
        self.play_button = Button(play_ax, "Play", hovercolor="0.85")
        self.play_button.on_clicked(self._on_play_button)

        # Event handlers
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        # Timer for animation (~30 fps)
        self.timer = self.fig.canvas.new_timer(interval=33)
        self.timer.add_callback(self._on_timer)
        self.timer.start()

    # ---------- initialization & view ----------

    def _init_view(self):
        if not self.objects:
            self.ax.set_xlim(-1, 1)
            self.ax.set_ylim(-1, 1)
        else:
            pad_x = 0.05 * (self.global_maxx - self.global_minx + 1e-9)
            pad_y = 0.05 * (self.global_maxy - self.global_miny + 1e-9)
            minx = self.global_minx - pad_x
            maxx = self.global_maxx + pad_x
            miny = self.global_miny - pad_y
            maxy = self.global_maxy + pad_y
            self.ax.set_xlim(minx, maxx)
            self.ax.set_ylim(miny, maxy)

        self._home_xlim = self.ax.get_xlim()
        self._home_ylim = self.ax.get_ylim()
        self._update_visible()

    # ---------- drawing & culling ----------

    def _on_draw(self, event):
        self._update_visible()

    def _update_visible(self):
        """
        Update geometry for current time and view window with culling.
        Also optionally re-center the view to follow focus object.
        """
        if self.follow_focus and self.focus_id is not None and self.focus_id < len(self.objects):
            self._recenter_on_focus()

        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        self._last_xlim = (x0, x1)
        self._last_ylim = (y0, y1)

        vx0 = x0 - self.cull_margin
        vx1 = x1 + self.cull_margin
        vy0 = y0 - self.cull_margin
        vy1 = y1 + self.cull_margin

        base_segs = []
        focus_segs = []
        hl_segs = []
        base_pts = []
        focus_pts = []
        hl_pts = []

        t = self.current_t

        for obj_idx, obj in enumerate(self.objects):
            dx, dy = displacement_at_time(obj, t)

            minx0, maxx0, miny0, maxy0 = obj.bbox0
            minx = minx0 + dx
            maxx = maxx0 + dx
            miny = miny0 + dy
            maxy = maxy0 + dy

            # BBox cull
            if maxx < vx0 or minx > vx1 or maxy < vy0 or miny > vy1:
                continue

            poly = obj.poly0 + np.array([dx, dy])
            k = poly.shape[0]

            is_focus = (self.focus_id is not None and obj_idx == self.focus_id)
            is_highlight = (self.highlight_id is not None and obj_idx == self.highlight_id)

            if k == 1:
                pt = poly[0]
                if is_focus:
                    focus_pts.append(pt)
                elif is_highlight:
                    hl_pts.append(pt)
                else:
                    base_pts.append(pt)
            elif k >= 2:
                seg_list = []
                for i in range(k):
                    p1 = poly[i]
                    p2 = poly[(i + 1) % k]
                    seg_list.append([p1, p2])
                if is_focus:
                    focus_segs.extend(seg_list)
                elif is_highlight:
                    hl_segs.extend(seg_list)
                else:
                    base_segs.extend(seg_list)

        def to_seg_array(seglist):
            if seglist:
                return np.array(seglist, dtype=float)
            else:
                return np.empty((0, 2, 2), dtype=float)

        def to_pts_array(ptslist):
            if ptslist:
                return np.array(ptslist, dtype=float)
            else:
                return np.empty((0, 2), dtype=float)

        self.base_lc.set_segments(to_seg_array(base_segs))
        self.focus_lc.set_segments(to_seg_array(focus_segs))
        self.hl_lc.set_segments(to_seg_array(hl_segs))

        self.base_points.set_offsets(to_pts_array(base_pts))
        self.focus_points.set_offsets(to_pts_array(focus_pts))
        self.hl_points.set_offsets(to_pts_array(hl_pts))

        self.ax.set_title(f"Project 3 Viewer (t = {self.current_t:.6f}, dt = {self.dt:.6f})")
        self.fig.canvas.draw_idle()

    def _recenter_on_focus(self):
        """
        Recenter the camera on the focus object's bounding box center
        at the current time, keeping the same width/height of view.
        """
        if self.focus_id is None or self.focus_id >= len(self.objects):
            return
        obj = self.objects[self.focus_id]
        dx, dy = displacement_at_time(obj, self.current_t)
        minx0, maxx0, miny0, maxy0 = obj.bbox0
        cx = 0.5 * (minx0 + maxx0) + dx
        cy = 0.5 * (miny0 + maxy0) + dy

        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        width = x1 - x0
        height = y1 - y0

        new_x0 = cx - width / 2.0
        new_x1 = cx + width / 2.0
        new_y0 = cy - height / 2.0
        new_y1 = cy + height / 2.0

        self.ax.set_xlim(new_x0, new_x1)
        self.ax.set_ylim(new_y0, new_y1)

    # ---------- sliders & animation ----------

    def _on_slider_t0(self, val):
        self.t0 = float(val)
        if self.t0 > self.t1:
            self.t1 = self.t0
            self.slider_t1.set_val(self.t1)
        if self.current_t < self.t0 or self.current_t > self.t1:
            self.current_t = self.t0
        self._update_visible()

    def _on_slider_t1(self, val):
        self.t1 = float(val)
        if self.t1 < self.t0:
            self.t0 = self.t1
            self.slider_t0.set_val(self.t0)
        if self.current_t > self.t1:
            self.current_t = self.t1
        self._update_visible()

    def _on_play_button(self, event):
        self.playing = not self.playing
        self.play_button.label.set_text("Pause" if self.playing else "Play")
        self.fig.canvas.draw_idle()

    def _on_timer(self):
        if not self.playing:
            return
        if self.t1 <= self.t0:
            return
        self.current_t += self.dt
        if self.current_t > self.t1:
            self.current_t = self.t1
            self.playing = False
            self.play_button.label.set_text("Play")
        self._update_visible()

    # ---------- keyboard interaction ----------

    def _on_key(self, event):
        key = event.key
        if key in ["+", "="]:
            self._zoom(1 / 1.2)
        elif key in ["-", "_"]:
            self._zoom(1.2)
        elif key == "left":
            self._pan(dx_frac=-0.1, dy_frac=0.0)
        elif key == "right":
            self._pan(dx_frac=0.1, dy_frac=0.0)
        elif key == "up":
            self._pan(dx_frac=0.0, dy_frac=0.1)
        elif key == "down":
            self._pan(dx_frac=0.0, dy_frac=-0.1)
        elif key == "0":
            self.ax.set_xlim(*self._home_xlim)
            self.ax.set_ylim(*self._home_ylim)
            self._update_visible()
        elif key in ["p", "r"]:
            self.playing = not self.playing
            self.play_button.label.set_text("Pause" if self.playing else "Play")
            self.fig.canvas.draw_idle()
        elif key == "[":
            # decrease dt by a fixed amount
            self.dt = max(self.dt_min, self.dt - 0.001)
            self._update_visible()
        elif key == "]":
            # increase dt by a fixed amount
            self.dt = min(self.dt_max, self.dt + 0.001)
            self._update_visible()
        elif key == "t":
            # reset current time to t0
            self.current_t = self.t0
            self._update_visible()
        elif key == "f":
            # toggle following focus object
            if self.focus_id is not None:
                self.follow_focus = not self.follow_focus
                mode = "ON" if self.follow_focus else "OFF"
                print(f"Follow focus: {mode}")
                self._update_visible()
            else:
                print("No focus-id set; cannot follow.")
        elif key == "q":
            plt.close(self.fig)

    # ---------- pan & zoom helpers ----------

    def _zoom(self, scale, center=None):
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        if center is None:
            cx = 0.5 * (x0 + x1)
            cy = 0.5 * (y0 + y1)
        else:
            cx, cy = center

        width = (x1 - x0) * scale
        height = (y1 - y0) * scale
        new_x0 = cx - width / 2.0
        new_x1 = cx + width / 2.0
        new_y0 = cy - height / 2.0
        new_y1 = cy + height / 2.0

        self.ax.set_xlim(new_x0, new_x1)
        self.ax.set_ylim(new_y0, new_y1)
        self._update_visible()

    def _pan(self, dx_frac=0.0, dy_frac=0.0):
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        width = x1 - x0
        height = y1 - y0

        dx = dx_frac * width
        dy = dy_frac * height

        self.ax.set_xlim(x0 + dx, x1 + dx)
        self.ax.set_ylim(y0 + dy, y1 + dy)
        self._update_visible()


# ------------------------- main -------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description="Project 3: 2D collision visualizer with time controls.")
    parser.add_argument("input", help="Path to Project 3 input file")
    parser.add_argument("--t0", type=float, default=0.0,
                        help="Initial start time t0 (default 0.0)")
    parser.add_argument("--t1", type=float, default=None,
                        help="Initial end time t1 (default = max motion time)")
    parser.add_argument("--dt", type=float, default=0.1,
                        help="Initial time step for animation (default 0.1; can be very small)")
    parser.add_argument("--cull-margin", type=float, default=50.0,
                        help="Margin around view for culling (world units, default 50)")
    parser.add_argument("--focus-id", type=int, default=None,
                        help="Object ID to focus and optionally follow")
    parser.add_argument("--highlight-id", type=int, default=None,
                        help="Second object ID to highlight")
    args = parser.parse_args(argv)

    objects, tmax = parse_input(args.input)
    viewer = CollisionViewer(
        objects,
        tmax,
        t0_init=args.t0,
        t1_init=args.t1,
        dt=args.dt,
        cull_margin=args.cull_margin,
        focus_id=args.focus_id,
        highlight_id=args.highlight_id
    )
    plt.show()


if __name__ == "__main__":
    main()

""" Prompts used to create. I could have improved more and did not test it thoroughly for efficiency. So a newer version might be needed. But from a quick visualization perspective this worked nicely.

Prompt 1:
Use the previously attached ProjectCollisionDetection.tex assignment for reference.

I want to create a visualizing tool (in Python) for the students. It should load in the input. Display the various objects on the screen. And support a few features (more to be added as we explore this tool). I want it to have a Zoom in and out feature as well as a pan feature so students can explore all or part of the scene at any time. It should also have a time sliding tool that allows users to select a start t0 and end time t1 (default would be 0 to max t in the input). And then if the user presses a play button (or hits a specific key R for run or P for play), it should animate the scene from t0 to t1 showing each polygons position along the way. There should be a dt (delta time) to identify the granularity of the time steps. If t0=t1 it would just draw the polygons in that current position.

For efficiency, the program should only print items that are in the display area. If you rely on clipping to happen naturally it might be far too slow for large inputs. I see that as using the bounding box of each object and if the bounding box intersects the viewing window then draw the polygon.

Suggestions in response:
If you'd like, we can next add:
  * A mode to highlight one particular object ID (e.g., to cross-check with a student's reported collision pair).
  * A simple “collision overlay” mode where you feed it a pair (i, j, T) and it draws them in a different color at time T to visually inspect the collision.

Prompt 2:
I like those two features. Can you also add a feature to reset the time back to t0? I like the idea of highlighting TWO objects - the two in question and perhaps following the first object as it moves... so if it goes off the initial window the window follows that object. Maybe if object 1 is identified then the visualization is centered on it for the animation and any other object listed is highlighted to show a potential collision. The collision overlay can be shown by highlighting object 1 and 2 and setting T0 and T1 to the time. So collision overlay seems like it would be already accomplished with this system.

Prompt 3:
I like the command line interface. Can you also add a way to also include t0, t1, and dt in there. And support a dt of up to 0.000001 (or even smaller) place - though I don't want [] to move by that little, moving by 0.001 is fine as long as we can set the dt to a lower granularity.
"""