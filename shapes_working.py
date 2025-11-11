from dataclasses import dataclass
from typing import List, Iterable, Tuple, Protocol, runtime_checkable
import math
import sys

EPS = 1e-8  # robust tolerance for predicates
DEBUG = False
class Shape:
    def __init__(self, pointCoors, pathTriples):
        self.points = []
        self.pathing = []
        self.radius = 0
        
        for i in range(len(pointCoors)//2):
            self.points.append((pointCoors[2*i], pointCoors[(2*i) + 1]))

        for j in range(len(pathTriples)/3):
            self.pathing.append((pathTriples[3*j], pathTriples[3*j + 1], pathTriples[3*j + 2]))

        if len(self.points) == 1:
            self.radius = 0
            self.center = self.points[0]
        elif len(self.points) == 2:
            self.radius = math.sqrt(math.pow(math.abs(self.points[0][0] - self.points[1][0]), 2) + math.pow(math.abs(self.points[0][1] - self.points[1][1]), 2))/2
            self.center = Point((self.points[0][0] + self.points[1][0])/2, (self.points[0][1] + self.points[1][1])/2)

        self.currentPathIndex = 0
        self.currentTime = 0
        


    def getRadius(self, startPoint, currentLargest):
        maxDist = currentLargest
        maxIndex = -1
        for i in range(len(self.points)):
            distance = math.sqrt(math.pow(math.abs(startPoint.x - self.points[i][0]), 2) + math.pow(math.abs(startPoint.y - self.points[i][1]), 2))
            if distance >= maxDist:
                maxDist = distance
                maxIndex = i

        if maxDist == currentLargest:
            self.radius = maxDist/2
            return Point((startPoint.x + self.points[maxIndex].x)/2, (startPoint.y + self.points[maxIndex].y)/2)
        else:
            return self.getRadius(self.points[maxIndex], maxDist)
            
            

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def isBetweenX(self, point1, point2):
        if self.x >= point1.x and self.x <= point2.x:
            return True
        else:
            return False
        
        
    def isBetweenY(self, point1, point2):
        if self.y >= min(point1.y, point2.y) and self.y <= max(point1.y, point2.y):
            return True
        else:
            return False
        
    
        

class Edge:
    #Always assume point1.x <= point2.x
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2

        run = point2.x - point1.x
        rise = point2.y - point1.y
        
        if(run == 0 and rise != 0):
            self.slope = math.inf
            self.yInt = None
        else:
            self.slope = rise/run
            #y intercept used for finding if a point is on the line
            self.yInt = point1.y - (point1.x * self.slope)
        
        
    #HAS EDGE CASES THAT DON'T WORK, USE y = mx + b
    def isIntersecting(self, candEdge):
        if (self.slope == candEdge.slope and self.yInt != None):
            if self.yInt == candEdge.yInt:
                if (self.point1.isBetweenX(candEdge.point1, candEdge.point2) or self.point2.isBetweenX(candEdge.point1, candEdge.point2) or candEdge.point1.isBetweenX(self.point1, self.point2) or candEdge.point2.isBetweenX(self.point1, self.point2)):
                    return True #Same line, overlaps, intersects
                else: 
                    return False #Same line, no overlap, does not intersect
            else:
                return False #Parallel, not identical, never intersects
        else:
            if(self.yInt != None and candEdge.yInt != None):
                #When m1x + b1 = m2x + b2, x = (b2 - b1)/(m1-m2)
                intersectX = (candEdge.yInt - self.yInt)/(self.slope-candEdge.slope)
                if self.inBoundsX(intersectX) and candEdge.inBoundsX(intersectX):
                    return True
                else: 
                    return False
                
            #In these cases, there is only one possible x value a vertical line can intersect.
            else:
                if(self.yInt == None):
                    intersectX = self.point1.x
                else:
                    intersectX = candEdge.point1.x
            
                if self.inBoundsX(intersectX) and candEdge.inBoundsX(intersectX):
                    if self.point1.isBetweenY(candEdge.point1, candEdge.point2) or self.point2.isBetweenY(candEdge.point1, candEdge.point2) or candEdge.point1.isBetweenY(self.point1, self.point2) or candEdge.point2.isBetweenY(self.point1, self.point2):
                        return True
                    return False
                else:
                    return False

                
        
    def isOnLine(self, candPoint):
        if((self.slope * candPoint.x) + self.yInt == candPoint.y):
            return True
        else:
            return False
        
    def inBoundsX(self, someX):
        if someX >= self.point1.x and someX <= self.point2.x:
            return True
        else:
            return False

@runtime_checkable
class _HasXY(Protocol):
    x: float
    y: float
    
def _xy(p) -> Tuple[float, float]:
    """Return (x,y) for either tuple-like or objects with .x/.y."""
    if isinstance(p, _HasXY):
        return float(p.x), float(p.y)
    # tuple/list like
    return float(p[0]), float(p[1])

class AABB2D:
    __slots__ = ("minx", "miny", "maxx", "maxy")

    def __init__(self, minx=math.inf, miny=math.inf, maxx=-math.inf, maxy=-math.inf):
        self.minx, self.miny, self.maxx, self.maxy = minx, miny, maxx, maxy

    def expand_pt(self, p) -> None:
        x, y = _xy(p)
        if x < self.minx: self.minx = x
        if y < self.miny: self.miny = y
        if x > self.maxx: self.maxx = x
        if y > self.maxy: self.maxy = y

    def merge(self, other: "AABB2D") -> "AABB2D":
        out = AABB2D()
        out.minx = min(self.minx, other.minx)
        out.miny = min(self.miny, other.miny)
        out.maxx = max(self.maxx, other.maxx)
        out.maxy = max(self.maxy, other.maxy)
        return out

    def overlaps(self, other: "AABB2D") -> bool:
        # Closed boxes; touching counts
        return not (
            self.maxx < other.minx - EPS or
            self.minx > other.maxx + EPS or
            self.maxy < other.miny - EPS or
            self.miny > other.maxy + EPS
        )

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.minx, self.miny, self.maxx, self.maxy)
    
    
def aabb_of_points_generic(pts: Iterable) -> AABB2D:
    box = AABB2D()
    for p in pts:
        box.expand_pt(p)
    return box

def aabb_of_polygon_generic(verts: List) -> AABB2D:
    return aabb_of_points_generic(verts)

# --- Lightweight vector helpers (no class, just functions) ---

def _v_add(p, q):
    x1, y1 = _xy(p); x2, y2 = _xy(q)
    return (x1 + x2, y1 + y2)

def _v_sub(p, q):
    x1, y1 = _xy(p); x2, y2 = _xy(q)
    return (x1 - x2, y1 - y2)

def _v_smul(p, s: float):
    x, y = _xy(p)
    return (x * s, y * s)

def _dist2(a, b):
    ax, ay = _xy(a); bx, by = _xy(b)
    dx, dy = ax - bx, ay - by
    return dx*dx + dy*dy

def _point_on_segment(p, a, b):
    # Collinear and within bounding box
    if abs(_orient(a, b, p)) > EPS: 
        return False
    px, py = _xy(p); ax, ay = _xy(a); bx, by = _xy(b)
    return (min(ax, bx) - EPS <= px <= max(ax, bx) + EPS and
            min(ay, by) - EPS <= py <= max(ay, by) + EPS)

def _point_in_or_on_poly(p, poly):
    # On any edge?
    n = len(poly)
    for i in range(n):
        if _point_on_segment(p, poly[i], poly[(i+1) % n]):
            return True
    # Strict inside test
    return _point_in_poly(p, poly)

# --- Path & Motion storage ---

@dataclass(frozen=True)
class PathSegment:
    vx: float
    vy: float
    dt: float

    def v_tuple(self) -> Tuple[float, float]:
        return (self.vx, self.vy)

class MotionPath:
    """
    Piecewise-constant velocity path.
    - segs: list[PathSegment]
    - prefix_t[i]: start time of seg i
    """
    __slots__ = ("segs", "prefix_t", "_t_end")

    def __init__(self, segs: List[PathSegment]):
        clean = [s for s in segs if s.dt > 0]
        self.segs: List[PathSegment] = clean
        # prefix times
        self.prefix_t: List[float] = []
        acc = 0.0
        for _ in clean:
            self.prefix_t.append(acc)
            acc += _.dt
        self._t_end = acc

    @staticmethod
    def from_flat(triples: List[float]) -> "MotionPath":
        if len(triples) % 3 != 0:
            raise ValueError("Path line must be vx vy dt triples.")
        segs = []
        for i in range(0, len(triples), 3):
            vx, vy, dt = float(triples[i]), float(triples[i+1]), float(triples[i+2])
            if dt < 0:
                raise ValueError("dt must be non-negative")
            if dt == 0:
                continue
            segs.append(PathSegment(vx, vy, dt))
        return MotionPath(segs)

    @property
    def t_end(self) -> float:
        return self._t_end

    def displacement_up_to(self, t: float) -> Tuple[float, float]:
        """Total translation from t=0 to time t (clamped to end)."""
        if not self.segs or t <= 0:
            return (0.0, 0.0)
        t_rem = min(max(t, 0.0), self._t_end)
        dx, dy = 0.0, 0.0
        for s in self.segs:
            if t_rem <= 0:
                break
            use = min(s.dt, t_rem)
            dx += s.vx * use
            dy += s.vy * use
            t_rem -= use
        return (dx, dy)
    
# --- Moving polygon wrapper ---

class MovingShape:
    """
    Wraps a static polygon (list of vertices) + MotionPath.
    Assumes your polygon is a list of vertices, where each vertex is either:
      - an object with .x/.y, or
      - a 2-tuple/list (x, y).
    No rotation/scaling; pure translation.
    """

    __slots__ = ("poly0", "path")

    def __init__(self, polygon_vertices: List, motion_path: MotionPath):
        # store original vertices as tuples to avoid mutating your partner's classes
        self.poly0 = [(_xy(v)[0], _xy(v)[1]) for v in polygon_vertices]
        self.path = motion_path

    @property
    def t_end(self) -> float:
        return self.path.t_end

    def polygon_at(self, t: float) -> List[Tuple[float, float]]:
        """Vertices translated to time t."""
        dx, dy = self.path.displacement_up_to(t)
        if abs(dx) < EPS and abs(dy) < EPS:
            return list(self.poly0)
        return [(x + dx, y + dy) for (x, y) in self.poly0]

    def aabb_at(self, t: float) -> AABB2D:
        """Tight AABB of the polygon at time t."""
        return aabb_of_polygon_generic(self.polygon_at(t))

    def swept_aabb(self, t0: float, t1: float) -> AABB2D:
        if t1 < t0:
            t0, t1 = t1, t0
        t0 = max(0.0, min(t0, self.t_end))
        t1 = max(0.0, min(t1, self.t_end))
        if t1 - t0 <= EPS:
            return self.aabb_at(t0)

        times = [t0]
        acc = 0.0
        for s in self.path.segs:
            t_start = acc
            t_end = acc + s.dt
            acc = t_end
            if t0 + EPS < t_start < t1 - EPS: times.append(t_start)
            if t0 + EPS < t_end   < t1 - EPS: times.append(t_end)
        times.append(t1)

        box = AABB2D()
        for tt in times:
            for v in self.polygon_at(tt):
                box.expand_pt(v)
        return box

def swept_aabb_for_segment_generic(polygon_vertices: List,
                                   start_offset: Tuple[float, float],
                                   vx: float, vy: float, dt: float) -> AABB2D:
    """
    Tight swept AABB for a polygon translated from 'start_offset' to
    'start_offset + (vx,vy)*dt'. Endpoint AABBs merged gives tight box for translation.
    """
    sx, sy = start_offset
    if dt <= EPS:
        # No motion: just the translated polygon AABB.
        a = AABB2D()
        for v in polygon_vertices:
            x, y = _xy(v)
            a.expand_pt((x + sx, y + sy))
        return a

    dx, dy = vx * dt, vy * dt
    a = AABB2D()
    for v in polygon_vertices:
        x, y = _xy(v)
        a.expand_pt((x + sx,     y + sy))
        a.expand_pt((x + sx+dx,  y + sy+dy))
    return a

def parse_polygon_line_generic(line: str) -> List[Tuple[float, float]]:
    parts = [float(x) for x in line.strip().split()]
    if len(parts) < 2 or len(parts) % 2 != 0:
        raise ValueError("Polygon line must be x1 y1 x2 y2 ...")
    verts = [(parts[i], parts[i+1]) for i in range(0, len(parts), 2)]
    return verts

def parse_path_line_to_motionpath(line: str) -> MotionPath:
    parts = [float(x) for x in line.strip().split()]
    return MotionPath.from_flat(parts)

def _read_nonempty_line(f):
    """Read the next non-empty, non-whitespace line (or return None)."""
    while True:
        line = f.readline()
        if not line:
            return None
        line = line.strip()
        if line:
            return line

def load_project_input(file_path: str):
    """
    Reads input in the project format:
      N
      <polygon line>
      <path line>
      (repeated N times)
    Returns a list[MovingShape].
    """
    objs = []
    with open(file_path, "r", encoding="utf-8") as f:
        # first line: number of objects
        first = _read_nonempty_line(f)
        if first is None:
            raise ValueError("Empty input file.")
        try:
            n = int(first)
        except ValueError:
            raise ValueError(f"First line must be an integer count, got: {first!r}")

        for i in range(n):
            poly_line = _read_nonempty_line(f)
            if poly_line is None:
                raise ValueError(f"Missing polygon line for object {i}.")
            path_line = _read_nonempty_line(f)
            if path_line is None:
                raise ValueError(f"Missing path line for object {i}.")

            # Parse using the helpers you added earlier
            verts = parse_polygon_line_generic(poly_line)
            mp = parse_path_line_to_motionpath(path_line)
            objs.append(MovingShape(verts, mp))
    return objs

def _summarize_objects(mobjs, sample_times=(0.0, 1.0, 5.0)):
    """Print quick summaries to verify parsing + motion math."""
    print(f"Loaded {len(mobjs)} objects.")
    show = min(5, len(mobjs))
    for i in range(show):
        ms = mobjs[i]
        print(f"\nObject {i}: duration t_end={ms.t_end:.6f}")
        # AABB at a few times (clamped to t_end)
        for t in sample_times:
            tt = min(t, ms.t_end)
            box = ms.aabb_at(tt).as_tuple()
            print(f"  AABB@t={tt:.3f} -> {box}")
        # Swept AABB over entire motion
        whole = ms.swept_aabb(0.0, ms.t_end).as_tuple()
        print(f"  Swept AABB [0, t_end]: {whole}")
        
# COLLISION PIPELINE

def _orient(a, b, c):
    ax, ay = _xy(a); bx, by = _xy(b); cx, cy = _xy(c)
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

def _on_segment(a, b, p):
    # collinear and within bbox
    if abs(_orient(a, b, p)) > EPS:
        return False
    ax, ay = _xy(a); bx, by = _xy(b); px, py = _xy(p)
    return (min(ax, bx) - EPS <= px <= max(ax, bx) + EPS and
            min(ay, by) - EPS <= py <= max(ay, by) + EPS)

def _segments_intersect(p1, p2, q1, q2):
    o1 = _orient(p1, p2, q1)
    o2 = _orient(p1, p2, q2)
    o3 = _orient(q1, q2, p1)
    o4 = _orient(q1, q2, p2)

    # proper crossing
    if (o1 * o2 < -EPS) and (o3 * o4 < -EPS):
        return True

    # endpoint/collinear touches
    if abs(o1) <= EPS and _on_segment(p1, p2, q1): return True
    if abs(o2) <= EPS and _on_segment(p1, p2, q2): return True
    if abs(o3) <= EPS and _on_segment(q1, q2, p1): return True
    if abs(o4) <= EPS and _on_segment(q1, q2, p2): return True
    return False

def _point_in_poly(pt, poly):
    px, py = _xy(pt)
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = _xy(poly[i])
        x2, y2 = _xy(poly[(i + 1) % n])
        # check edge straddles the horizontal ray
        cond = ((y1 > py) != (y2 > py))
        if cond:
            # x coordinate of intersection with the ray
            xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1 + 0.0)
            if xint > px + EPS:
                inside = not inside
    return inside

def _polygons_intersect_at_time(msA: MovingShape, msB: MovingShape, t: float) -> bool:
    A = msA.polygon_at(t)
    B = msB.polygon_at(t)

    # Quick reject: AABB
    if not aabb_of_polygon_generic(A).overlaps(aabb_of_polygon_generic(B)):
        return False

    nA, nB = len(A), len(B)

    # === Handle degenerate cases ===
    # Point ↔ Point
    if nA == 1 and nB == 1:
        return _dist2(A[0], B[0]) <= (EPS * EPS)

    # Point ↔ Segment
    if nA == 1 and nB == 2:
        return _point_on_segment(A[0], B[0], B[1])
    if nA == 2 and nB == 1:
        return _point_on_segment(B[0], A[0], A[1])

    # Segment ↔ Segment
    if nA == 2 and nB == 2:
        return _segments_intersect(A[0], A[1], B[0], B[1])

    # Point ↔ Polygon
    if nA == 1 and nB >= 3:
        return _point_in_or_on_poly(A[0], B)
    if nB == 1 and nA >= 3:
        return _point_in_or_on_poly(B[0], A)

    # Segment ↔ Polygon
    if nA == 2 and nB >= 3:
        # edge-edge
        for j in range(nB):
            if _segments_intersect(A[0], A[1], B[j], B[(j+1) % nB]):
                return True
        # segment fully inside polygon?
        return _point_in_poly(A[0], B) and _point_in_poly(A[1], B)
    if nB == 2 and nA >= 3:
        for i in range(nA):
            if _segments_intersect(B[0], B[1], A[i], A[(i+1) % nA]):
                return True
        return _point_in_poly(B[0], A) and _point_in_poly(B[1], A)

    # === Polygon ↔ Polygon (both n ≥ 3) ===
    # Edge-edge
    for i in range(nA):
        a1, a2 = A[i], A[(i + 1) % nA]
        for j in range(nB):
            b1, b2 = B[j], B[(j + 1) % nB]
            if _segments_intersect(a1, a2, b1, b2):
                return True
    # Containment (one inside the other)
    if _point_in_or_on_poly(A[0], B): return True
    if _point_in_or_on_poly(B[0], A): return True

    return False

def _segment_boundaries(path: MotionPath):
    """Return absolute start times of each segment and the final end time."""
    times = [0.0]
    acc = 0.0
    for s in path.segs:
        acc += s.dt
        times.append(acc)
    # times: [0, t1, t2, ..., t_end]
    # if empty path -> [0.0]
    return times


# --- Continuous-time helpers (analytic swept AABB) ---
def _current_velocity(path: MotionPath, t: float) -> Tuple[float, float]:
    """Return (vx, vy) of the active segment at time t (clamped)."""
    if not path.segs:
        return (0.0, 0.0)
    t = max(0.0, min(t, path.t_end))
    acc = 0.0
    for s in path.segs:
        if t < acc + s.dt or abs(t - (acc + s.dt)) <= EPS:
            return (s.vx, s.vy)
        acc += s.dt
    last = path.segs[-1]
    return (last.vx, last.vy)

def _axis_overlap_times(a1, b1, a2, b2, rv):
    if abs(rv) <= EPS:
        if b1 < a2 - EPS or b2 < a1 - EPS:
            return None
        return (-math.inf, math.inf)
    if rv > 0.0:
        return ((a2 - b1) / rv, (b2 - a1) / rv)
    else:
        return ((b2 - a1) / rv, (a2 - b1) / rv)

def _swept_aabb_hit(t0, t1, msA: MovingShape, msB: MovingShape):
    aA = msA.aabb_at(t0)
    aB = msB.aabb_at(t0)
    ax0, ay0, ax1, ay1 = aA.minx, aA.miny, aA.maxx, aA.maxy
    bx0, by0, bx1, by1 = aB.minx, aB.miny, aB.maxx, aB.maxy
    vAx, vAy = _current_velocity(msA.path, t0)
    vBx, vBy = _current_velocity(msB.path, t0)
    rvx, rvy = (vAx - vBx), (vAy - vBy)
    tx = _axis_overlap_times(ax0, ax1, bx0, bx1, rvx)
    if tx is None:
        return None
    ty = _axis_overlap_times(ay0, ay1, by0, by1, rvy)
    if ty is None:
        return None
    t_enter = max(tx[0], ty[0])
    t_exit  = min(tx[1], ty[1])
    if t_enter - t_exit > EPS:
        return None
    span = t1 - t0
    if t_exit < -EPS:
        return None
    local_hit = max(0.0, t_enter)
    cand = t0 + local_hit
    if cand - t1 <= EPS:
        return cand
    return None
def _union_boundaries(msA: MovingShape, msB: MovingShape):
    """Sorted unique times from both paths (segment starts/ends)."""
    ta = _segment_boundaries(msA.path)
    tb = _segment_boundaries(msB.path)
    all_t = sorted(set(ta + tb))
    # also consider final max(t_end_A, t_end_B), because after motion objects stop
    tmax = max((ta[-1] if ta else 0.0), (tb[-1] if tb else 0.0))
    if not all_t or abs(all_t[-1] - tmax) > EPS:
        all_t.append(tmax)
    return all_t


def find_first_contact_pair(msA: MovingShape, msB: MovingShape, tol=1e-6):
    """
    Robust earliest-contact finder:
      1) Split the timeline into intervals where both objects have constant velocity.
      2) For each [t0,t1], compute analytic swept-AABB earliest hit (if any).
      3) If a hit exists, refine with a monotone search (binary) using the exact polygon collision.
    Returns earliest absolute time or None.
    """
    times = _union_boundaries(msA, msB)
    if not times:
        return None
    best = None

    def collide(t: float) -> bool:
        return _polygons_intersect_at_time(msA, msB, t)

    def refine(L, R):
        lo, hi = L, R
        for _ in range(60):
            mid = (lo + hi) / 2.0
            if collide(mid):
                hi = mid
            else:
                lo = mid
        return hi

    for k in range(len(times) - 1):
        t0, t1 = times[k], times[k+1]
        if best is not None and t0 - best > EPS:
            break
        if collide(t0):
            best = t0 if best is None else min(best, t0)
            continue
        thit = _swept_aabb_hit(t0, t1, msA, msB)
        if thit is None:
            continue
        L = max(t0, thit - 1e-7)
        R = min(t1, thit + 1e-3)
        steps = [0.0, 1e-9, 1e-8, 1e-7, 1e-6, 5e-6, 1e-5, 1e-4, (R-L)*0.25, (R-L)*0.5, (R-L)*0.75, (R-L)*0.95]
        bracket = None
        for s in steps:
            tt = L + min(s, R-L)
            if collide(tt):
                bracket = tt
                break
        if bracket is None:
            L2 = max(t0, thit - 1e-5)
            R2 = min(t1, thit + 1e-2)
            for s in steps:
                tt = L2 + min(s, R2-L2)
                if collide(tt):
                    bracket = tt
                    break
        if bracket is not None:
            tfirst = refine(t0, bracket)
            if best is None or tfirst < best:
                best = tfirst

    return best


    maxT = max(msA.t_end, msB.t_end)

    def _clip(t):  # clamp to [0, maxT]
        return 0.0 if t < 0.0 else (maxT if t > maxT else t)

    def collide(t):
        return _polygons_intersect_at_time(msA, msB, _clip(t))

    # number of interior probes per interval (tune as needed)
    PROBES = 16

    best = None

    for i in range(len(times) - 1):
        t0 = times[i]
        t1 = times[i + 1]
        if t1 - t0 <= 1e-12:
            if collide(t0):
                best = t0 if best is None else min(best, t0)
            continue

        # If already colliding at the left endpoint, that's the earliest for this interval.
        if collide(t0):
            if best is None or t0 < best:
                best = t0
            continue

        # Probe interior points left-to-right; remember the last non-colliding time.
        last_free_t = t0
        hit = None
        step = (t1 - t0) / (PROBES + 1)
        tt = t0
        for k in range(1, PROBES + 1):
            tt = t0 + k * step
            if collide(tt):
                hit = tt
                break
            else:
                last_free_t = tt

        # If no interior hit, also check right endpoint once (just in case)
        if hit is None and collide(t1):
            hit = t1
            # last_free_t is already the last false (maybe a probe, maybe t0)

        # If we got a hit, bisect [last_free_t, hit] to first contact
        if hit is not None:
            lo, hi = last_free_t, hit
            for _ in range(64):
                mid = 0.5 * (lo + hi)
                if collide(mid):
                    hi = mid
                else:
                    lo = mid
                if hi - lo <= tol:
                    break
            cand = max(0.0, hi)  # sanitize tiny negatives
            if best is None or cand < best:
                best = cand

        # Early exit: if we already found a contact at/before t0, nothing earlier exists
        # (Not strictly necessary; best is monotone nonincreasing in the scan order.)
        if best is not None and best <= t0 + tol:
            break

    return best
def build_candidates_sweep(ms_list: list[MovingShape]):
    """
    Build candidate pairs (i, j) with overlapping global swept AABBs over [0, maxT].
    This is a coarse filter; narrow-phase will confirm.
    """
    if not ms_list:
        return []

    maxT = max(ms.t_end for ms in ms_list) if ms_list else 0.0
    boxes = []
    for i, ms in enumerate(ms_list):
        box = ms.swept_aabb(0.0, maxT)  # includes sampling at boundaries if you patched it
        boxes.append((i, box))

    # sweep on x
    items = sorted(boxes, key=lambda x: x[1].minx)
    active = []
    candidates = []

    for i_idx, ibox in items:
        # pop from active anything whose maxx < current minx
        new_active = []
        for j_idx, jbox in active:
            if jbox.maxx + EPS >= ibox.minx:
                new_active.append((j_idx, jbox))
                # potential x-overlap; check y before adding
                if not (jbox.maxy < ibox.miny - EPS or jbox.miny > ibox.maxy + EPS):
                    a, b = (j_idx, i_idx) if j_idx < i_idx else (i_idx, j_idx)
                    if a != b:
                        candidates.append((a, b))
        active = new_active
        active.append((i_idx, ibox))

    # dedupe
    candidates = sorted(set(candidates))
    return candidates

# Full solve: earliest collision over all objects ----

def earliest_collision(ms_list: list[MovingShape], tol=1e-6):
    """
    Returns (T, i, j) for earliest collision, or None if no collision occurs.
    """
    best_T = None
    best_pair = None

    # Broad-phase candidate generation
    cand_pairs = build_candidates_sweep(ms_list)

    # Narrow-phase per candidate
    for i, j in cand_pairs:
        msA, msB = ms_list[i], ms_list[j]
        T = find_first_contact_pair(msA, msB, tol=tol)

        if T is not None:
            if DEBUG:
                print(f"[debug] hit pair ({i},{j}) at T={T}")
            if T < 0.0:
                T = 0.0
            if (best_T is None or
                T < best_T - 1e-12 or
                (abs(T - best_T) <= 1e-12 and (i, j) < best_pair)):
                best_T = T
                best_pair = (i, j)

    if best_T is None:
        return None
    return (best_T, best_pair[0], best_pair[1])

def _selftest():
    # Two unit squares: A at (0,0)-(1,1) moves right 1 unit/s for 10s
    # B at (5,0)-(6,1) moves left 1 unit/s for 10s
    # They touch at x=3 at t=3.0.
    A = [(0,0),(0,1),(1,1),(1,0)]
    B = [(5,0),(5,1),(6,1),(6,0)]
    mpA = MotionPath.from_flat([1, 0, 10])   # vx=+1, vy=0, dt=10
    mpB = MotionPath.from_flat([-1, 0, 10])  # vx=-1, vy=0, dt=10
    ms_list = [MovingShape(A, mpA), MovingShape(B, mpB)]
    ans = earliest_collision(ms_list, tol=1e-6)
    print("SELFTEST result:", ans)

if __name__ == "__main__":
    import sys

    # If no arguments are given, run the self-test instead of loading a file
    if len(sys.argv) == 1:
        print("No input file provided — running internal self-test...")
        _selftest()
        sys.exit(0)

    # Otherwise, treat first argument as input file
    file_path = sys.argv[1]
    try:
        ms_list = load_project_input(file_path)

        DEBUG = True
        cand_pairs = build_candidates_sweep(ms_list)
        if DEBUG:
            print(f"[debug] candidates from broad-phase: {len(cand_pairs)}")
            if len(cand_pairs) > 0:
                print("[debug] first few pairs:", cand_pairs[:10])

        ans = earliest_collision(ms_list, tol=1e-6)
        if ans is None:
            print("-1")
        else:
            T, i, j = ans
            ii, jj = (i, j) if i < j else (j, i)
            print(f"{T:.6f} {ii} {jj}")
    except Exception as e:
        print("Error:", e)
        sys.exit(1)