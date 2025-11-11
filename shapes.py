from dataclasses import dataclass
from typing import List, Iterable, Tuple, Protocol, runtime_checkable
import math
import sys

EPS = 1e-12

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
        if self.y >= min(point1.y, point2.y) and self.y <= math.max(point1.y, point2.y):
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


# plus any intersection helpers

# ===========================
# MOTION / BOUNDING UTILITIES (your part)
# ===========================
# (paste the whole motion + swept AABB block here)
# includes _HasXY, AABB2D, PathSegment, MotionPath, MovingShape, etc.

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

# --- Generic AABB (no class name collisions with your code) ---

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
        """
        Tight AABB of polygon positions while moving from t0 to t1 (clamped).
        For pure translations, the tight axis-aligned swept AABB over [t0, t1]
        is just the AABB of all vertices at t0 and at t1.
        """
        if t1 < t0:
            t0, t1 = t1, t0
        t0 = max(0.0, min(t0, self.t_end))
        t1 = max(0.0, min(t1, self.t_end))
        if t1 - t0 <= EPS:
            return self.aabb_at(t0)
        v0 = self.polygon_at(t0)
        v1 = self.polygon_at(t1)
        return aabb_of_points_generic(v0 + v1)

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

if __name__ == "__main__":
    # CLI: python shapes.py <input_file>
    # If no arg given, fall back to sample filename.
    file_path = sys.argv[1] if len(sys.argv) > 1 else "sample_aabb.txt"
    try:
        ms_list = load_project_input(file_path)
        # You can tweak the sample times if motions are very long.
        _summarize_objects(ms_list, sample_times=(0.0, 2.0, 10.0))
    except Exception as e:
        print("Error:", e)
        sys.exit(1)