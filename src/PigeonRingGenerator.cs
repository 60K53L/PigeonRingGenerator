using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Text;
using System.Xml;

using System.Windows.Forms;


public class PigeonRingGenerator
{
    // ==========================================
    // 1. EARCUT TRIANGULATION
    // ==========================================
    public class Earcut
    {
        public class Node
        {
            public int i;
            public double x, y;
            public Node prev, next;
            public bool steiner;
            public Node(int index, double x, double y)
            {
                this.i = index;
                this.x = x;
                this.y = y;
            }
        }

        private static List<Node> splitStack = new List<Node>();
        private static bool filteredOut = false;

        public static List<int> Triangulate(List<double> data, List<int> holeIndices, int dim = 2)
        {
            splitStack.Clear();
            filteredOut = false;

            bool hasHoles = holeIndices != null && holeIndices.Count > 0;
            int outerLen = hasHoles ? holeIndices[0] * dim : data.Count;

            Node outerNode = LinkedList(data, 0, outerLen, dim, true);
            List<int> triangles = new List<int>();

            if (outerNode == null || outerNode.next == outerNode.prev) return triangles;

            if (hasHoles) outerNode = EliminateHoles(data, holeIndices, outerNode, dim);

            EarcutLinked(outerNode, triangles);
            while (splitStack.Count > 0)
            {
                Node n = splitStack[splitStack.Count - 1];
                splitStack.RemoveAt(splitStack.Count - 1);
                EarcutLinked(n, triangles);
            }

            return triangles;
        }

        private static Node LinkedList(List<double> data, int start, int end, int dim, bool clockwise)
        {
            Node last = null;
            if (clockwise == (SignedArea(data, start, end, dim) > 0))
            {
                for (int i = start; i < end; i += dim)
                    last = InsertNode(i / dim, data[i], data[i + 1], last);
            }
            else
            {
                for (int i = end - dim; i >= start; i -= dim)
                    last = InsertNode(i / dim, data[i], data[i + 1], last);
            }
            if (last != null && Equals(last, last.next))
            {
                RemoveNode(last);
                last = last.next;
            }
            return last;
        }

        private static double SignedArea(List<double> data, int start, int end, int dim)
        {
            double sum = 0;
            for (int i = start, j = end - dim; i < end; i += dim)
            {
                sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
                j = i;
            }
            return sum;
        }

        private static Node InsertNode(int i, double x, double y, Node last)
        {
            Node p = new Node(i, x, y);
            if (last == null)
            {
                p.prev = p;
                p.next = p;
            }
            else
            {
                p.next = last.next;
                p.prev = last;
                last.next.prev = p;
                last.next = p;
            }
            return p;
        }

        private static void RemoveNode(Node p)
        {
            p.prev.next = p.next;
            p.next.prev = p.prev;
        }

        private static bool Equals(Node p1, Node p2)
        {
            return Math.Abs(p1.x - p2.x) < 1e-9 && Math.Abs(p1.y - p2.y) < 1e-9;
        }

        private static Node FilterPoints(Node start, Node end = null)
        {
            if (start == null) return start;
            if (end == null) end = start;

            bool full = (end == start);
            Node p = start;
            bool again;
            do
            {
                again = false;
                if (p != p.next && !p.steiner && (Equals(p, p.next) || Area(p.prev, p, p.next) == 0))
                {
                    if (full || p == end) end = p.prev;
                    filteredOut = true;
                    RemoveNode(p);
                    p = p.prev;
                    again = true;
                }
                else if (full || p != end)
                {
                    p = p.next;
                    again = !full;
                }
            } while (again || p != end);

            return end;
        }

        private static void EarcutLinked(Node ear, List<int> triangles)
        {
            if (ear == null) return;
            Node stop = ear;
            bool cured = false;

            while (ear.prev != ear.next)
            {
                Node prev = ear.prev;
                Node next = ear.next;

                if (Area(prev, ear, next) < 0 && IsEar(ear))
                {
                    triangles.Add(prev.i);
                    triangles.Add(ear.i);
                    triangles.Add(next.i);

                    RemoveNode(ear);
                    ear = next;
                    stop = next;
                    continue;
                }

                ear = next;

                if (ear == stop)
                {
                    filteredOut = false;
                    ear = FilterPoints(ear);
                    if (filteredOut) { stop = ear; continue; }

                    if (!cured)
                    {
                        ear = CureLocalIntersections(ear, triangles);
                        stop = ear;
                        cured = true;
                        continue;
                    }

                    int beforeStack = splitStack.Count;
                    SplitEarcut(ear);
                    if (splitStack.Count == beforeStack)
                    {
                        // Earcut takıldığında kalan poligonu fan üçgenleme ile %100 kapatır (delik kalmasını önler)
                        while (ear != null && ear.prev != ear.next && ear.next.next != ear)
                        {
                            triangles.Add(ear.prev.i);
                            triangles.Add(ear.i);
                            triangles.Add(ear.next.i);
                            RemoveNode(ear);
                            ear = ear.next;
                        }
                        if (ear != null && ear.prev != ear.next)
                        {
                            triangles.Add(ear.prev.i);
                            triangles.Add(ear.i);
                            triangles.Add(ear.next.i);
                        }
                    }
                    break;
                }
            }
        }

        private static bool IsEar(Node ear)
        {
            Node a = ear.prev, b = ear, c = ear.next;
            double ax = a.x, bx = b.x, cx = c.x;
            double ay = a.y, by = b.y, cy = c.y;

            double x0 = Math.Min(ax, Math.Min(bx, cx));
            double y0 = Math.Min(ay, Math.Min(by, cy));
            double x1 = Math.Max(ax, Math.Max(bx, cx));
            double y1 = Math.Max(ay, Math.Max(by, cy));

            Node p = c.next;
            while (p != a)
            {
                if (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1 &&
                    !(ax == p.x && ay == p.y) &&
                    PointInTriangle(ax, ay, bx, by, cx, cy, p.x, p.y) &&
                    Area(p.prev, p, p.next) >= 0)
                {
                    return false;
                }
                p = p.next;
            }
            return true;
        }

        private static Node CureLocalIntersections(Node start, List<int> triangles)
        {
            Node p = start;
            bool cured = false;
            do
            {
                Node a = p.prev;
                Node b = p.next.next;

                if (Intersects(a, p, p.next, b, false) && LocallyInside(a, b) && LocallyInside(b, a))
                {
                    triangles.Add(a.i);
                    triangles.Add(p.i);
                    triangles.Add(b.i);

                    RemoveNode(p);
                    RemoveNode(p.next);

                    p = start = b;
                    cured = true;
                }
                p = p.next;
            } while (p != start);

            return cured ? FilterPoints(p) : p;
        }

        private static void SplitEarcut(Node start)
        {
            Node a = start;
            do
            {
                Node b = a.next.next;
                while (b != a.prev)
                {
                    if (a.i != b.i && IsValidDiagonal(a, b))
                    {
                        Node c = SplitPolygon(a, b);
                        a = FilterPoints(a, a.next);
                        c = FilterPoints(c, c.next);
                        splitStack.Add(c);
                        splitStack.Add(a);
                        return;
                    }
                    b = b.next;
                }
                a = a.next;
            } while (a != start);
        }

        private static Node EliminateHoles(List<double> data, List<int> holeIndices, Node outerNode, int dim)
        {
            List<Node> queue = new List<Node>();
            int len = holeIndices.Count;
            for (int i = 0; i < len; i++)
            {
                int start = holeIndices[i] * dim;
                int end = i < len - 1 ? holeIndices[i + 1] * dim : data.Count;
                Node list = LinkedList(data, start, end, dim, false);
                if (list == list.next) list.steiner = true;
                queue.Add(GetLeftmost(list));
            }

            queue.Sort((a, b) =>
            {
                int c = a.x.CompareTo(b.x);
                if (c != 0) return c;
                c = a.y.CompareTo(b.y);
                if (c != 0) return c;
                double slopeA = (a.next.x != a.x) ? (a.next.y - a.y) / (a.next.x - a.x) : 0;
                double slopeB = (b.next.x != b.x) ? (b.next.y - b.y) / (b.next.x - b.x) : 0;
                return slopeA.CompareTo(slopeB);
            });

            for (int i = 0; i < queue.Count; i++)
            {
                outerNode = EliminateHole(queue[i], outerNode);
            }

            return FilterPoints(outerNode);
        }

        private static Node EliminateHole(Node hole, Node outerNode)
        {
            Node bridge = FindHoleBridge(hole, outerNode);
            if (bridge == null) return outerNode;

            Node bridgeReverse = SplitPolygon(bridge, hole);

            FilterPoints(bridgeReverse.prev, bridgeReverse);

            Node a = bridge.prev;
            if (Area(a, bridge, hole) == 0 && (a.x - bridge.x) * (hole.x - bridge.x) + (a.y - bridge.y) * (hole.y - bridge.y) < 0)
                return bridge;

            return FilterPoints(bridge, bridge.next);
        }

        private static Node FindHoleBridge(Node hole, Node outerNode)
        {
            Node p = outerNode;
            double hx = hole.x, hy = hole.y;
            double qx = -double.MaxValue;
            Node m = null;

            if (Equals(hole, p)) return p;

            do
            {
                if (Equals(hole, p.next)) return p.next;
                else if (hy <= p.y && hy >= p.next.y && p.next.y != p.y)
                {
                    double x = p.x + (hy - p.y) * (p.next.x - p.x) / (p.next.y - p.y);
                    if (x <= hx && x > qx)
                    {
                        qx = x;
                        m = p.x < p.next.x ? p : p.next;
                        if (x == hx) return m;
                    }
                }
                p = p.next;
            } while (p != outerNode);

            if (m == null) return null;

            double mx = m.x, my = m.y;
            double tanMin = double.MaxValue;
            Node stop = m;
            p = m;

            do
            {
                if (hx >= p.x && p.x >= mx && hx != p.x &&
                    PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p.x, p.y))
                {
                    double tan = Math.Abs(hy - p.y) / (hx - p.x);

                    if ((LocallyInside(p, hole) || (p.y == hy && p.next.y == hy && p.next.x > hx)) &&
                        (tan < tanMin || (tan == tanMin && (p.x > m.x || (p.x == m.x && SectorContainsSector(m, p))))))
                    {
                        m = p;
                        tanMin = tan;
                    }
                }
                p = p.next;
            } while (p != stop);

            return m;
        }

        private static bool SectorContainsSector(Node m, Node p)
        {
            return Area(m.prev, m, p.prev) < 0 && Area(p.next, m, m.next) < 0;
        }

        private static Node GetLeftmost(Node start)
        {
            Node p = start, leftmost = start;
            do
            {
                if (p.x < leftmost.x || (p.x == leftmost.x && p.y < leftmost.y)) leftmost = p;
                p = p.next;
            } while (p != start);
            return leftmost;
        }

        private static bool PointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py)
        {
            return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
                   (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
                   (bx - px) * (cy - py) >= (cx - px) * (by - py);
        }

        private static bool IsValidDiagonal(Node a, Node b)
        {
            bool zeroLength = Equals(a, b) && Area(a.prev, a, a.next) > 0 && Area(b.prev, b, b.next) > 0;
            return a.next.i != b.i && (zeroLength || (LocallyInside(a, b) && LocallyInside(b, a) &&
                (Area(a.prev, a, b.prev) != 0 || Area(a, b.prev, b) != 0))) &&
                !IntersectsPolygon(a, b) && (zeroLength || MiddleInside(a, b));
        }

        private static double Area(Node p, Node q, Node r)
        {
            return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        }

        private static bool Intersects(Node p1, Node q1, Node p2, Node q2, bool includeBoundary = true)
        {
            double o1 = Area(p1, q1, p2);
            double o2 = Area(p1, q1, q2);
            double o3 = Area(p2, q2, p1);
            double o4 = Area(p2, q2, q1);

            if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return true;

            if (!includeBoundary) return false;

            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;

            return false;
        }

        private static bool OnSegment(Node p, Node q, Node r)
        {
            return q.x <= Math.Max(p.x, r.x) && q.x >= Math.Min(p.x, r.x) &&
                   q.y <= Math.Max(p.y, r.y) && q.y >= Math.Min(p.y, r.y);
        }

        private static bool IntersectsPolygon(Node a, Node b)
        {
            double minX = Math.Min(a.x, b.x), maxX = Math.Max(a.x, b.x);
            double minY = Math.Min(a.y, b.y), maxY = Math.Max(a.y, b.y);

            Node p = a;
            do
            {
                Node n = p.next;
                if ((p.x > maxX && n.x > maxX) || (p.x < minX && n.x < minX) ||
                    (p.y > maxY && n.y > maxY) || (p.y < minY && n.y < minY))
                {
                    p = n;
                    continue;
                }
                if (p.i != a.i && n.i != a.i && p.i != b.i && n.i != b.i &&
                    Intersects(p, n, a, b)) return true;
                p = n;
            } while (p != a);

            return false;
        }

        private static bool LocallyInside(Node a, Node b)
        {
            return Area(a.prev, a, a.next) < 0 ?
                Area(a, b, a.next) >= 0 && Area(a, a.prev, b) >= 0 :
                Area(a, b, a.prev) < 0 || Area(a, a.next, b) < 0;
        }

        private static bool MiddleInside(Node a, Node b)
        {
            Node p = a;
            bool inside = false;
            double px = (a.x + b.x) / 2.0;
            double py = (a.y + b.y) / 2.0;
            do
            {
                Node n = p.next;
                if (((p.y > py) != (n.y > py)) && (px < (n.x - p.x) * (py - p.y) / (n.y - p.y) + p.x))
                    inside = !inside;
                p = n;
            } while (p != a);

            return inside;
        }

        private static Node SplitPolygon(Node a, Node b)
        {
            Node a2 = new Node(a.i, a.x, a.y);
            Node b2 = new Node(b.i, b.x, b.y);
            Node an = a.next;
            Node bp = b.prev;

            a.next = b;
            b.prev = a;

            a2.next = an;
            an.prev = a2;

            b2.next = a2;
            a2.prev = b2;

            bp.next = b2;
            b2.prev = bp;

            return b2;
        }
    }

    // ==========================================
    // 2. 3D SOLID MESH STRUCTURES
    // ==========================================
    public struct Vec3
    {
        public double x, y, z;
        public Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    public struct Tri
    {
        public int v1, v2, v3;
        public Tri(int v1, int v2, int v3) { this.v1 = v1; this.v2 = v2; this.v3 = v3; }
    }

    public class SolidMesh
    {
        public List<Vec3> Vertices = new List<Vec3>();
        public List<Tri> Triangles = new List<Tri>();

        public int AddVertex(double x, double y, double z)
        {
            Vertices.Add(new Vec3(x, y, z));
            return Vertices.Count - 1;
        }

        public void AddTriangle(int v1, int v2, int v3)
        {
            Triangles.Add(new Tri(v1, v2, v3));
        }

        public double ComputeSignedVolume()
        {
            double vol = 0;
            foreach (var t in Triangles)
            {
                Vec3 a = Vertices[t.v1], b = Vertices[t.v2], c = Vertices[t.v3];
                double crossX = b.y * c.z - b.z * c.y;
                double crossY = b.z * c.x - b.x * c.z;
                double crossZ = b.x * c.y - b.y * c.x;
                vol += (a.x * crossX + a.y * crossY + a.z * crossZ) / 6.0;
            }
            return vol;
        }
    }

    private static double PolygonArea(List<PointF> pts)
    {
        double area = 0;
        int n = pts.Count;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            area += pts[i].X * pts[j].Y - pts[j].X * pts[i].Y;
        }
        return area * 0.5;
    }

    private static bool PointInPolygon(PointF pt, List<PointF> poly)
    {
        bool inside = false;
        int n = poly.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            if (((poly[i].Y > pt.Y) != (poly[j].Y > pt.Y)) &&
                (pt.X < (poly[j].X - poly[i].X) * (pt.Y - poly[i].Y) / (poly[j].Y - poly[i].Y) + poly[i].X))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    // OffsetContour'un ürettiği kontur geçerli mi? (kendini kesme, ters dönme, çakışık nokta)
    private static bool IsValidOffsetContour(List<PointF> c, List<PointF> orig)
    {
        if (c == null || c.Count < 3) return false;

        double area = PolygonArea(c);
        double origArea = PolygonArea(orig);
        if (Math.Abs(origArea) < 1e-9) return false;
        if (Math.Sign(area) != Math.Sign(origArea)) return false;
        if (Math.Abs(area) < Math.Abs(origArea) * 0.05) return false;

        int n = c.Count;
        for (int i = 0; i < n; i++)
        {
            PointF a = c[i];
            PointF b = c[(i + 1) % n];
            if (Math.Abs(a.X - b.X) < 1e-3 && Math.Abs(a.Y - b.Y) < 1e-3) return false;
        }

        // Kendini kesme testi (komşu kenarlar hariç)
        for (int i = 0; i < n; i++)
        {
            int i2 = (i + 1) % n;
            for (int j = i + 1; j < n; j++)
            {
                int j2 = (j + 1) % n;
                if (i == j || i2 == j || j2 == i) continue;
                if (SegmentsIntersect(c[i], c[i2], c[j], c[j2])) return false;
            }
        }
        return true;
    }

    private static double Cross2(PointF a, PointF b, PointF c)
    {
        return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
    }

    private static bool OnSegment(PointF a, PointF b, PointF p)
    {
        return Math.Min(a.X, b.X) - 1e-9 <= p.X && p.X <= Math.Max(a.X, b.X) + 1e-9 &&
               Math.Min(a.Y, b.Y) - 1e-9 <= p.Y && p.Y <= Math.Max(a.Y, b.Y) + 1e-9;
    }

    private static bool SegmentsIntersect(PointF p1, PointF p2, PointF p3, PointF p4)
    {
        double d1 = Cross2(p3, p4, p1);
        double d2 = Cross2(p3, p4, p2);
        double d3 = Cross2(p1, p2, p3);
        double d4 = Cross2(p1, p2, p4);

        if (((d1 > 1e-9 && d2 < -1e-9) || (d1 < -1e-9 && d2 > 1e-9)) &&
            ((d3 > 1e-9 && d4 < -1e-9) || (d3 < -1e-9 && d4 > 1e-9)))
            return true;

        if (Math.Abs(d1) <= 1e-9 && OnSegment(p3, p4, p1)) return true;
        if (Math.Abs(d2) <= 1e-9 && OnSegment(p3, p4, p2)) return true;
        if (Math.Abs(d3) <= 1e-9 && OnSegment(p1, p2, p3)) return true;
        if (Math.Abs(d4) <= 1e-9 && OnSegment(p1, p2, p4)) return true;

        return false;
    }

    // Güvenli offset: tam pahı dener, kontur bozulursa yarıya/çeyreğe düşer,
    // en sonunda pahsız (nominal) kontura döner. Böylece mesh her zaman kapalı kalır.
    public static List<PointF> SafeOffsetContour(List<PointF> pts, double delta, bool isHole)
    {
        if (Math.Abs(delta) < 1e-5) return new List<PointF>(pts);
        double[] factors = new double[] { 1.0, 0.5, 0.25 };
        foreach (double f in factors)
        {
            List<PointF> off = OffsetContour(pts, delta * f, isHole);
            if (IsValidOffsetContour(off, pts)) return off;
        }
        return new List<PointF>(pts);
    }

    public static List<PointF> OffsetContour(List<PointF> pts, double delta, bool isHole)
    {
        int n = pts.Count;
        if (n < 3 || Math.Abs(delta) < 1e-5) return new List<PointF>(pts);

        double area = 0;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            area += pts[i].X * pts[j].Y - pts[j].X * pts[i].Y;
        }

        double effDelta = isHole ? -delta : delta;
        double dir = (area > 0) ? 1.0 : -1.0;

        List<PointF> result = new List<PointF>(n);
        PointF[] normals = new PointF[n];

        for (int i = 0; i < n; i++)
        {
            int next = (i + 1) % n;
            double dx = pts[next].X - pts[i].X;
            double dy = pts[next].Y - pts[i].Y;
            double len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-6) normals[i] = new PointF(0, 0);
            else normals[i] = new PointF((float)(dir * dy / len), (float)(-dir * dx / len));
        }

        for (int i = 0; i < n; i++)
        {
            int prev = (i - 1 + n) % n;
            PointF n1 = normals[prev];
            PointF n2 = normals[i];

            double dot = n1.X * n2.X + n1.Y * n2.Y;
            double bisectX = n1.X + n2.X;
            double bisectY = n1.Y + n2.Y;
            double bisectLen = Math.Sqrt(bisectX * bisectX + bisectY * bisectY);

            if (bisectLen < 1e-4)
            {
                result.Add(new PointF((float)(pts[i].X + n2.X * effDelta), (float)(pts[i].Y + n2.Y * effDelta)));
            }
            else
            {
                double miterScale = 2.0 / (1.0 + Math.Max(-0.5, dot));
                if (miterScale > 2.0) miterScale = 2.0;
                double shift = effDelta * (miterScale / bisectLen);
                result.Add(new PointF((float)(pts[i].X + bisectX * shift), (float)(pts[i].Y + bisectY * shift)));
            }
        }
        return result;
    }

    public static List<RectangleF> GetNaturalCharBounds(Graphics g, string text, Font font, StringFormat sf)
    {
        List<RectangleF> list = new List<RectangleF>();
        if (string.IsNullOrEmpty(text)) return list;
        int total = text.Length;
        int batchSize = 32;

        for (int start = 0; start < total; start += batchSize)
        {
            int count = Math.Min(batchSize, total - start);
            CharacterRange[] ranges = new CharacterRange[count];
            for (int i = 0; i < count; i++) ranges[i] = new CharacterRange(start + i, 1);

            sf.SetMeasurableCharacterRanges(ranges);
            Region[] regions = g.MeasureCharacterRanges(text, font, new RectangleF(0, 0, 100000, 10000), sf);
            for (int i = 0; i < regions.Length; i++)
            {
                list.Add(regions[i].GetBounds(g));
                regions[i].Dispose();
            }
        }
        return list;
    }

    // ==========================================
    // 3. 360-DEGREE PRECISE RADIUS PROFILE
    // ==========================================
    private static readonly double[] DegKeys = new double[] {
        0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
        100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175,
        180, 185, 190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 255,
        260, 265, 270, 275, 280, 285, 290, 295, 300, 305, 310, 315, 320, 325, 330, 335,
        340, 345, 350, 355, 360
    };
    private static readonly double[] RValues = new double[] {
        6.042, 6.020, 5.990, 5.968, 6.200, 6.651, 6.917, 6.943, 6.942, 6.791, 6.403, 6.340,
        6.358, 6.319, 6.067, 5.771, 5.887, 6.331, 6.310, 5.849, 5.769, 5.777, 5.783, 5.792,
        5.802, 5.820, 5.827, 5.842, 5.863, 5.888, 5.901, 5.932, 5.952, 5.972, 6.001, 6.015,
        6.036, 6.061, 6.093, 6.119, 6.140, 6.159, 6.177, 6.199, 6.222, 6.242, 6.256, 6.268,
        6.278, 6.290, 6.301, 6.307, 6.311, 6.312, 6.313, 6.311, 6.310, 6.305, 6.298, 6.290,
        6.281, 6.262, 6.254, 6.234, 6.216, 6.200, 6.182, 6.161, 6.135, 6.107, 6.084, 6.063,
        6.042
    };

    public static double GetInterpolatedRadius(double deg)
    {
        while (deg < 0) deg += 360.0;
        while (deg >= 360.0) deg -= 360.0;
        for (int i = 0; i < DegKeys.Length - 1; i++)
        {
            if (deg >= DegKeys[i] && deg <= DegKeys[i + 1])
            {
                double t = (deg - DegKeys[i]) / (DegKeys[i + 1] - DegKeys[i]);
                return RValues[i] + t * (RValues[i + 1] - RValues[i]);
            }
        }
        return 6.10;
    }

    // ==========================================
    // 4. CYLINDRICAL DISTRIBUTED TEXT GENERATION
    // ==========================================
    // ==========================================
    // 4. CYLINDRICAL DISTRIBUTED TEXT GENERATION (WITH AUTO-FIT)
    // ==========================================
    // ==========================================
    // 4. CYLINDRICAL DISTRIBUTED TEXT GENERATION (WITH AUTO-FIT & ROBUST TOPOLOGY)
    // ==========================================
    // ==========================================
    // 3.5 GENERATOR MODES (İSİMLİ, SIRALI, MARKA TİPİ)
    // ==========================================
    public enum GeneratorMode
    {
        IsimliKlips,    // Mode 1: Kişiye Özel İsim & Telefon Klipsi
        SiraliKlips,    // Mode 2: Numaralı Seri Klipsler (2 Tabla Multi-Plate)
        MarkaTipi       // Mode 3: Kesiksiz Yuvarlak Kapalı Marka Halkası
    }

    // ==========================================
    // 4. CYLINDRICAL DISTRIBUTED TEXT GENERATION (ARIAL BOLD + ROBUST TOPOLOGY)
    // ==========================================
    // Kullanıcı G yazdığında tik seçiliyse otomatik Ğ yapıcı, seçili değilse şapkasız G yapıcı; 'i' harfini (örn: BEKiR) daima korur:
    public static string ProcessTextWithSoftG(string input, bool useSoftG)
    {
        if (string.IsNullOrEmpty(input)) return input;
        // Tik seçili değilse: Kullanıcının yazdığı metin harfi harfine aynen korunur!
        if (!useSoftG)
        {
            return input;
        }

        // Türkçe Karakter Tiki seçili ise:
        // i -> İ, ı -> I, ğ -> Ğ, ş -> Ş, ç -> Ç, ö -> Ö, ü -> Ü
        // Normal C, S, G harfleri ASLA Ç, Ş, Ğ yapılmaz! (C->C, S->S, G->G kalır)
        StringBuilder sb = new StringBuilder();
        foreach (char c in input)
        {
            if (c == 'i' || c == 'İ')
            {
                sb.Append('İ');
            }
            else if (c == 'ı' || c == 'I')
            {
                sb.Append('I');
            }
            else if (c == 'ğ' || c == 'Ğ')
            {
                sb.Append('Ğ');
            }
            else if (c == 'ş' || c == 'Ş')
            {
                sb.Append('Ş');
            }
            else if (c == 'ç' || c == 'Ç')
            {
                sb.Append('Ç');
            }
            else if (c == 'ö' || c == 'Ö')
            {
                sb.Append('Ö');
            }
            else if (c == 'ü' || c == 'Ü')
            {
                sb.Append('Ü');
            }
            else
            {
                // Normal harfleri standart büyüt (c->C, s->S, g->G)
                sb.Append(char.ToUpper(c, new CultureInfo("tr-TR")));
            }
        }
        return sb.ToString();
    }

    // Karakter Yolu Üretici: Türkçe harfleri (Ğ, Ş, Ç, İ, Ö, Ü) noktasız büyük harf boyutuna eşitleyen sağlam manifold glifler
    public static GraphicsPath CreateCharPath(char c, FontFamily ff, int style, float emSize, StringFormat sf, bool useTurkishFix = false)
    {
        GraphicsPath gp = new GraphicsPath();

        // Eğer Türkçe karakter tiki kapalı ise fontun kendi doğal karakterini çiz:
        if (!useTurkishFix)
        {
            gp.AddString(c.ToString(), ff, style, emSize, new PointF(0, 0), sf);
            return gp;
        }

        // 1. Ğ glifi: G gövdesine bağlı, boyutu bozmayan kompakt şapka
        if (c == 'Ğ' || c == 'ğ')
        {
            gp.AddString("G", ff, style, emSize, new PointF(0, 0), sf);
            RectangleF bG = gp.GetBounds();

            PointF[] origBreve = new PointF[] {
                new PointF(53.15f, 0.65f), new PointF(52.85f, 3.90f), new PointF(52.00f, 6.90f),
                new PointF(50.50f, 9.60f), new PointF(48.45f, 12.00f), new PointF(45.95f, 14.00f),
                new PointF(43.10f, 15.40f), new PointF(39.90f, 16.25f), new PointF(36.35f, 16.55f),
                new PointF(32.95f, 16.30f), new PointF(29.95f, 15.45f), new PointF(27.30f, 14.10f),
                new PointF(25.00f, 12.20f), new PointF(23.15f, 9.85f), new PointF(21.85f, 7.15f),
                new PointF(21.05f, 4.05f), new PointF(20.80f, 0.65f), new PointF(27.65f, 0.65f),
                new PointF(27.80f, 2.15f), new PointF(28.30f, 3.45f), new PointF(29.10f, 4.65f),
                new PointF(30.20f, 5.65f), new PointF(33.05f, 7.05f), new PointF(36.35f, 7.55f),
                new PointF(40.35f, 7.10f), new PointF(43.45f, 5.70f), new PointF(44.65f, 4.70f),
                new PointF(45.50f, 3.55f), new PointF(46.00f, 2.20f), new PointF(46.15f, 0.65f)
            };

            float targetBottomY = bG.Top + 1.20f;
            float targetTopY = bG.Top - 6.50f;

            PointF[] attachedBreve = new PointF[origBreve.Length];
            float origCenterX = 36.975f;
            float centerX = bG.X + bG.Width * 0.50f;

            for (int i = 0; i < origBreve.Length; i++)
            {
                float t = (16.55f - origBreve[i].Y) / 15.90f;
                float newY = targetBottomY - (t * (targetBottomY - targetTopY));
                float newX = centerX + (origBreve[i].X - origCenterX);
                attachedBreve[i] = new PointF(newX, newY);
            }

            gp.AddPolygon(attachedBreve);
            return gp;
        }

        // 2. İ glifi: Standart I boyutunda (tavanı ve tabanı I ile birebir eşit), tek parça manifold gövde, belirgin yuvarlak başlıklı ve çentikli büyük İ
        if (c == 'İ' || c == 'i')
        {
            GraphicsPath gpRef = new GraphicsPath();
            gpRef.AddString("H", ff, style, emSize, new PointF(0, 0), sf);
            RectangleF bRef = gpRef.GetBounds();
            float refH = bRef.Height > 0 ? bRef.Height : emSize * 0.72f;
            float refTop = bRef.Top;
            float refBottom = bRef.Bottom;

            GraphicsPath gpI = new GraphicsPath();
            gpI.AddString("I", ff, style, emSize, new PointF(0, 0), sf);
            gpI.Flatten(null, 0.20f);
            RectangleF bI = gpI.GetBounds();

            float stemW = bI.Width;
            try
            {
                float yMid = bI.Top + bI.Height * 0.5f;
                PointF[] iPts = gpI.PathPoints;
                byte[] iTypes = gpI.PathTypes;
                float minMidX = float.MaxValue, maxMidX = float.MinValue;
                int segStart = 0;
                for (int pi = 0; pi < iPts.Length; pi++)
                {
                    bool isClose = (iTypes[pi] & 0x80) != 0;
                    bool nextStart = (pi + 1 < iPts.Length && (iTypes[pi + 1] & 0x07) == 0);
                    if (isClose || nextStart || pi == iPts.Length - 1)
                    {
                        for (int pk = segStart; pk <= pi; pk++)
                        {
                            PointF pa = iPts[pk];
                            PointF pc = iPts[(pk == pi) ? segStart : (pk + 1)];
                            if ((pa.Y <= yMid && pc.Y > yMid) || (pc.Y <= yMid && pa.Y > yMid))
                            {
                                float ix = pa.X + (yMid - pa.Y) * (pc.X - pa.X) / (pc.Y - pa.Y);
                                if (ix < minMidX) minMidX = ix;
                                if (ix > maxMidX) maxMidX = ix;
                            }
                        }
                        segStart = pi + 1;
                    }
                }
                if (maxMidX > minMidX) stemW = maxMidX - minMidX;
            }
            catch { }
            float dotD = Math.Min(stemW * 1.50f, stemW + refH * 0.08f); // Serif tırnaklarından bağımsız orantılı yuvarlak başlık
            float dotH = refH * 0.24f;  // Belirgin nokta yüksekliği (~17 birim = 0.82 mm)
            float cx = bI.Left + bI.Width * 0.5f;

            float dotCenterY = refTop + dotH * 0.5f;
            float rx = dotD * 0.5f;
            float ry = dotH * 0.5f;

            float waistY = refTop + dotH * 0.85f;
            float waistW = stemW * 0.70f; // İçe kıvrılan belirgin V-çentik

            List<PointF> poly = new List<PointF>();
            poly.Add(new PointF(cx - stemW * 0.5f, refBottom));
            poly.Add(new PointF(cx - stemW * 0.5f, waistY + 2.0f));
            poly.Add(new PointF(cx - waistW * 0.5f, waistY));

            int numArcSteps = 16;
            double angleStart = Math.PI * 1.15;
            double angleEnd = -Math.PI * 0.15;
            for (int k = 0; k <= numArcSteps; k++)
            {
                double t = (double)k / numArcSteps;
                double angle = angleStart + t * (angleEnd - angleStart);
                float px = cx + (float)(rx * Math.Cos(angle));
                float py = dotCenterY - (float)(ry * Math.Sin(angle));
                poly.Add(new PointF(px, py));
            }

            poly.Add(new PointF(cx + waistW * 0.5f, waistY));
            poly.Add(new PointF(cx + stemW * 0.5f, waistY + 2.0f));
            poly.Add(new PointF(cx + stemW * 0.5f, refBottom));

            gp.AddPolygon(poly.ToArray());
            return gp;
        }

        // 3. Ö glifi: O gövdesine tam birleşik tek parça dış kontur (iki belirgin tepe noktası) + iç delik, tavanı ve tabanı standart harfle birebir eşit büyük Ö
        if (c == 'Ö' || c == 'ö')
        {
            GraphicsPath gpRef = new GraphicsPath();
            gpRef.AddString("H", ff, style, emSize, new PointF(0, 0), sf);
            RectangleF bRef = gpRef.GetBounds();
            float refH = bRef.Height > 0 ? bRef.Height : emSize * 0.72f;
            float refTop = bRef.Top;
            float refBottom = bRef.Bottom;

            GraphicsPath gpO = new GraphicsPath();
            gpO.AddString("O", ff, style, emSize, new PointF(0, 0), sf);
            gpO.Flatten(null, 0.20f);

            PointF[] pts = gpO.PathPoints;
            byte[] types = gpO.PathTypes;
            List<List<PointF>> contours = new List<List<PointF>>();
            List<PointF> cur = new List<PointF>();
            for (int i = 0; i < pts.Length; i++)
            {
                if ((types[i] & 0x07) == 0 && cur.Count > 0)
                {
                    contours.Add(cur);
                    cur = new List<PointF>();
                }
                cur.Add(pts[i]);
                if ((types[i] & 0x80) != 0)
                {
                    contours.Add(cur);
                    cur = new List<PointF>();
                }
            }
            if (cur.Count > 0) contours.Add(cur);

            List<PointF> outer = (contours.Count > 1 && contours[1].Count > contours[0].Count) ? contours[1] : (contours.Count > 0 ? contours[0] : new List<PointF>());
            List<PointF> hole = (contours.Count > 1 && contours[1].Count <= contours[0].Count) ? contours[1] : (contours.Count > 1 ? contours[0] : new List<PointF>());

            RectangleF bO = gpO.GetBounds();
            float cx = bO.Left + bO.Width * 0.5f;

            float dotW = bO.Width * 0.28f;
            float dotH = refH * 0.22f;
            float sp = bO.Width * 0.22f;

            float bodyTop = refTop + dotH * 0.65f;
            float scaleY = (refBottom - bodyTop) / (refBottom - bO.Top);

            List<PointF> scaledHole = new List<PointF>();
            foreach (var p in hole)
            {
                float ny = refBottom - (refBottom - p.Y) * scaleY;
                scaledHole.Add(new PointF(p.X, ny));
            }

            float cxL = cx - sp;
            float cxR = cx + sp;
            float rx = dotW * 0.55f;

            List<PointF> unifiedOuter = new List<PointF>();
            int numSteps = 72;
            for (int i = 0; i < numSteps; i++)
            {
                double angle = (2.0 * Math.PI * i) / numSteps;
                float cosA = (float)Math.Cos(angle);
                float sinA = (float)Math.Sin(angle);

                float px = cx + (bO.Width * 0.5f) * cosA;
                float py;

                if (sinA >= 0)
                {
                    py = bodyTop + ((refBottom - bodyTop) * 0.5f) * (1.0f + sinA);
                }
                else
                {
                    float distL = Math.Abs(px - cxL);
                    float distR = Math.Abs(px - cxR);

                    float bumpL = 0;
                    if (distL < rx)
                    {
                        float t = distL / rx;
                        bumpL = (float)Math.Sqrt(Math.Max(0f, 1.0f - t * t)) * (bodyTop - refTop);
                    }

                    float bumpR = 0;
                    if (distR < rx)
                    {
                        float t = distR / rx;
                        bumpR = (float)Math.Sqrt(Math.Max(0f, 1.0f - t * t)) * (bodyTop - refTop);
                    }

                    float bump = Math.Max(bumpL, bumpR);
                    float nominalTop = bodyTop + ((refBottom - bodyTop) * 0.5f) * (1.0f + sinA);
                    py = Math.Min(nominalTop, bodyTop - bump);
                }

                unifiedOuter.Add(new PointF(px, py));
            }

            gp.AddPolygon(unifiedOuter.ToArray());
            if (scaledHole.Count > 2) gp.AddPolygon(scaledHole.ToArray());
            return gp;
        }

        // 4. Ü glifi: U bacaklarına tam birleşik iki belirgin yuvarlak başlık (nokta), tavanı ve tabanı standart harfle birebir eşit büyük Ü
        if (c == 'Ü' || c == 'ü')
        {
            GraphicsPath gpRef = new GraphicsPath();
            gpRef.AddString("H", ff, style, emSize, new PointF(0, 0), sf);
            RectangleF bRef = gpRef.GetBounds();
            float refH = bRef.Height > 0 ? bRef.Height : emSize * 0.72f;
            float refTop = bRef.Top;
            float refBottom = bRef.Bottom;

            GraphicsPath gpU = new GraphicsPath();
            gpU.AddString("U", ff, style, emSize, new PointF(0, 0), sf);
            gpU.Flatten(null, 0.20f);

            RectangleF bU = gpU.GetBounds();
            float dotH = refH * 0.22f;
            float bodyTop = refTop + dotH * 0.65f;
            float scaleY = (refBottom - bodyTop) / (refBottom - bU.Top);

            PointF[] pts = gpU.PathPoints;
            List<PointF> scaledPts = new List<PointF>();
            foreach (var p in pts)
            {
                float ny = refBottom - (refBottom - p.Y) * scaleY;
                scaledPts.Add(new PointF(p.X, ny));
            }

            float minX = bU.Left;
            float maxX = bU.Right;
            float stemW = bU.Width * 0.24f;
            float cxL = minX + stemW * 0.5f;
            float cxR = maxX - stemW * 0.5f;

            float dotD = stemW * 1.45f;
            float rx = dotD * 0.5f;
            float ry = (bodyTop - refTop);

            List<PointF> poly = new List<PointF>();
            for (int i = 0; i < scaledPts.Count; i++)
            {
                PointF p = scaledPts[i];
                PointF next = scaledPts[(i + 1) % scaledPts.Count];

                bool isLeftTop = (Math.Abs(p.Y - bodyTop) < 3.0f && Math.Abs(next.Y - bodyTop) < 3.0f &&
                                  p.X <= cxL + stemW * 0.8f && next.X <= cxL + stemW * 0.8f);

                bool isRightTop = (Math.Abs(p.Y - bodyTop) < 3.0f && Math.Abs(next.Y - bodyTop) < 3.0f &&
                                   p.X >= cxR - stemW * 0.8f && next.X >= cxR - stemW * 0.8f);

                if (isLeftTop)
                {
                    float startX = p.X;
                    float endX = next.X;
                    int arcSteps = 12;
                    for (int s = 0; s <= arcSteps; s++)
                    {
                        double t = (double)s / arcSteps;
                        double angle = Math.PI - t * Math.PI;
                        if (startX > endX) angle = t * Math.PI;

                        float kx = cxL + (float)(rx * Math.Cos(angle));
                        float ky = bodyTop - (float)(ry * Math.Sin(angle));
                        poly.Add(new PointF(kx, ky));
                    }
                }
                else if (isRightTop)
                {
                    float startX = p.X;
                    float endX = next.X;
                    int arcSteps = 12;
                    for (int s = 0; s <= arcSteps; s++)
                    {
                        double t = (double)s / arcSteps;
                        double angle = Math.PI - t * Math.PI;
                        if (startX > endX) angle = t * Math.PI;

                        float kx = cxR + (float)(rx * Math.Cos(angle));
                        float ky = bodyTop - (float)(ry * Math.Sin(angle));
                        poly.Add(new PointF(kx, ky));
                    }
                }
                else
                {
                    poly.Add(p);
                }
            }

            gp.AddPolygon(poly.ToArray());
            return gp;
        }

        // 5. Ş glifi: S gövdesine bağlı, alt bant sınırını delmeyen kompakt kuyruklu Ş
        if (c == 'Ş' || c == 'ş')
        {
            GraphicsPath gpS = new GraphicsPath();
            gpS.AddString("S", ff, style, emSize, new PointF(0, 0), sf);
            gpS.Flatten(null, 0.20f);
            RectangleF bS = gpS.GetBounds();

            gp.AddPath(gpS, false);

            float cx = bS.Left + bS.Width * 0.52f;
            float cy = bS.Bottom - 1.5f;

            PointF[] cedilla = new PointF[] {
                new PointF(cx - 3.5f, cy),
                new PointF(cx + 3.5f, cy),
                new PointF(cx + 2.0f, cy + 5.0f),
                new PointF(cx - 1.0f, cy + 6.0f),
                new PointF(cx - 3.0f, cy + 4.5f)
            };
            gp.AddPolygon(cedilla);
            return gp;
        }

        // 6. Ç glifi: C gövdesine bağlı, alt bant sınırını delmeyen kompakt kuyruklu Ç
        if (c == 'Ç' || c == 'ç')
        {
            GraphicsPath gpC = new GraphicsPath();
            gpC.AddString("C", ff, style, emSize, new PointF(0, 0), sf);
            gpC.Flatten(null, 0.20f);
            RectangleF bC = gpC.GetBounds();

            gp.AddPath(gpC, false);

            float cx = bC.Left + bC.Width * 0.50f;
            float cy = bC.Bottom - 1.5f;

            PointF[] cedilla = new PointF[] {
                new PointF(cx - 3.5f, cy),
                new PointF(cx + 3.5f, cy),
                new PointF(cx + 2.0f, cy + 5.0f),
                new PointF(cx - 1.0f, cy + 6.0f),
                new PointF(cx - 3.0f, cy + 4.5f)
            };
            gp.AddPolygon(cedilla);
            return gp;
        }

        gp.AddString(c.ToString(), ff, style, emSize, new PointF(0, 0), sf);
        return gp;
    }

    // Konturdan çakışık ve doğrusal (collinear) noktaları temizler.
    // Earcut bu noktaları atladığı için kapak üçgenlemesi ile yan duvar döngüsü
    // uyuşmaz ve mesh'te açık kenarlar (su geçirmezlik hatası) oluşur.
    
    // Düz kenarları kavisli silindire sarmak için ara nokta ekleyici (Subdivision)
    private static List<PointF> SubdivideContour(List<PointF> pts, double maxSegmentLenLocal)
    {
        if (pts == null || pts.Count < 2) return pts;
        List<PointF> result = new List<PointF>();
        for (int i = 0; i < pts.Count; i++)
        {
            PointF p1 = pts[i];
            PointF p2 = pts[(i + 1) % pts.Count];
            result.Add(p1);

            double dx = p2.X - p1.X;
            double dy = p2.Y - p1.Y;
            double dist = Math.Sqrt(dx * dx + dy * dy);

            if (dist > maxSegmentLenLocal)
            {
                int numSubdivisions = (int)Math.Ceiling(dist / maxSegmentLenLocal);
                for (int k = 1; k < numSubdivisions; k++)
                {
                    double t = (double)k / numSubdivisions;
                    result.Add(new PointF((float)(p1.X + t * dx), (float)(p1.Y + t * dy)));
                }
            }
        }
        return result;
    }

private static List<PointF> CleanContour(List<PointF> pts, double tol)
    {
        if (pts == null) return new List<PointF>();
        if (pts.Count < 3) return new List<PointF>(pts);
        if (tol < 1e-9) tol = 1e-9;

        List<PointF> p = new List<PointF>(pts.Count);
        for (int i = 0; i < pts.Count; i++)
        {
            PointF c = pts[i];
            if (p.Count > 0)
            {
                PointF last = p[p.Count - 1];
                if (Math.Abs(c.X - last.X) <= tol && Math.Abs(c.Y - last.Y) <= tol) continue;
            }
            p.Add(c);
        }
        while (p.Count > 1)
        {
            PointF a = p[0];
            PointF b = p[p.Count - 1];
            if (Math.Abs(a.X - b.X) <= tol && Math.Abs(a.Y - b.Y) <= tol) p.RemoveAt(p.Count - 1);
            else break;
        }
        if (p.Count < 3) return p;

        bool changed = true;
        while (changed && p.Count > 3)
        {
            changed = false;
            int n = p.Count;
            for (int i = 0; i < n; i++)
            {
                PointF prev = p[(i - 1 + n) % n];
                PointF cur = p[i];
                PointF next = p[(i + 1) % n];

                double ax = cur.X - prev.X, ay = cur.Y - prev.Y;
                double bx = next.X - cur.X, by = next.Y - cur.Y;
                double lenA = Math.Sqrt(ax * ax + ay * ay);
                double lenB = Math.Sqrt(bx * bx + by * by);
                if (lenA <= tol || lenB <= tol) { p.RemoveAt(i); changed = true; break; }

                double perp = Math.Abs(ax * by - ay * bx) / lenA;
                double dot = ax * bx + ay * by;
                if (perp <= tol && dot > 0) { p.RemoveAt(i); changed = true; break; }
            }
        }
        return p;
    }

    // Glif konturlarını (dış + delikler) Earcut için hazırlar.
    // Dış kontur saat yönünün tersine (CCW), delikler saat yönüne (CW) zorlanır.
    
    // ==========================================
    // ==========================================
    // USE-SURFACE: GERÇEK ŞABLON YÜZEYİ ÖRNEKLEME VE MESH YARDIMCILARI
    // ==========================================
    public static SolidMesh ParseMeshFromModelXml(string modelXmlContent)
    {
        SolidMesh mesh = new SolidMesh();
        XmlDocument doc = new XmlDocument();
        doc.LoadXml(modelXmlContent);
        XmlNamespaceManager nsmgr = new XmlNamespaceManager(doc.NameTable);
        nsmgr.AddNamespace("m", "http://schemas.microsoft.com/3dmanufacturing/core/2015/02");

        XmlNodeList vNodes = doc.SelectNodes("//m:mesh/m:vertices/m:vertex", nsmgr);
        foreach (XmlNode n in vNodes)
        {
            double x = double.Parse(n.Attributes["x"].Value, CultureInfo.InvariantCulture);
            double y = double.Parse(n.Attributes["y"].Value, CultureInfo.InvariantCulture);
            double z = double.Parse(n.Attributes["z"].Value, CultureInfo.InvariantCulture);
            mesh.AddVertex(x, y, z);
        }

        XmlNodeList tNodes = doc.SelectNodes("//m:mesh/m:triangles/m:triangle", nsmgr);
        foreach (XmlNode n in tNodes)
        {
            int v1 = int.Parse(n.Attributes["v1"].Value);
            int v2 = int.Parse(n.Attributes["v2"].Value);
            int v3 = int.Parse(n.Attributes["v3"].Value);
            mesh.AddTriangle(v1, v2, v3);
        }
        return mesh;
    }

    public class RingSurfaceSampler
    {
        private readonly SolidMesh _mesh;
        private readonly double _cx;
        private readonly double _cy;
        private readonly double _minZ;
        private readonly double _maxZ;
        private readonly List<int>[] _angleBuckets;
        private readonly double[] _triMinZ;
        private readonly double[] _triMaxZ;

        public double MinZ { get { return _minZ; } }
        public double MaxZ { get { return _maxZ; } }

        public RingSurfaceSampler(SolidMesh ringMesh, double cx, double cy)
        {
            _mesh = ringMesh;
            _cx = cx;
            _cy = cy;
            _angleBuckets = new List<int>[360];
            for (int i = 0; i < 360; i++) _angleBuckets[i] = new List<int>();

            double minZ = double.MaxValue, maxZ = double.MinValue;
            foreach (var v in ringMesh.Vertices)
            {
                if (v.z < minZ) minZ = v.z;
                if (v.z > maxZ) maxZ = v.z;
            }
            _minZ = minZ < maxZ ? minZ : -4.5;
            _maxZ = maxZ > minZ ? maxZ : 4.5;

            int triCount = ringMesh.Triangles.Count;
            _triMinZ = new double[triCount];
            _triMaxZ = new double[triCount];

            for (int ti = 0; ti < triCount; ti++)
            {
                Tri t = ringMesh.Triangles[ti];
                Vec3 a = ringMesh.Vertices[t.v1];
                Vec3 b = ringMesh.Vertices[t.v2];
                Vec3 c = ringMesh.Vertices[t.v3];

                double tzMin = Math.Min(a.z, Math.Min(b.z, c.z));
                double tzMax = Math.Max(a.z, Math.Max(b.z, c.z));
                _triMinZ[ti] = tzMin;
                _triMaxZ[ti] = tzMax;

                // Tam yatay kapak üçgenlerini dış duvar taramasından ele
                if (Math.Abs(tzMax - tzMin) < 1e-6) continue;

                double a1 = Math.Atan2(a.y - cy, a.x - cx) * 180.0 / Math.PI;
                double a2 = Math.Atan2(b.y - cy, b.x - cx) * 180.0 / Math.PI;
                double a3 = Math.Atan2(c.y - cy, c.x - cx) * 180.0 / Math.PI;
                a1 = ((a1 % 360.0) + 360.0) % 360.0;
                a2 = ((a2 % 360.0) + 360.0) % 360.0;
                a3 = ((a3 % 360.0) + 360.0) % 360.0;

                // Unwrap a2, a3 relative to a1
                double d2 = ((a2 - a1 + 540.0) % 360.0) - 180.0;
                double d3 = ((a3 - a1 + 540.0) % 360.0) - 180.0;
                double lo = a1 + Math.Min(0.0, Math.Min(d2, d3)) - 2.0;
                double hi = a1 + Math.Max(0.0, Math.Max(d2, d3)) + 2.0;

                int iLo = (int)Math.Floor(lo);
                int iHi = (int)Math.Ceiling(hi);
                if (iHi - iLo > 180)
                {
                    for (int k = 0; k < 360; k++) _angleBuckets[k].Add(ti);
                }
                else
                {
                    for (int k = iLo; k <= iHi; k++)
                    {
                        int bk = ((k % 360) + 360) % 360;
                        _angleBuckets[bk].Add(ti);
                    }
                }
            }
        }

        public double SampleRadius(double thetaRad, double zLocal, double fallbackR)
        {
            double zClamp = Math.Max(_minZ + 1e-4, Math.Min(_maxZ - 1e-4, zLocal));
            double deg = ((thetaRad * 180.0 / Math.PI) % 360.0 + 360.0) % 360.0;
            int bucket = ((int)Math.Floor(deg)) % 360;

            double ux = Math.Cos(thetaRad);
            double uy = Math.Sin(thetaRad);

            double bestT = RayBucketMaxT(bucket, ux, uy, zClamp);
            if (bestT <= 0.0)
            {
                // Nadir sınır geçişleri için ufak Z ofseti dene
                bestT = RayBucketMaxT(bucket, ux, uy, Math.Min(_maxZ - 1e-4, zClamp + 0.002));
                if (bestT <= 0.0)
                    bestT = RayBucketMaxT(bucket, ux, uy, Math.Max(_minZ + 1e-4, zClamp - 0.002));
            }
            return (bestT > 0.5) ? bestT : fallbackR;
        }

        private double RayBucketMaxT(int bucket, double ux, double uy, double pz)
        {
            List<int> list = _angleBuckets[bucket];
            double maxT = -1.0;
            const double uvTol = 1e-4;

            for (int i = 0; i < list.Count; i++)
            {
                int ti = list[i];
                if (pz < _triMinZ[ti] - 1e-4 || pz > _triMaxZ[ti] + 1e-4) continue;

                Tri tr = _mesh.Triangles[ti];
                Vec3 v0 = _mesh.Vertices[tr.v1];
                Vec3 v1 = _mesh.Vertices[tr.v2];
                Vec3 v2 = _mesh.Vertices[tr.v3];

                double e1x = v1.x - v0.x, e1y = v1.y - v0.y, e1z = v1.z - v0.z;
                double e2x = v2.x - v0.x, e2y = v2.y - v0.y, e2z = v2.z - v0.z;

                // h = D x e2, where D = (ux, uy, 0)
                double hx = uy * e2z;
                double hy = -ux * e2z;
                double hz = ux * e2y - uy * e2x;

                double a = e1x * hx + e1y * hy + e1z * hz;
                if (Math.Abs(a) < 1e-10) continue;

                double f = 1.0 / a;
                double sx = _cx - v0.x, sy = _cy - v0.y, sz = pz - v0.z;
                double u = f * (sx * hx + sy * hy + sz * hz);
                if (u < -uvTol || u > 1.0 + uvTol) continue;

                double qx = sy * e1z - sz * e1y;
                double qy = sz * e1x - sx * e1z;
                double qz = sx * e1y - sy * e1x;
                double v = f * (ux * qx + uy * qy);
                if (v < -uvTol || (u + v) > 1.0 + uvTol) continue;

                double t = f * (e2x * qx + e2y * qy + e2z * qz);
                if (t > maxT) maxT = t;
            }
            return maxT;
        }
    }

    // ==========================================
    // PER-GLYPH 2D MESH SUBDIVISION (SUB-MICRON <= 0.15 MM CONFORMAL WRAP)
    // ==========================================
    // Earcut sonrası tüm uzun kenarları (hem sınır hem iç üçgenleri) çok geçişli
    // 1-to-4 bölme ile Dünya biriminde <= 0.15 mm altına indirger.
    // Kiriş sapmasını ~0.30 mm'den < 0.0005 mm (yarım mikron) seviyesine çeker.
    private static List<List<PointF>> BuildCleanGlyphContoursWpf(GraphicsPath gp, double effBoldnessLocal)
    {
        List<List<PointF>> result = new List<List<PointF>>();
        if (gp == null || gp.PointCount < 3) return result;

        PointF[] pts = gp.PathPoints;
        byte[] types = gp.PathTypes;
        if (pts == null || pts.Length < 3) return result;

        System.Windows.Media.StreamGeometry sg = new System.Windows.Media.StreamGeometry();
        sg.FillRule = System.Windows.Media.FillRule.Nonzero;
        using (var ctx = sg.Open())
        {
            bool inFig = false;
            for (int i = 0; i < pts.Length; i++)
            {
                byte t = (byte)(types[i] & 0x07);
                var wp = new System.Windows.Point(pts[i].X, pts[i].Y);
                if (t == 0)
                {
                    ctx.BeginFigure(wp, true, true);
                    inFig = true;
                }
                else if (inFig)
                {
                    ctx.LineTo(wp, true, false);
                }
            }
        }
        sg.Freeze();

        System.Windows.Media.PathGeometry united = System.Windows.Media.Geometry.Combine(
            sg,
            System.Windows.Media.Geometry.Empty,
            System.Windows.Media.GeometryCombineMode.Union,
            null, 0.05, System.Windows.Media.ToleranceType.Absolute);

        if (effBoldnessLocal > 1e-4)
        {
            var pen = new System.Windows.Media.Pen(System.Windows.Media.Brushes.Black, effBoldnessLocal * 2.0);
            pen.LineJoin = System.Windows.Media.PenLineJoin.Round;
            pen.StartLineCap = System.Windows.Media.PenLineCap.Round;
            pen.EndLineCap = System.Windows.Media.PenLineCap.Round;
            var widened = united.GetWidenedPathGeometry(pen, 0.05, System.Windows.Media.ToleranceType.Absolute);
            united = System.Windows.Media.Geometry.Combine(
                united, widened,
                System.Windows.Media.GeometryCombineMode.Union,
                null, 0.05, System.Windows.Media.ToleranceType.Absolute);
        }

        System.Windows.Media.PathGeometry flat = united.GetFlattenedPathGeometry(0.10, System.Windows.Media.ToleranceType.Absolute);
        foreach (var fig in flat.Figures)
        {
            List<PointF> poly = new List<PointF>();
            poly.Add(new PointF((float)fig.StartPoint.X, (float)fig.StartPoint.Y));
            foreach (var seg in fig.Segments)
            {
                var pls = seg as System.Windows.Media.PolyLineSegment;
                if (pls != null)
                {
                    foreach (var p in pls.Points) poly.Add(new PointF((float)p.X, (float)p.Y));
                }
                else
                {
                    var ls = seg as System.Windows.Media.LineSegment;
                    if (ls != null) poly.Add(new PointF((float)ls.Point.X, (float)ls.Point.Y));
                }
            }

            List<PointF> cleaned = CleanContour(poly, 1e-3);
            if (cleaned.Count >= 3 && Math.Abs(PolygonArea(cleaned)) > 1e-4)
                result.Add(cleaned);
        }
        return result;
    }

    private static void SplitNonManifoldPinchVertices2D(List<double> pts, List<int> tris)
    {
        if (pts == null || tris == null || tris.Count < 3) return;
        int nVerts = pts.Count / 2;
        int nTris = tris.Count / 3;
        List<int>[] inc = new List<int>[nVerts];
        for (int ti = 0; ti < nTris; ti++)
        {
            for (int k = 0; k < 3; k++)
            {
                int v = tris[ti * 3 + k];
                if (inc[v] == null) inc[v] = new List<int>();
                inc[v].Add(ti);
            }
        }

        for (int v = 0; v < nVerts; v++)
        {
            var vTris = inc[v];
            if (vTris == null || vTris.Count <= 1) continue;

            // v çevresindeki üçgenleri ortak kenar üzerinden bileşenlere (fan) ayır
            bool[] visited = new bool[vTris.Count];
            List<List<int>> comps = new List<List<int>>();

            for (int start = 0; start < vTris.Count; start++)
            {
                if (visited[start]) continue;
                List<int> comp = new List<int>();
                Queue<int> q = new Queue<int>();
                visited[start] = true;
                q.Enqueue(start);

                while (q.Count > 0)
                {
                    int curIdx = q.Dequeue();
                    int tA = vTris[curIdx];
                    comp.Add(tA);

                    int a1 = tris[tA * 3], a2 = tris[tA * 3 + 1], a3 = tris[tA * 3 + 2];
                    int oa1 = (a1 == v) ? a2 : (a2 == v ? a3 : a1);
                    int oa2 = (a1 == v) ? a3 : (a2 == v ? a1 : a2);

                    for (int nxt = 0; nxt < vTris.Count; nxt++)
                    {
                        if (visited[nxt]) continue;
                        int tB = vTris[nxt];
                        int b1 = tris[tB * 3], b2 = tris[tB * 3 + 1], b3 = tris[tB * 3 + 2];
                        if (b1 == oa1 || b2 == oa1 || b3 == oa1 || b1 == oa2 || b2 == oa2 || b3 == oa2)
                        {
                            visited[nxt] = true;
                            q.Enqueue(nxt);
                        }
                    }
                }
                comps.Add(comp);
            }

            if (comps.Count <= 1) continue;

            double vx = pts[v * 2], vy = pts[v * 2 + 1];
            for (int cIdx = 1; cIdx < comps.Count; cIdx++)
            {
                double sx = 0, sy = 0;
                int cnt = 0;
                foreach (int ti in comps[cIdx])
                {
                    for (int k = 0; k < 3; k++)
                    {
                        int u = tris[ti * 3 + k];
                        if (u != v) { sx += pts[u * 2]; sy += pts[u * 2 + 1]; cnt++; }
                    }
                }
                double dx = cnt > 0 ? (sx / cnt - vx) : 1.0;
                double dy = cnt > 0 ? (sy / cnt - vy) : 0.0;
                double dlen = Math.Sqrt(dx * dx + dy * dy);
                double nx = vx + (dlen > 1e-9 ? (dx / dlen) * 1e-3 : 1e-3);
                double ny = vy + (dlen > 1e-9 ? (dy / dlen) * 1e-3 : 0.0);

                int newV = pts.Count / 2;
                pts.Add(nx);
                pts.Add(ny);
                foreach (int ti in comps[cIdx])
                {
                    for (int k = 0; k < 3; k++)
                        if (tris[ti * 3 + k] == v) tris[ti * 3 + k] = newV;
                }
            }
        }
    }

    private static void SubdivideGlyphMesh2D(List<double> pts, ref List<int> tris,
                                             double maxEdgeULocal, double maxEdgeVLocal)
    {
        if (pts == null || tris == null || pts.Count < 6 || tris.Count < 3) return;
        if (maxEdgeULocal < 1e-5) maxEdgeULocal = 1.0;
        if (maxEdgeVLocal < 1e-5) maxEdgeVLocal = 1.0;

        int maxVertsBudget = Math.Max(4000, (pts.Count / 2) + 8000);
        const int maxPasses = 12;

        for (int pass = 0; pass < maxPasses; pass++)
        {
            if ((pts.Count / 2) >= maxVertsBudget) break;

            Dictionary<long, int> edgeMid = new Dictionary<long, int>();
            Func<int, int, long> edgeKey = (i1, i2) =>
            {
                int lo = i1 < i2 ? i1 : i2;
                int hi = i1 < i2 ? i2 : i1;
                return ((long)lo << 32) | (uint)hi;
            };

            // 1. Bu geçişte maxEdge sınırını aşan TÜM kenarları bul ve böl
            for (int t = 0; t < tris.Count; t += 3)
            {
                for (int k = 0; k < 3; k++)
                {
                    int i1 = tris[t + k];
                    int i2 = tris[t + ((k + 1) % 3)];
                    long ek = edgeKey(i1, i2);
                    if (edgeMid.ContainsKey(ek)) continue;

                    double dx = (pts[i2 * 2] - pts[i1 * 2]) / maxEdgeULocal;
                    double dy = (pts[i2 * 2 + 1] - pts[i1 * 2 + 1]) / maxEdgeVLocal;
                    if ((dx * dx + dy * dy) > 1.0)
                    {
                        double mx = (pts[i1 * 2] + pts[i2 * 2]) * 0.5;
                        double my = (pts[i1 * 2 + 1] + pts[i2 * 2 + 1]) * 0.5;
                        int midIdx = pts.Count / 2;
                        pts.Add(mx);
                        pts.Add(my);
                        edgeMid[ek] = midIdx;
                        if ((pts.Count / 2) >= maxVertsBudget) break;
                    }
                }
                if ((pts.Count / 2) >= maxVertsBudget) break;
            }

            if (edgeMid.Count == 0) break;

            // 2. Her üçgeni 0, 1, 2 veya 3 bölünmüş kenara göre yön koruyarak (CCW) alt üçgenlere ayır
            List<int> newTris = new List<int>(tris.Count * 2);
            for (int t = 0; t < tris.Count; t += 3)
            {
                int a = tris[t], b = tris[t + 1], c = tris[t + 2];
                int mAB, mBC, mCA;
                bool sAB = edgeMid.TryGetValue(edgeKey(a, b), out mAB);
                bool sBC = edgeMid.TryGetValue(edgeKey(b, c), out mBC);
                bool sCA = edgeMid.TryGetValue(edgeKey(c, a), out mCA);

                int splitCount = (sAB ? 1 : 0) + (sBC ? 1 : 0) + (sCA ? 1 : 0);
                if (splitCount == 0)
                {
                    newTris.Add(a); newTris.Add(b); newTris.Add(c);
                }
                else if (splitCount == 3)
                {
                    newTris.Add(a);   newTris.Add(mAB); newTris.Add(mCA);
                    newTris.Add(mAB); newTris.Add(b);   newTris.Add(mBC);
                    newTris.Add(mCA); newTris.Add(mBC); newTris.Add(c);
                    newTris.Add(mAB); newTris.Add(mBC); newTris.Add(mCA);
                }
                else if (splitCount == 1)
                {
                    // Bölünen kenarı (u, v), karşı tepeyi w olarak hizala: (w, u, v) CCW
                    int w, u, v, mUV;
                    if (sAB) { w = c; u = a; v = b; mUV = mAB; }
                    else if (sBC) { w = a; u = b; v = c; mUV = mBC; }
                    else { w = b; u = c; v = a; mUV = mCA; }

                    newTris.Add(w); newTris.Add(u);   newTris.Add(mUV);
                    newTris.Add(w); newTris.Add(mUV); newTris.Add(v);
                }
                else // splitCount == 2
                {
                    // Bölünmeyen kenar (u, v), ortak tepe w olsun: (w, u, v) CCW; mWU ve mWV bölünmüş
                    int w, u, v, mWU, mWV;
                    if (!sBC) { w = a; u = b; v = c; mWU = mAB; mWV = mCA; }
                    else if (!sCA) { w = b; u = c; v = a; mWU = mBC; mWV = mAB; }
                    else { w = c; u = a; v = b; mWU = mCA; mWV = mBC; }

                    newTris.Add(w); newTris.Add(mWU); newTris.Add(mWV);
                    // Dörtgen (mWU, u, v, mWV) için kısa köşegeni seç
                    double d1x = (pts[v * 2] - pts[mWU * 2]) / maxEdgeULocal;
                    double d1y = (pts[v * 2 + 1] - pts[mWU * 2 + 1]) / maxEdgeVLocal;
                    double d2x = (pts[mWV * 2] - pts[u * 2]) / maxEdgeULocal;
                    double d2y = (pts[mWV * 2 + 1] - pts[u * 2 + 1]) / maxEdgeVLocal;
                    if ((d1x * d1x + d1y * d1y) <= (d2x * d2x + d2y * d2y))
                    {
                        newTris.Add(mWU); newTris.Add(u); newTris.Add(v);
                        newTris.Add(mWU); newTris.Add(v); newTris.Add(mWV);
                    }
                    else
                    {
                        newTris.Add(mWU); newTris.Add(u); newTris.Add(mWV);
                        newTris.Add(u);   newTris.Add(v); newTris.Add(mWV);
                    }
                }
            }
            tris = newTris;
        }
    }

    private static void TriangulateGlyphContours(List<PointF> outer, List<List<PointF>> holes,
        out List<double> data, out List<int> tri)
    {
        if (PolygonArea(outer) < 0) outer.Reverse();

        data = new List<double>();
        List<int> holeIndices = new List<int>();

        foreach (var p in outer) { data.Add(p.X); data.Add(p.Y); }
        foreach (var h in holes)
        {
            if (PolygonArea(h) > 0) h.Reverse();
            holeIndices.Add(data.Count / 2);
            foreach (var p in h) { data.Add(p.X); data.Add(p.Y); }
        }

        List<int> rawTri = Earcut.Triangulate(data, holeIndices, 2);
        tri = new List<int>(rawTri.Count);

        // Tüm üçgenleri doğrula: sıfır alanlı / dejenere üçgenleri at ve kesin CCW (area > 0) yap
        for (int i = 0; i < rawTri.Count; i += 3)
        {
            int i0 = rawTri[i], i1 = rawTri[i + 1], i2 = rawTri[i + 2];
            if (i0 == i1 || i1 == i2 || i2 == i0) continue;
            double x0 = data[i0 * 2], y0 = data[i0 * 2 + 1];
            double x1 = data[i1 * 2], y1 = data[i1 * 2 + 1];
            double x2 = data[i2 * 2], y2 = data[i2 * 2 + 1];
            double area2 = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
            if (Math.Abs(area2) < 1e-10) continue;
            if (area2 > 0)
            {
                tri.Add(i0); tri.Add(i1); tri.Add(i2);
            }
            else
            {
                tri.Add(i0); tri.Add(i2); tri.Add(i1);
            }
        }
    }

    public static SolidMesh GenerateDistributedCylindricalText(string text, string fontName, double targetHeightWorld,
        double centerZLocal, double centerAngleDeg, double embedDepth, double embossHeight, double zScale,
        double charGapWorld, double targetSpanDeg = 0, double boldnessWorld = 0.0, double maxAllowedSpanDeg = 252.0,
        double verticalStretch = 1.0, bool isCircularRing = false, double circularRadiusLocal = 4.50,
        double rootChamferWorld = 0.0, bool useTurkishFix = true, RingSurfaceSampler surfaceSampler = null)
    {
        SolidMesh mesh = new SolidMesh();
        if (string.IsNullOrEmpty(text)) return mesh;

        if (string.IsNullOrEmpty(fontName)) fontName = "Trebuchet MS";
        FontFamily ff = FontManager.GetFontFamily(fontName);

        int style = (int)FontStyle.Bold;
        if (!ff.IsStyleAvailable(FontStyle.Bold)) style = (int)FontStyle.Regular;
        float emSize = 100.0f;
        StringFormat sf = StringFormat.GenericTypographic;

        GraphicsPath refGp = new GraphicsPath();
        refGp.AddString("H0", ff, style, emSize, new PointF(0, 0), sf);
        refGp.Flatten(null, 0.20f);
        RectangleF refBounds = refGp.GetBounds();
        double refGlyphH = refBounds.Height > 0 ? refBounds.Height : 70.0;

        // 1. Karakterleri topla ve gerçek sınır kutusunu hesapla
        List<char> glyphChars = new List<char>();
        List<GraphicsPath> charGps = new List<GraphicsPath>();
        List<RectangleF> charBounds = new List<RectangleF>();

        float textMinY = float.MaxValue;
        float textMaxY = float.MinValue;

        for (int i = 0; i < text.Length; i++)
        {
            char c = text[i];
            if (c == ' ') continue;

            GraphicsPath cGp = CreateCharPath(c, ff, style, emSize, sf, useTurkishFix);
            cGp.Flatten(null, 0.20f);
            RectangleF cb = cGp.GetBounds();
            if (cb.Width <= 0 || cb.Height <= 0) continue;

            glyphChars.Add(c);
            charGps.Add(cGp);
            charBounds.Add(cb);

            if (cb.Y < textMinY) textMinY = cb.Y;
            if (cb.Y + cb.Height > textMaxY) textMaxY = cb.Y + cb.Height;
        }

        if (glyphChars.Count == 0) return mesh;

        double textCenterY = (textMinY + textMaxY) * 0.5;
        double textSpanH = (textMaxY > textMinY) ? (textMaxY - textMinY) : refGlyphH;

        double requestedH = targetHeightWorld * verticalStretch;
        bool isSingleCentered = Math.Abs(centerZLocal) < 0.5;
        double maxAllowedPhysicalH = isSingleCentered ? 7.60 : 4.00;
        double spanToCapRatio = textSpanH / refGlyphH;
        if (spanToCapRatio > 1.0 && (requestedH * spanToCapRatio) > maxAllowedPhysicalH)
            requestedH = maxAllowedPhysicalH / spanToCapRatio;
        else if (requestedH > maxAllowedPhysicalH)
            requestedH = maxAllowedPhysicalH;

        double scaleFactorY = (requestedH / zScale) / refGlyphH;
        double scaleFactorWorld = targetHeightWorld / refGlyphH;
        double boldnessLocal = (boldnessWorld > 0) ? (boldnessWorld / scaleFactorWorld) : 0;

        // 1.5 Fontun orijinal doğal karakter sınırlarını ölç
        List<RectangleF> naturalBounds;
        using (Bitmap bmpDummy = new Bitmap(1, 1))
        using (Graphics gDummy = Graphics.FromImage(bmpDummy))
        using (Font fFont = new Font(ff, emSize, (FontStyle)style, GraphicsUnit.Pixel))
        {
            naturalBounds = GetNaturalCharBounds(gDummy, text, fFont, StringFormat.GenericTypographic);
        }

        List<double> charOriginsX = new List<double>();
        double extraGapLocal = (charGapWorld != 0) ? (charGapWorld / scaleFactorWorld) : 0;
        double minClearanceLocal = 0.65 / scaleFactorWorld;
        double prevRightWithBoldness = double.MinValue;
        double minLeftLocal = double.MaxValue;
        double maxRightLocal = double.MinValue;

        int gIdx = 0;
        for (int i = 0; i < text.Length; i++)
        {
            if (text[i] == ' ') continue;
            if (i >= naturalBounds.Count || gIdx >= charBounds.Count) break;

            RectangleF nb = naturalBounds[i];
            RectangleF cb = charBounds[gIdx];
            double originX = nb.X + (gIdx * extraGapLocal);

            if (gIdx > 0 && prevRightWithBoldness > double.MinValue)
            {
                double reqOriginX = prevRightWithBoldness + minClearanceLocal - (cb.Left - boldnessLocal);
                if (originX < reqOriginX) originX = reqOriginX;
            }

            charOriginsX.Add(originX);
            double cLeft = originX + cb.Left - boldnessLocal;
            double cRight = originX + cb.Right + (boldnessLocal * 2.0);
            prevRightWithBoldness = originX + cb.Right + boldnessLocal;

            if (cLeft < minLeftLocal) minLeftLocal = cLeft;
            if (cRight > maxRightLocal) maxRightLocal = cRight;
            gIdx++;
        }

        if (charOriginsX.Count == 0) return mesh;

        double totalWidthLocal = maxRightLocal - minLeftLocal;
        double localTextCenterX = (minLeftLocal + maxRightLocal) * 0.5;

        double ringCenterX = 0.0;
        double ringCenterY = isCircularRing ? 0.0 : -0.2754;
        double baseRadius = isCircularRing ? circularRadiusLocal : 6.0369;

        double rCenter = isCircularRing ? (circularRadiusLocal * 1.25) : baseRadius;
        double maxSafeWidthWorld = rCenter * (maxAllowedSpanDeg * Math.PI / 180.0);

        double textTotalWidthWorld = totalWidthLocal * scaleFactorWorld;
        double fitRatio = 1.0;
        if (textTotalWidthWorld > maxSafeWidthWorld && maxSafeWidthWorld > 0)
        {
            fitRatio = maxSafeWidthWorld / textTotalWidthWorld;
            textTotalWidthWorld = maxSafeWidthWorld;
        }
        else if (textTotalWidthWorld < maxSafeWidthWorld && textTotalWidthWorld > 0 && charOriginsX.Count > 1 && charGapWorld <= 0.05)
        {
            double remainingWorld = maxSafeWidthWorld - textTotalWidthWorld;
            double extraPerGapWorld = remainingWorld / (charOriginsX.Count - 1);
            double maxAutoGapWorld = (charOriginsX.Count >= 4) ? 2.60 : 1.50;
            if (extraPerGapWorld > maxAutoGapWorld) extraPerGapWorld = maxAutoGapWorld;
            if (extraPerGapWorld > 0.05)
            {
                double extraPerGapLocal = extraPerGapWorld / scaleFactorWorld;
                for (int k = 1; k < charOriginsX.Count; k++)
                    charOriginsX[k] += k * extraPerGapLocal;
                minLeftLocal = double.MaxValue;
                maxRightLocal = double.MinValue;
                for (int k = 0; k < charOriginsX.Count; k++)
                {
                    double cLeft = charOriginsX[k] + charBounds[k].Left - boldnessLocal;
                    double cRight = charOriginsX[k] + charBounds[k].Right + (boldnessLocal * 2.0);
                    if (cLeft < minLeftLocal) minLeftLocal = cLeft;
                    if (cRight > maxRightLocal) maxRightLocal = cRight;
                }
                totalWidthLocal = maxRightLocal - minLeftLocal;
                localTextCenterX = (minLeftLocal + maxRightLocal) * 0.5;
                textTotalWidthWorld = totalWidthLocal * scaleFactorWorld;
                if (textTotalWidthWorld > maxSafeWidthWorld)
                    fitRatio = maxSafeWidthWorld / textTotalWidthWorld;
            }
        }

        double centerAngleRad = centerAngleDeg * Math.PI / 180.0;
        double baseR = baseRadius;
        double xyScale = isCircularRing ? 1.25 : 1.0;

        double effScaleXWorld = (scaleFactorWorld / xyScale) * fitRatio;
        double effScaleYWorld = scaleFactorY * zScale;
        double maxEdgeULocal = 0.20 / Math.Max(effScaleXWorld * xyScale, 1e-6);
        double maxEdgeVLocal = 2.00 / Math.Max(effScaleYWorld, 1e-6);

        for (int cIdx = 0; cIdx < glyphChars.Count; cIdx++)
        {
            GraphicsPath gp = charGps[cIdx];
            double charOriginX = charOriginsX[cIdx];

            double minNozzleBoldnessWorld = 0.08;
            double effBoldnessWorld = Math.Max(boldnessWorld, minNozzleBoldnessWorld);
            double effBoldness = effBoldnessWorld / scaleFactorWorld;

            List<List<PointF>> allContours = BuildCleanGlyphContoursWpf(gp, effBoldness);
            if (allContours.Count == 0) continue;

            List<int> parentOuter = new List<int>();
            for (int i = 0; i < allContours.Count; i++)
            {
                int pIdx = -1;
                double areaI = Math.Abs(PolygonArea(allContours[i]));
                for (int j = 0; j < allContours.Count; j++)
                {
                    if (i == j) continue;
                    double areaJ = Math.Abs(PolygonArea(allContours[j]));
                    if (areaI >= areaJ) continue;
                    int insideCount = 0;
                    for (int k = 0; k < allContours[i].Count; k++)
                    {
                        if (PointInPolygon(allContours[i][k], allContours[j])) insideCount++;
                    }
                    if (insideCount > allContours[i].Count * 0.5)
                    {
                        if (pIdx == -1 || areaJ < Math.Abs(PolygonArea(allContours[pIdx])))
                            pIdx = j;
                    }
                }
                parentOuter.Add(pIdx);
            }

            List<int> outerIndices = new List<int>();
            Dictionary<int, List<List<PointF>>> outerHoles = new Dictionary<int, List<List<PointF>>>();
            int[] depth = new int[allContours.Count];
            for (int i = 0; i < allContours.Count; i++)
            {
                int d = 0, cur = parentOuter[i];
                while (cur != -1) { d++; cur = parentOuter[cur]; }
                depth[i] = d;
                if ((d % 2) == 0)
                {
                    outerIndices.Add(i);
                    outerHoles[i] = new List<List<PointF>>();
                }
            }
            for (int i = 0; i < allContours.Count; i++)
            {
                if ((depth[i] % 2) == 1 && parentOuter[i] != -1 && outerHoles.ContainsKey(parentOuter[i]))
                    outerHoles[parentOuter[i]].Add(allContours[i]);
            }

            foreach (int oIdx in outerIndices)
            {
                List<PointF> topOuter = new List<PointF>(allContours[oIdx]);
                List<List<PointF>> topHoles = new List<List<PointF>>();
                foreach (var h in outerHoles[oIdx])
                {
                    if (h.Count > 2) topHoles.Add(new List<PointF>(h));
                }
                if (topOuter.Count < 3) continue;

                if (PolygonArea(topOuter) < 0) topOuter.Reverse();
                foreach (var h in topHoles)
                {
                    if (PolygonArea(h) > 0) h.Reverse();
                }

                List<double> topData;
                List<int> topTri;
                TriangulateGlyphContours(topOuter, topHoles, out topData, out topTri);
                if (topTri.Count < 3) continue;

                // PER-GLYPH CURVED-U SUBDIVISION (0.20mm kavis, Z boyunca bloat yok)
                SubdivideGlyphMesh2D(topData, ref topTri, maxEdgeULocal, maxEdgeVLocal);

                // Yalnızca gerçek üçgenlerde kullanılan vertex'leri sıkıştır (0 Sahipsiz V=1 Vertex!)
                int rawPtCount = topData.Count / 2;
                int[] remap = new int[rawPtCount];
                for (int k = 0; k < rawPtCount; k++) remap[k] = -1;
                List<double> compactPts = new List<double>();
                for (int t = 0; t < topTri.Count; t++)
                {
                    int oldIdx = topTri[t];
                    if (remap[oldIdx] == -1)
                    {
                        remap[oldIdx] = compactPts.Count / 2;
                        compactPts.Add(topData[oldIdx * 2]);
                        compactPts.Add(topData[oldIdx * 2 + 1]);
                    }
                    topTri[t] = remap[oldIdx];
                }

                // Olası tek-tepe (pinch) temaslarını ayır (garantili basit manifold sınır halkaları)
                SplitNonManifoldPinchVertices2D(compactPts, topTri);

                // %100 Garantili Sınır Kenarı Çıkarma (Half-Edge: (u->v) olup (v->u) olmayanlar)
                HashSet<long> dirEdgeSet = new HashSet<long>();
                for (int t = 0; t < topTri.Count; t += 3)
                {
                    int a = topTri[t], b = topTri[t + 1], c = topTri[t + 2];
                    dirEdgeSet.Add(((long)a << 32) | (uint)b);
                    dirEdgeSet.Add(((long)b << 32) | (uint)c);
                    dirEdgeSet.Add(((long)c << 32) | (uint)a);
                }
                List<int> boundaryEdges = new List<int>();
                foreach (long ek in dirEdgeSet)
                {
                    int u = (int)(ek >> 32);
                    int v = (int)(ek & 0xFFFFFFFF);
                    long rev = ((long)v << 32) | (uint)u;
                    if (!dirEdgeSet.Contains(rev))
                    {
                        boundaryEdges.Add(u);
                        boundaryEdges.Add(v);
                    }
                }

                // Her kontur adasını izole bir SolidMesh'te kurup SignedVolume > 0 doğrula
                SolidMesh glyphSolid = new SolidMesh();
                int totalPts = compactPts.Count / 2;

                // Ön Yüz (Front: rSurf + emboss) ve Arka Kök (Back: rSurf - embed)
                // USE SURFACE: Her (theta, z) noktası için gerçek şablon yüzeyi R_actual ölçülür!
                for (int k = 0; k < totalPts; k++)
                {
                    double px = compactPts[k * 2];
                    double py = compactPts[k * 2 + 1];
                    double u = ((charOriginX + px) - localTextCenterX) * effScaleXWorld;
                    double v = -(py - textCenterY) * scaleFactorY;

                    double theta = centerAngleRad + u / baseR;
                    double z = centerZLocal + v;
                    double rSurf = (surfaceSampler != null)
                        ? surfaceSampler.SampleRadius(theta, z, baseRadius)
                        : baseRadius;

                    double rFront = rSurf + (embossHeight / xyScale);
                    glyphSolid.AddVertex(ringCenterX + rFront * Math.Cos(theta), ringCenterY + rFront * Math.Sin(theta), z);
                }

                int backOffsetLocal = glyphSolid.Vertices.Count;
                for (int k = 0; k < totalPts; k++)
                {
                    double px = compactPts[k * 2];
                    double py = compactPts[k * 2 + 1];
                    double u = ((charOriginX + px) - localTextCenterX) * effScaleXWorld;
                    double v = -(py - textCenterY) * scaleFactorY;

                    double theta = centerAngleRad + u / baseR;
                    double z = centerZLocal + v;
                    double rSurf = (surfaceSampler != null)
                        ? surfaceSampler.SampleRadius(theta, z, baseRadius)
                        : baseRadius;

                    double rBack = rSurf - (embedDepth / xyScale);
                    glyphSolid.AddVertex(ringCenterX + rBack * Math.Cos(theta), ringCenterY + rBack * Math.Sin(theta), z);
                }

                // Ön kapak (dışa bakan)
                for (int t = 0; t < topTri.Count; t += 3)
                    glyphSolid.AddTriangle(topTri[t], topTri[t + 2], topTri[t + 1]);

                // Arka kapak (gövde içine gömülü)
                for (int t = 0; t < topTri.Count; t += 3)
                    glyphSolid.AddTriangle(backOffsetLocal + topTri[t], backOffsetLocal + topTri[t + 1], backOffsetLocal + topTri[t + 2]);

                // Yan duvarlar (garantili su geçirmez 1-to-1 half-edge bağlantı)
                for (int be = 0; be < boundaryEdges.Count; be += 2)
                {
                    int u = boundaryEdges[be];
                    int v = boundaryEdges[be + 1];
                    int f1 = u, f2 = v;
                    int b1 = backOffsetLocal + u, b2 = backOffsetLocal + v;
                    glyphSolid.AddTriangle(f1, f2, b1);
                    glyphSolid.AddTriangle(f2, b2, b1);
                }

                // Hacim negatif çıkarsa normalleri dışarı çevir
                bool flipWinding = glyphSolid.ComputeSignedVolume() < 0;
                int baseV = mesh.Vertices.Count;
                foreach (var gv in glyphSolid.Vertices)
                    mesh.AddVertex(gv.x, gv.y, gv.z);
                foreach (var gt in glyphSolid.Triangles)
                {
                    if (flipWinding)
                        mesh.AddTriangle(baseV + gt.v1, baseV + gt.v3, baseV + gt.v2);
                    else
                        mesh.AddTriangle(baseV + gt.v1, baseV + gt.v2, baseV + gt.v3);
                }
            }
        }

        double finalSpanDeg = (textTotalWidthWorld / (baseR * xyScale)) * 180.0 / Math.PI;
        double mzMin = double.MaxValue, mzMax = double.MinValue;
        foreach (var v in mesh.Vertices) { if (v.z < mzMin) mzMin = v.z; if (v.z > mzMax) mzMax = v.z; }
        Console.WriteLine("Generated text '{0}': {1} verts, {2} tris (Font={3}, Gap={4:F2}mm, Boldness={5:F2}mm, Arc={6:F1} deg, SignedVol={7:F2}, Z=[{8:F3}..{9:F3}], Center={10:F3})",
            text, mesh.Vertices.Count, mesh.Triangles.Count, fontName, charGapWorld, boldnessWorld, finalSpanDeg, mesh.ComputeSignedVolume(), mzMin, mzMax, (mzMin + mzMax) * 0.5);
        return mesh;
    }

    public static void SplitAndCapRing(string modelXmlContent, out SolidMesh lowerMesh, out SolidMesh upperMesh, double splitZ = 0.0)
    {
        SolidMesh orig = ParseMeshFromModelXml(modelXmlContent);
        SplitAndCapRing(orig, out lowerMesh, out upperMesh, splitZ);
    }

    public static void SplitAndCapRing(SolidMesh orig, out SolidMesh lowerMesh, out SolidMesh upperMesh, double splitZ = 0.0)
    {
        List<Vec3> origVerts = orig.Vertices;
        List<Tri> origTris = orig.Triangles;

        lowerMesh = new SolidMesh();
        upperMesh = new SolidMesh();

        // Köşe tekilleştirme (1e-6 çözünürlük ile Exact Index haritalama)
        Dictionary<string, int> lowerVertMap = new Dictionary<string, int>();
        Dictionary<string, int> upperVertMap = new Dictionary<string, int>();

        Func<SolidMesh, Dictionary<string, int>, Vec3, int> getOrAdd = (mesh, map, v) =>
        {
            string key = string.Format(CultureInfo.InvariantCulture, "{0:F6}_{1:F6}_{2:F6}", v.x, v.y, v.z);
            int idx;
            if (map.TryGetValue(key, out idx)) return idx;
            idx = mesh.AddVertex(v.x, v.y, v.z);
            map[key] = idx;
            return idx;
        };

        // Exact edge-key tabanlı kesim önbelleği (BOS_KLIPS_9MM vs PETG tüm şablonlarda sıfır çakışma!)
        Dictionary<long, Vec3> edgeCutPt = new Dictionary<long, Vec3>();
        List<long> cutSegKeys = new List<long>(); // (edgeKeyA, edgeKeyB) çiftleri halinde
        Dictionary<long, Vec3> ptByEdgeKey = new Dictionary<long, Vec3>();

        Func<int, int, Vec3> getEdgeCut = (idx1, idx2) =>
        {
            int lo = idx1 < idx2 ? idx1 : idx2;
            int hi = idx1 < idx2 ? idx2 : idx1;
            long ek = ((long)lo << 32) | (uint)hi;
            Vec3 hit;
            if (edgeCutPt.TryGetValue(ek, out hit)) return hit;

            Vec3 p1 = origVerts[lo];
            Vec3 p2 = origVerts[hi];
            double dz = p2.z - p1.z;
            double t = Math.Abs(dz) < 1e-12 ? 0.5 : (splitZ - p1.z) / dz;
            t = Math.Max(0.0, Math.Min(1.0, t));
            hit = new Vec3(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y), splitZ);
            edgeCutPt[ek] = hit;
            ptByEdgeKey[ek] = hit;
            return hit;
        };

        Func<int, int, long> makeEdgeKey = (idx1, idx2) =>
        {
            int lo = idx1 < idx2 ? idx1 : idx2;
            int hi = idx1 < idx2 ? idx2 : idx1;
            return ((long)lo << 32) | (uint)hi;
        };

        const double eps = 1e-7;

        foreach (var t in origTris)
        {
            int[] idxs = new int[] { t.v1, t.v2, t.v3 };
            Vec3 v1 = origVerts[t.v1], v2 = origVerts[t.v2], v3 = origVerts[t.v3];
            double z1 = v1.z - splitZ, z2 = v2.z - splitZ, z3 = v3.z - splitZ;

            bool allLower = z1 <= eps && z2 <= eps && z3 <= eps;
            bool allUpper = z1 >= -eps && z2 >= -eps && z3 >= -eps;

            if (allLower && !allUpper)
            {
                int i1 = getOrAdd(lowerMesh, lowerVertMap, v1);
                int i2 = getOrAdd(lowerMesh, lowerVertMap, v2);
                int i3 = getOrAdd(lowerMesh, lowerVertMap, v3);
                lowerMesh.AddTriangle(i1, i2, i3);
            }
            else if (allUpper && !allLower)
            {
                int i1 = getOrAdd(upperMesh, upperVertMap, v1);
                int i2 = getOrAdd(upperMesh, upperVertMap, v2);
                int i3 = getOrAdd(upperMesh, upperVertMap, v3);
                upperMesh.AddTriangle(i1, i2, i3);
            }
            else if (!(allLower && allUpper))
            {
                int[] side = new int[3];
                for (int i = 0; i < 3; i++) side[i] = (origVerts[idxs[i]].z - splitZ) > 0 ? 1 : -1;

                int solo = 0;
                if (side[0] != side[1] && side[0] != side[2]) solo = 0;
                else if (side[1] != side[0] && side[1] != side[2]) solo = 1;
                else solo = 2;

                int ia_idx = idxs[solo];
                int ib_idx = idxs[(solo + 1) % 3];
                int ic_idx = idxs[(solo + 2) % 3];

                Vec3 a = origVerts[ia_idx];
                Vec3 b = origVerts[ib_idx];
                Vec3 c = origVerts[ic_idx];

                Vec3 i_ab = getEdgeCut(ia_idx, ib_idx);
                Vec3 i_ac = getEdgeCut(ia_idx, ic_idx);
                long ek_ab = makeEdgeKey(ia_idx, ib_idx);
                long ek_ac = makeEdgeKey(ia_idx, ic_idx);

                if (side[solo] > 0)
                {
                    int ua = getOrAdd(upperMesh, upperVertMap, a);
                    int uab = getOrAdd(upperMesh, upperVertMap, i_ab);
                    int uac = getOrAdd(upperMesh, upperVertMap, i_ac);
                    upperMesh.AddTriangle(ua, uab, uac);

                    int lib = getOrAdd(lowerMesh, lowerVertMap, i_ab);
                    int lb = getOrAdd(lowerMesh, lowerVertMap, b);
                    int lc = getOrAdd(lowerMesh, lowerVertMap, c);
                    int lic = getOrAdd(lowerMesh, lowerVertMap, i_ac);
                    lowerMesh.AddTriangle(lib, lb, lc);
                    lowerMesh.AddTriangle(lib, lc, lic);

                    cutSegKeys.Add(ek_ac);
                    cutSegKeys.Add(ek_ab);
                }
                else
                {
                    int la = getOrAdd(lowerMesh, lowerVertMap, a);
                    int lab = getOrAdd(lowerMesh, lowerVertMap, i_ab);
                    int lac = getOrAdd(lowerMesh, lowerVertMap, i_ac);
                    lowerMesh.AddTriangle(la, lab, lac);

                    int uib = getOrAdd(upperMesh, upperVertMap, i_ab);
                    int ub = getOrAdd(upperMesh, upperVertMap, b);
                    int uc = getOrAdd(upperMesh, upperVertMap, c);
                    int uic = getOrAdd(upperMesh, upperVertMap, i_ac);
                    upperMesh.AddTriangle(uib, ub, uc);
                    upperMesh.AddTriangle(uib, uc, uic);

                    cutSegKeys.Add(ek_ab);
                    cutSegKeys.Add(ek_ac);
                }
            }
        }

        // Kesim segmentlerini exact edgeKey ile kapalı halkalara (loops) bağla
        Dictionary<long, long> nextEdgeKey = new Dictionary<long, long>();
        for (int s = 0; s < cutSegKeys.Count; s += 2)
        {
            long eA = cutSegKeys[s];
            long eB = cutSegKeys[s + 1];
            nextEdgeKey[eA] = eB;
        }

        HashSet<long> visitedKeys = new HashSet<long>();
        List<List<Vec3>> loops = new List<List<Vec3>>();

        foreach (long startKey in nextEdgeKey.Keys)
        {
            if (visitedKeys.Contains(startKey)) continue;
            List<Vec3> loop = new List<Vec3>();
            long curr = startKey;
            while (!visitedKeys.Contains(curr))
            {
                visitedKeys.Add(curr);
                Vec3 pt = ptByEdgeKey[curr];
                if (loop.Count == 0 || Math.Abs(pt.x - loop[loop.Count - 1].x) > 1e-7 || Math.Abs(pt.y - loop[loop.Count - 1].y) > 1e-7)
                    loop.Add(pt);
                long nxt;
                if (!nextEdgeKey.TryGetValue(curr, out nxt)) break;
                curr = nxt;
                if (curr == startKey) break;
            }
            if (loop.Count > 2)
            {
                Vec3 f = loop[0], l = loop[loop.Count - 1];
                if (Math.Abs(f.x - l.x) <= 1e-7 && Math.Abs(f.y - l.y) <= 1e-7) loop.RemoveAt(loop.Count - 1);
                if (loop.Count > 2) loops.Add(loop);
            }
        }

        Func<List<Vec3>, double> loopArea = l =>
        {
            double a = 0;
            int n = l.Count;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                a += l[i].x * l[j].y - l[j].x * l[i].y;
            }
            return a * 0.5;
        };

        // Gerçek Outer vs Hole Hiyerarşisi (PointInPolygon ile)
        List<List<PointF>> loops2D = new List<List<PointF>>(loops.Count);
        foreach (var l in loops)
        {
            List<PointF> p2 = new List<PointF>(l.Count);
            foreach (var v in l) p2.Add(new PointF((float)v.x, (float)v.y));
            loops2D.Add(p2);
        }

        int[] parentOuter = new int[loops.Count];
        for (int i = 0; i < loops.Count; i++) parentOuter[i] = -1;
        for (int i = 0; i < loops.Count; i++)
        {
            double ai = Math.Abs(loopArea(loops[i]));
            for (int j = 0; j < loops.Count; j++)
            {
                if (i == j) continue;
                double aj = Math.Abs(loopArea(loops[j]));
                if (ai >= aj) continue;
                if (PointInPolygon(loops2D[i][0], loops2D[j]))
                {
                    if (parentOuter[i] == -1 || aj < Math.Abs(loopArea(loops[parentOuter[i]])))
                        parentOuter[i] = j;
                }
            }
        }

        for (int i = 0; i < loops.Count; i++)
        {
            if (parentOuter[i] != -1) continue; // Delikler parent outer ile kapatılır
            List<Vec3> outer = new List<Vec3>(loops[i]);
            if (loopArea(outer) < 0) outer.Reverse();

            List<double> earcutData = new List<double>();
            List<int> holeIndices = new List<int>();
            foreach (var p in outer) { earcutData.Add(p.x); earcutData.Add(p.y); }

            for (int j = 0; j < loops.Count; j++)
            {
                if (parentOuter[j] != i) continue;
                List<Vec3> h = new List<Vec3>(loops[j]);
                if (loopArea(h) > 0) h.Reverse();
                holeIndices.Add(earcutData.Count / 2);
                foreach (var p in h) { earcutData.Add(p.x); earcutData.Add(p.y); }
            }

            List<int> tris = Earcut.Triangulate(earcutData, holeIndices, 2);
            int nPts = earcutData.Count / 2;
            int[] lowerIndices = new int[nPts];
            int[] upperIndices = new int[nPts];

            for (int p = 0; p < nPts; p++)
            {
                Vec3 pt = new Vec3(earcutData[p * 2], earcutData[p * 2 + 1], splitZ);
                lowerIndices[p] = getOrAdd(lowerMesh, lowerVertMap, pt);
                upperIndices[p] = getOrAdd(upperMesh, upperVertMap, pt);
            }

            List<int> capTris = new List<int>(tris.Count);
            bool[] usedInCap = new bool[nPts];
            for (int t = 0; t < tris.Count; t += 3)
            {
                int i1 = tris[t], i2 = tris[t + 1], i3 = tris[t + 2];
                if (i1 == i2 || i2 == i3 || i3 == i1) continue;
                double ax = earcutData[i1 * 2], ay = earcutData[i1 * 2 + 1];
                double bx = earcutData[i2 * 2], by = earcutData[i2 * 2 + 1];
                double cx = earcutData[i3 * 2], cy = earcutData[i3 * 2 + 1];
                double a2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
                if (Math.Abs(a2) < 1e-12) continue;
                if (a2 < 0) { int tmp = i2; i2 = i3; i3 = tmp; }
                capTris.Add(i1); capTris.Add(i2); capTris.Add(i3);
                usedInCap[i1] = true; usedInCap[i2] = true; usedInCap[i3] = true;
            }

            // Earcut FilterPoints'in atladığı doğrusal (collinear) duvar orta noktalarını kapak kenarlarına geri böl (0 T-Junction!)
            List<int> skippedPts = new List<int>();
            for (int p = 0; p < nPts; p++)
            {
                if (!usedInCap[p]) skippedPts.Add(p);
            }

            if (skippedPts.Count > 0)
            {
                bool didSplit = true;
                while (skippedPts.Count > 0 && didSplit)
                {
                    didSplit = false;
                    HashSet<int> resolvedThisPass = new HashSet<int>();
                    List<int> nextCapTris = new List<int>(capTris.Count + skippedPts.Count * 6);
                    for (int t = 0; t < capTris.Count; t += 3)
                    {
                        int v0 = capTris[t], v1 = capTris[t + 1], v2 = capTris[t + 2];
                        int[] vs = new int[] { v0, v1, v2 };
                        bool triSplit = false;

                        for (int ek = 0; ek < 3 && !triSplit; ek++)
                        {
                            int ea = vs[ek];
                            int eb = vs[(ek + 1) % 3];
                            int ec = vs[(ek + 2) % 3];
                            double ax = earcutData[ea * 2], ay = earcutData[ea * 2 + 1];
                            double bx = earcutData[eb * 2], by = earcutData[eb * 2 + 1];
                            double dx = bx - ax, dy = by - ay;
                            double len2 = dx * dx + dy * dy;
                            if (len2 < 1e-12) continue;
                            double len = Math.Sqrt(len2);

                            List<KeyValuePair<double, int>> hits = null;
                            for (int sIdx = 0; sIdx < skippedPts.Count; sIdx++)
                            {
                                int mp = skippedPts[sIdx];
                                double mx = earcutData[mp * 2], my = earcutData[mp * 2 + 1];
                                double proj = ((mx - ax) * dx + (my - ay) * dy) / len2;
                                if (proj <= 1e-6 || proj >= 1.0 - 1e-6) continue;
                                double perp = Math.Abs(dx * (my - ay) - dy * (mx - ax)) / len;
                                if (perp <= 1e-4)
                                {
                                    if (hits == null) hits = new List<KeyValuePair<double, int>>();
                                    hits.Add(new KeyValuePair<double, int>(proj, mp));
                                }
                            }

                            if (hits != null && hits.Count > 0)
                            {
                                hits.Sort((h1, h2) => h1.Key.CompareTo(h2.Key));
                                int prevP = ea;
                                foreach (var hit in hits)
                                {
                                    nextCapTris.Add(prevP);
                                    nextCapTris.Add(hit.Value);
                                    nextCapTris.Add(ec);
                                    resolvedThisPass.Add(hit.Value);
                                    prevP = hit.Value;
                                }
                                nextCapTris.Add(prevP);
                                nextCapTris.Add(eb);
                                nextCapTris.Add(ec);
                                triSplit = true;
                                didSplit = true;
                            }
                        }

                        if (!triSplit)
                        {
                            nextCapTris.Add(v0);
                            nextCapTris.Add(v1);
                            nextCapTris.Add(v2);
                        }
                    }
                    if (resolvedThisPass.Count > 0)
                        skippedPts.RemoveAll(mp => resolvedThisPass.Contains(mp));
                    capTris = nextCapTris;
                }
            }

            for (int t = 0; t < capTris.Count; t += 3)
            {
                int l1 = lowerIndices[capTris[t]], l2 = lowerIndices[capTris[t + 1]], l3 = lowerIndices[capTris[t + 2]];
                if (l1 != l2 && l2 != l3 && l3 != l1)
                    lowerMesh.AddTriangle(l1, l2, l3);

                int u1 = upperIndices[capTris[t]], u2 = upperIndices[capTris[t + 1]], u3 = upperIndices[capTris[t + 2]];
                if (u1 != u2 && u2 != u3 && u3 != u1)
                    upperMesh.AddTriangle(u1, u3, u2);
            }
        }
        Console.WriteLine("Split ring: Lower={0} verts/{1} tris, Upper={2} verts/{3} tris",
            lowerMesh.Vertices.Count, lowerMesh.Triangles.Count, upperMesh.Vertices.Count, upperMesh.Triangles.Count);
    }

    // ==========================================
    // 5.5. RING CONFIGURATION & METADATA MANAGEMENT
    // ==========================================
    public class RingConfig
    {
        public GeneratorMode Mode = GeneratorMode.IsimliKlips;

        // Üst Yazı Ayarları (İsimli / Marka)
        public string TopText = "G.CIVAN";
        public string BotText = "0506 938 19 49";
        public string FontTop = "Trebuchet MS";
        public string FontBot = "Trebuchet MS";
        public bool UseSoftG = false;

        public double TextHeightTopMm = 3.40;
        public double CharGapTopMm = 0.00;
        public double WidthScaleTop = 0.00;
        public double StretchTop = 100.0; // Dikey Boy / Esnetme Yüzdesi (%100 = 1.0)

        // Alt Yazı Ayarları
        public double TextHeightBotMm = 3.40;
        public double CharGapBotMm = 0.00;
        public double WidthScaleBot = 0.00;
        public double StretchBot = 100.0;

        // Sıralı Klips Ayarları
        public int StartNum = 1;
        public int EndNum = 10;
        public bool IncludeGender = true;
        public string SeqFont = "Trebuchet MS";
        public double SeqTextHeightMm = 3.80;
        public double SeqStretch = 100.0;
        public double SeqCharGapMm = 0.00;
        public double SeqWidthScale = 0.00;
        public double SeqLayerHeight = 0.12;

        // Geriye Uyumluluk Özellikleri
        public double TextHeightMm
        {
            get { return TextHeightTopMm; }
            set { TextHeightTopMm = value; TextHeightBotMm = value; }
        }
        public double CharGapMm
        {
            get { return CharGapTopMm; }
            set { CharGapTopMm = value; CharGapBotMm = value; }
        }
        public double WidthScale
        {
            get { return WidthScaleTop; }
            set { WidthScaleTop = value; WidthScaleBot = value; }
        }

        public double EmbossMm = 0.15;
        public double EmbedMm = 0.50;
        public string Material = "PLA";
        public string SpeedProfile = "fast"; // "fast" (Hızlı Baskı) veya "slow" (Yavaş Baskı)
        public string Color1Hex = "#FFFF00";
        public string Color2Hex = "#FF0000";
        public string CreatedDate = "";

        public string ToJson()
        {
            return string.Format(CultureInfo.InvariantCulture,
                "{{\n" +
                "  \"mode\": \"{0}\",\n" +
                "  \"topText\": \"{1}\",\n" +
                "  \"botText\": \"{2}\",\n" +
                "  \"fontTop\": \"{3}\",\n" +
                "  \"fontBot\": \"{4}\",\n" +
                "  \"textHeightTopMm\": {5:F2},\n" +
                "  \"charGapTopMm\": {6:F2},\n" +
                "  \"widthScaleTop\": {7:F2},\n" +
                "  \"stretchTop\": {8:F1},\n" +
                "  \"textHeightBotMm\": {9:F2},\n" +
                "  \"charGapBotMm\": {10:F2},\n" +
                "  \"widthScaleBot\": {11:F2},\n" +
                "  \"stretchBot\": {12:F1},\n" +
                "  \"startNum\": {13},\n" +
                "  \"endNum\": {14},\n" +
                "  \"includeGender\": {15},\n" +
                "  \"seqFont\": \"{16}\",\n" +
                "  \"seqTextHeightMm\": {17:F2},\n" +
                "  \"seqStretch\": {18:F1},\n" +
                "  \"seqCharGapMm\": {19:F2},\n" +
                "  \"seqWidthScale\": {20:F2},\n" +
                "  \"embossMm\": {21:F2},\n" +
                "  \"embedMm\": {22:F2},\n" +
                "  \"material\": \"{23}\",\n" +
                "  \"color1Hex\": \"{24}\",\n" +
                "  \"color2Hex\": \"{25}\",\n" +
                "  \"speedProfile\": \"{26}\",\n" +
                "  \"useSoftG\": {27},\n" +
                "  \"seqLayerHeight\": {28:F2},\n" +
                "  \"createdDate\": \"{29}\"\n" +
                "}}",
                Mode.ToString(),
                EscapeJson(TopText),
                EscapeJson(BotText),
                EscapeJson(FontTop),
                EscapeJson(FontBot),
                TextHeightTopMm,
                CharGapTopMm,
                WidthScaleTop,
                StretchTop,
                TextHeightBotMm,
                CharGapBotMm,
                WidthScaleBot,
                StretchBot,
                StartNum,
                EndNum,
                IncludeGender ? "true" : "false",
                EscapeJson(SeqFont),
                SeqTextHeightMm,
                SeqStretch,
                SeqCharGapMm,
                SeqWidthScale,
                EmbossMm,
                EmbedMm,
                EscapeJson(Material),
                EscapeJson(Color1Hex),
                EscapeJson(Color2Hex),
                EscapeJson(SpeedProfile),
                UseSoftG ? "true" : "false",
                SeqLayerHeight,
                EscapeJson(CreatedDate));
        }

        public string ToReportText(string fileName)
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("============================================================");
            sb.AppendLine("🕊️ GÜVERCİN HALKASI ÜRETİM BİLGİLERİ VE AYAR RAPORU");
            sb.AppendLine("============================================================");
            sb.AppendLine(string.Format("Dosya Adı        : {0}", fileName));
            sb.AppendLine(string.Format("Model Tipi       : {0}", Mode == GeneratorMode.SiraliKlips ? "SIRALI KLİPS (Çoklu Tabla)" : (Mode == GeneratorMode.MarkaTipi ? "MARKA TİPİ (Dairesel Halka)" : "İSİMLİ KLİPS")));
            sb.AppendLine(string.Format("Üretim Tarihi    : {0}", string.IsNullOrEmpty(CreatedDate) ? DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") : CreatedDate));
            sb.AppendLine(string.Format("Filament Malzeme : {0}", Material));
            sb.AppendLine(string.Format("Baskı Hız Profili : {0}", SpeedProfile == "fast" ? "HIZLI BASKI (50 / 80 / 300 mm/s)" : "YAVAŞ BASKI (18 / 30 / 45 mm/s)"));
            sb.AppendLine();

            if (Mode == GeneratorMode.SiraliKlips)
            {
                sb.AppendLine("[SIRALI KLİPS BİLGİLERİ]");
                sb.AppendLine(string.Format("Numara Aralığı   : {0}..{1} ({2})", StartNum, EndNum, IncludeGender ? "+ DiŞi, ERKEK" : "Sadece Numaralar"));
                sb.AppendLine(string.Format("Yazı Tipi (Font) : {0}", SeqFont));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Boyu        : {0:F2} mm", SeqTextHeightMm));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Dikey Boy        : %{0:0}", SeqStretch));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Boşluk (Gap): {0:F2} mm", SeqCharGapMm));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Kalınlığı   : {0:F2} mm (Boldness)", SeqWidthScale));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Katman Yüksekliği: {0:F2} mm", SeqLayerHeight));
                sb.AppendLine("Tabla 1 Düzeni   : Düz Siyah Gövde / Beyaz Yazı");
                sb.AppendLine(string.Format("Tabla 2 Düzeni   : Çift Renk ({0} Alt / {1} Üst) / Siyah Yazı", Color1Hex, Color2Hex));
            }
            else
            {
                sb.AppendLine(string.Format("[{0} - İSİM / MARKA]", Mode == GeneratorMode.MarkaTipi ? "ÜST BÖLÜM (MARKA / İSİM)" : "ÜST BÖLÜM - İSİM SOYİSİM"));
                sb.AppendLine(string.Format("Yazı             : {0}", TopText));
                sb.AppendLine(string.Format("Yazı Tipi (Font) : {0}", FontTop));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Boyu        : {0:F2} mm (%{1:0} - 4.50 mm bant içinde)", TextHeightTopMm, (TextHeightTopMm / 4.50) * 100.0));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Dikey Boy        : %{0:0} (Esnetme / Yükseklik)", StretchTop));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Boşluk (Gap): {0:F2} mm", CharGapTopMm));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Kalınlığı   : {0:F2} mm (Boldness)", WidthScaleTop));
                sb.AppendLine();
                sb.AppendLine(string.Format("[{0} - TELEFON / SERİ NO]", Mode == GeneratorMode.MarkaTipi ? "ALT BÖLÜM (SERİ NO / TELEFON)" : "ALT BÖLÜM - TELEFON NUMARASI"));
                sb.AppendLine(string.Format("Yazı             : {0}", BotText));
                sb.AppendLine(string.Format("Yazı Tipi (Font) : {0}", FontBot));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Boyu        : {0:F2} mm (%{1:0} - 4.50 mm bant içinde)", TextHeightBotMm, (TextHeightBotMm / 4.50) * 100.0));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Dikey Boy        : %{0:0} (Esnetme / Yükseklik)", StretchBot));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Boşluk (Gap): {0:F2} mm", CharGapBotMm));
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Harf Kalınlığı   : {0:F2} mm (Boldness)", WidthScaleBot));
            }

            sb.AppendLine();
            sb.AppendLine("[GEOMETRİ VE DİLİMLEME AYARLARI]");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "Dış Kabartma     : {0:F2} mm (Emboss)", EmbossMm));
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "İç Gömülme       : {0:F2} mm (Embed)", EmbedMm));
            sb.AppendLine("Kırlangıç Kilit  : 0.20 mm (Yazı kökü gövde içinde genişler - sökülmez)");
            sb.AppendLine(string.Format("Halka Tipi       : {0}", Mode == GeneratorMode.MarkaTipi ? "Marka Tipi Dairesel Halka (310° Yay, Arkada 50° Dikiş/Seam Boşluğu)" : "Klipsli Güvercin Halkası (252° Güvenli Yay)"));
            sb.AppendLine("Toplam Halka Boyu: 9.00 mm (Tabla teması Z = 0.00 mm)");
            sb.AppendLine();
            sb.AppendLine("[BASKI RENKLERİ]");
            sb.AppendLine(string.Format("Renk 1 (Gövde/Yazı) : {0} (Alt Bant Gövdesi / Üst Yazı)", Color1Hex));
            sb.AppendLine(string.Format("Renk 2 (Gövde/Yazı) : {0} (Üst Bant Gövdesi / Alt Yazı)", Color2Hex));
            sb.AppendLine("============================================================");
            sb.AppendLine("NOT: Bu ayarları tekrar uygulamaya yüklemek için");
            sb.AppendLine("PigeonRingGenerator'da '📂 Ayarları .3mf'den Yükle...'");
            sb.AppendLine("butonuna basıp ilgili .3mf dosyasını seçebilirsiniz.");
            sb.AppendLine("============================================================");
            return sb.ToString();
        }

        private static string EscapeJson(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            return s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\r", "").Replace("\n", "\\n");
        }

        public static RingConfig FromJson(string json)
        {
            RingConfig cfg = new RingConfig();
            if (string.IsNullOrEmpty(json)) return cfg;

            string modeStr = ExtractString(json, "mode", "IsimliKlips");
            if (string.Equals(modeStr, "SiraliKlips", StringComparison.OrdinalIgnoreCase)) cfg.Mode = GeneratorMode.SiraliKlips;
            else if (string.Equals(modeStr, "MarkaTipi", StringComparison.OrdinalIgnoreCase)) cfg.Mode = GeneratorMode.MarkaTipi;
            else cfg.Mode = GeneratorMode.IsimliKlips;

            cfg.TopText = ExtractString(json, "topText", cfg.TopText);
            cfg.BotText = ExtractString(json, "botText", cfg.BotText);
            cfg.FontTop = ExtractString(json, "fontTop", cfg.FontTop);
            cfg.FontBot = ExtractString(json, "fontBot", cfg.FontBot);
            cfg.Material = ExtractString(json, "material", cfg.Material);
            cfg.Color1Hex = ExtractString(json, "color1Hex", cfg.Color1Hex);
            cfg.Color2Hex = ExtractString(json, "color2Hex", cfg.Color2Hex);
            cfg.SpeedProfile = ExtractString(json, "speedProfile", "fast");
            cfg.CreatedDate = ExtractString(json, "createdDate", cfg.CreatedDate);

            double legacyHeight = ExtractDouble(json, "textHeightMm", 3.40);
            double legacyGap = ExtractDouble(json, "charGapMm", 0.00);
            double legacyScale = ExtractDouble(json, "widthScale", 0.00);
            double legacyStretch = ExtractDouble(json, "stretch", 100.0);

            cfg.TextHeightTopMm = ExtractDouble(json, "textHeightTopMm", legacyHeight);
            cfg.CharGapTopMm = ExtractDouble(json, "charGapTopMm", legacyGap);
            cfg.WidthScaleTop = ExtractDouble(json, "widthScaleTop", legacyScale);
            cfg.StretchTop = ExtractDouble(json, "stretchTop", legacyStretch);

            cfg.TextHeightBotMm = ExtractDouble(json, "textHeightBotMm", legacyHeight);
            cfg.CharGapBotMm = ExtractDouble(json, "charGapBotMm", legacyGap);
            cfg.WidthScaleBot = ExtractDouble(json, "widthScaleBot", legacyScale);
            cfg.StretchBot = ExtractDouble(json, "stretchBot", legacyStretch);

            cfg.StartNum = (int)ExtractDouble(json, "startNum", 1);
            cfg.EndNum = (int)ExtractDouble(json, "endNum", 10);
            string incGen = ExtractString(json, "includeGender", "true");
            cfg.IncludeGender = !string.Equals(incGen, "false", StringComparison.OrdinalIgnoreCase);
            cfg.SeqFont = ExtractString(json, "seqFont", cfg.SeqFont);
            cfg.SeqTextHeightMm = ExtractDouble(json, "seqTextHeightMm", 3.80);
            cfg.SeqStretch = ExtractDouble(json, "seqStretch", 100.0);
            cfg.SeqCharGapMm = ExtractDouble(json, "seqCharGapMm", 0.00);
            cfg.SeqWidthScale = ExtractDouble(json, "seqWidthScale", 0.00);
            cfg.SeqLayerHeight = ExtractDouble(json, "seqLayerHeight", 0.12);

            cfg.EmbossMm = ExtractDouble(json, "embossMm", cfg.EmbossMm);
            cfg.EmbedMm = ExtractDouble(json, "embedMm", cfg.EmbedMm);

            string sSoftG = ExtractString(json, "useSoftG", "false");
            cfg.UseSoftG = string.Equals(sSoftG, "true", StringComparison.OrdinalIgnoreCase);

            return cfg;
        }

        private static string ExtractString(string json, string key, string fallback)
        {
            var match = System.Text.RegularExpressions.Regex.Match(json, "\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
            return match.Success ? match.Groups[1].Value : fallback;
        }

        private static double ExtractDouble(string json, string key, double fallback)
        {
            var match = System.Text.RegularExpressions.Regex.Match(json, "\"" + key + "\"\\s*:\\s*([0-9\\.]+)");
            if (match.Success)
            {
                double val;
                if (double.TryParse(match.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out val))
                    return val;
            }
            return fallback;
        }

        public static RingConfig LoadFrom3mf(string path)
        {
            if (!File.Exists(path)) return null;

            try
            {
                using (ZipArchive zip = ZipFile.OpenRead(path))
                {
                    var entry = zip.GetEntry("Metadata/pigeon_ring_config.json");
                    if (entry != null)
                    {
                        using (StreamReader sr = new StreamReader(entry.Open(), Encoding.UTF8))
                        {
                            return FromJson(sr.ReadToEnd());
                        }
                    }

                    var modelEntry = zip.GetEntry("3D/3dmodel.model");
                    if (modelEntry != null)
                    {
                        using (StreamReader sr = new StreamReader(modelEntry.Open(), Encoding.UTF8))
                        {
                            string xml = sr.ReadToEnd();
                            var m = System.Text.RegularExpressions.Regex.Match(xml, "<metadata name=\"PigeonRingConfig\"(?: content=\"(.*?)\"|>(.*?)</metadata>)", System.Text.RegularExpressions.RegexOptions.Singleline);
                            if (m.Success)
                            {
                                string raw = !string.IsNullOrEmpty(m.Groups[1].Value) ? m.Groups[1].Value : m.Groups[2].Value;
                                string unescaped = System.Net.WebUtility.HtmlDecode(raw);
                                return FromJson(unescaped);
                            }
                        }
                    }
                }
            }
            catch { }

            string dir = Path.GetDirectoryName(path);
            string baseName = Path.GetFileNameWithoutExtension(path);
            string txtPath = Path.Combine(dir, baseName + "_bilgi.txt");
            if (!File.Exists(txtPath)) txtPath = Path.Combine(dir, baseName + ".txt");
            if (File.Exists(txtPath))
            {
                return LoadFromTextFile(txtPath);
            }

            return null;
        }

        public static RingConfig LoadFromTextFile(string path)
        {
            if (!File.Exists(path)) return null;
            try
            {
                RingConfig cfg = new RingConfig();
                string[] lines = File.ReadAllLines(path, Encoding.UTF8);
                bool inTopSection = false;
                bool inBotSection = false;
                bool inSeqSection = false;

                foreach (string line in lines)
                {
                    string trimmed = line.Trim();
                    if (trimmed.StartsWith("[ÜST BÖLÜM", StringComparison.OrdinalIgnoreCase)) { inTopSection = true; inBotSection = false; inSeqSection = false; continue; }
                    if (trimmed.StartsWith("[ALT BÖLÜM", StringComparison.OrdinalIgnoreCase)) { inTopSection = false; inBotSection = true; inSeqSection = false; continue; }
                    if (trimmed.StartsWith("[SIRALI KLİPS", StringComparison.OrdinalIgnoreCase)) { inTopSection = false; inBotSection = false; inSeqSection = true; cfg.Mode = GeneratorMode.SiraliKlips; continue; }
                    if (trimmed.StartsWith("[GEOMETRİ", StringComparison.OrdinalIgnoreCase) || trimmed.StartsWith("[BASKI", StringComparison.OrdinalIgnoreCase)) { inTopSection = false; inBotSection = false; inSeqSection = false; }

                    string[] parts = line.Split(new char[] { ':' }, 2);
                    if (parts.Length != 2) continue;
                    string k = parts[0].Trim();
                    string v = parts[1].Trim();

                    if (k.Contains("Model Tipi"))
                    {
                        if (v.Contains("SIRALI")) cfg.Mode = GeneratorMode.SiraliKlips;
                        else if (v.Contains("MARKA")) cfg.Mode = GeneratorMode.MarkaTipi;
                        else cfg.Mode = GeneratorMode.IsimliKlips;
                    }
                    else if (k.Contains("Numara Aralığı"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"(\d+)\.\.(\d+)");
                        if (m.Success)
                        {
                            int.TryParse(m.Groups[1].Value, out cfg.StartNum);
                            int.TryParse(m.Groups[2].Value, out cfg.EndNum);
                        }
                        cfg.IncludeGender = v.Contains("DiŞi");
                    }
                    else if (inSeqSection && k.Contains("Font")) cfg.SeqFont = v;
                    else if (k.Contains("Üst Bölüm (İsim)") || (inTopSection && k.Equals("Yazı", StringComparison.OrdinalIgnoreCase))) cfg.TopText = v;
                    else if (k.Contains("Üst Font") || (inTopSection && k.Contains("Font"))) cfg.FontTop = v;
                    else if (k.Contains("Alt Bölüm (Tel)") || (inBotSection && k.Equals("Yazı", StringComparison.OrdinalIgnoreCase))) cfg.BotText = v;
                    else if (k.Contains("Alt Font") || (inBotSection && k.Contains("Font"))) cfg.FontBot = v;
                    else if (k.Contains("Filament Malzeme")) cfg.Material = v;
                    else if (k.Contains("Harf Boyu"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"([0-9\.]+)");
                        if (m.Success)
                        {
                            double val;
                            if (double.TryParse(m.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out val))
                            {
                                if (inSeqSection) cfg.SeqTextHeightMm = val;
                                else if (inTopSection) cfg.TextHeightTopMm = val;
                                else if (inBotSection) cfg.TextHeightBotMm = val;
                                else { cfg.TextHeightTopMm = val; cfg.TextHeightBotMm = val; }
                            }
                        }
                    }
                    else if (k.Contains("Dikey Boy") || k.Contains("Esnetme"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"([0-9\.]+)");
                        if (m.Success)
                        {
                            double val;
                            if (double.TryParse(m.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out val))
                            {
                                if (inSeqSection) cfg.SeqStretch = val;
                                else if (inTopSection) cfg.StretchTop = val;
                                else if (inBotSection) cfg.StretchBot = val;
                                else { cfg.StretchTop = val; cfg.StretchBot = val; }
                            }
                        }
                    }
                    else if (k.Contains("Harf Boşluk"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"([0-9\.]+)");
                        if (m.Success)
                        {
                            double val;
                            if (double.TryParse(m.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out val))
                            {
                                if (inSeqSection) cfg.SeqCharGapMm = val;
                                else if (inTopSection) cfg.CharGapTopMm = val;
                                else if (inBotSection) cfg.CharGapBotMm = val;
                                else { cfg.CharGapTopMm = val; cfg.CharGapBotMm = val; }
                            }
                        }
                    }
                    else if (k.Contains("Harf Kalınlığı"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"([0-9\.]+)");
                        if (m.Success)
                        {
                            double val;
                            if (double.TryParse(m.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out val))
                            {
                                if (inSeqSection) cfg.SeqWidthScale = val;
                                else if (inTopSection) cfg.WidthScaleTop = val;
                                else if (inBotSection) cfg.WidthScaleBot = val;
                                else { cfg.WidthScaleTop = val; cfg.WidthScaleBot = val; }
                            }
                        }
                    }
                    else if (k.Contains("Dış Kabartma"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"([0-9\.]+)");
                        if (m.Success) double.TryParse(m.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out cfg.EmbossMm);
                    }
                    else if (k.Contains("İç Gömülme"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"([0-9\.]+)");
                        if (m.Success) double.TryParse(m.Groups[1].Value, NumberStyles.Any, CultureInfo.InvariantCulture, out cfg.EmbedMm);
                    }
                    else if (k.Contains("Renk 1"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"(#[0-9A-Fa-f]{6})");
                        if (m.Success) cfg.Color1Hex = m.Groups[1].Value;
                    }
                    else if (k.Contains("Renk 2"))
                    {
                        var m = System.Text.RegularExpressions.Regex.Match(v, @"(#[0-9A-Fa-f]{6})");
                        if (m.Success) cfg.Color2Hex = m.Groups[1].Value;
                    }
                }
                return cfg;
            }
            catch
            {
                return null;
            }
        }
    }

    // ==========================================
    // 5.6. SEQUENTIAL MULTI-PLATE EXECUTION (SIRALI KLİPS)
    // ==========================================
    public static List<string> BuildLabels(int startNum, int endNum, bool includeGender)
    {
        List<string> list = new List<string>();
        int pad = endNum >= 100 ? 3 : 2;
        string fmt = "D" + pad;
        for (int i = startNum; i <= endNum; i++)
        {
            list.Add(i.ToString(fmt));
        }
        if (includeGender)
        {
            list.Add("DiŞi");
            list.Add("ERKEK");
        }
        return list;
    }

    public static bool ExecuteSequentialGeneration(string backup3mf, string output3mf, 
                                                   int startNum, int endNum, bool includeGender,
                                                   double textHeightWorld, double embossMm, double embedMm,
                                                   string fontName, double charGapMm, double widthScale,
                                                   Action<string> log, string material = "PLA",
                                                   string color1Hex = "#FFFF00", string color2Hex = "#FF0000",
                                                   string color3Hex = "#FFFFFF",
                                                   double stretchY = 100.0,
                                                   string speedProfile = "fast",
                                                   double layerHeight = 0.12)
    {
        output3mf = output3mf.Trim('"', '\'').TrimEnd('\\', '/');
        string logPath = Path.Combine(GetAppDataDir(), "generator.log");
        Action<string> fullLog = s =>
        {
            try { File.AppendAllText(logPath, s + "\r\n", Encoding.UTF8); } catch { }
            if (log != null) log(s);
            try { Console.WriteLine(s); } catch { }
        };

        try
        {
            if (string.IsNullOrEmpty(fontName)) fontName = "Trebuchet MS";
            if (textHeightWorld > 9.00) textHeightWorld = 9.00;
            bool isPla = string.Equals(material, "PLA", StringComparison.OrdinalIgnoreCase);
            string matType = isPla ? "PLA" : "PETG";
            bool isFastSpeed = string.Equals(speedProfile, "fast", StringComparison.OrdinalIgnoreCase) || string.IsNullOrEmpty(speedProfile);

            List<string> labels = BuildLabels(startNum, endNum, includeGender);
            int count = labels.Count;

            fullLog("=== GÜVERCİN SIRALI KLİPS ÜRETİCİ BAŞLATILDI ===");
            fullLog(string.Format("Tarih/Saat: {0}", DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss")));
            fullLog(string.Format("Seçilen Malzeme (Filament): {0}", matType));
            fullLog(string.Format("Numara Aralığı: {0}..{1} {2} (Toplam: {3} Adet)", 
                startNum, endNum, includeGender ? "(+ DiŞi, ERKEK)" : "", count));
            fullLog(string.Format("Baskı Renkleri (Tam 3 Renk): 1={0} (Alt/Düz Gövde) | 2={1} (Üst Gövde) | 3={2} (Yazı Rengi)", color1Hex, color2Hex, color3Hex));
            fullLog("Hedef: 2 Ayrı Tabla (" + count + "şer Sıralı Klips)");
            fullLog(string.Format("Tabla 1: Düz Renk ({0} Gövde, {1} Yazı)", color1Hex, color3Hex));
            fullLog(string.Format("Tabla 2: Çift Renk ({0} Alt / {1} Üst Gövde, {2} Yazı)", color1Hex, color2Hex, color3Hex));
            fullLog(string.Format("Yazı: Font={0} | Boyut={1:F2} mm (Dikey: %{2:0}) | Gap={3:F2} mm | Kalınlık={4:F2} mm", fontName, textHeightWorld, stretchY, charGapMm, widthScale));
            fullLog(string.Format("Kabartma (Emboss): {0:F2} mm | İç Gömülme (Embed): {1:F2} mm", embossMm, embedMm));

            if (!File.Exists(backup3mf))
            {
                fullLog("HATA: Şablon dosyası bulunamadı: " + backup3mf);
                return false;
            }

            // 1. Orijinal halka modelini oku
            fullLog("[1/5] Şablon dosyasından halka geometrisi okunuyor...");
            string ringXml = "";
            using (ZipArchive zip = ZipFile.OpenRead(backup3mf))
            {
                foreach (var e in zip.Entries)
                {
                    if (e.FullName.EndsWith(".model", StringComparison.OrdinalIgnoreCase) && !e.FullName.Contains("3dmodel.model"))
                    {
                        using (StreamReader sr = new StreamReader(e.Open(), Encoding.UTF8))
                        {
                            ringXml = sr.ReadToEnd();
                        }
                        break;
                    }
                }
                if (string.IsNullOrEmpty(ringXml))
                {
                    var mEntry = zip.GetEntry("3D/3dmodel.model");
                    if (mEntry != null)
                    {
                        using (StreamReader sr = new StreamReader(mEntry.Open(), Encoding.UTF8))
                        {
                            ringXml = sr.ReadToEnd();
                        }
                    }
                }
            }
            if (string.IsNullOrEmpty(ringXml))
            {
                fullLog("HATA: Şablon zip içinde halka model XML verisi bulunamadı.");
                return false;
            }

            // 2. Ham tek parça gövdeyi oku, dinamik Z ölçeğini hesapla ve Use-Surface örnekleyici kur
            SolidMesh wholeRing = ParseMeshFromModelXml(ringXml);
            double origMinZ = double.MaxValue, origMaxZ = double.MinValue;
            foreach (var v in wholeRing.Vertices) { if (v.z < origMinZ) origMinZ = v.z; if (v.z > origMaxZ) origMaxZ = v.z; }
            double origH = (origMaxZ > origMinZ) ? (origMaxZ - origMinZ) : 7.168;
            double splitZ = (origMinZ + origMaxZ) * 0.5;
            double dynZScale = 9.000 / origH;
            double compTz = -0.616014361;
            double seqPosZ = -(origMinZ + compTz) * dynZScale;

            RingSurfaceSampler surfaceSampler = new RingSurfaceSampler(wholeRing, 0.0, -0.2754);

            fullLog("[2/5] Tek parça 9 mm gövde korunuyor (bölme yok)...");
            fullLog(string.Format("  -> Tek Parça Gövde: {0} köşe, {1} üçgen | Z=[{2:F3}, {3:F3}] (dynZScale={4:F6})", wholeRing.Vertices.Count, wholeRing.Triangles.Count, origMinZ, origMaxZ, dynZScale));

            // 3. Sıralı yazı mesh'lerini üret (Use-Surface + Per-Glyph, 236.0° simetrik merkez / 258.0° kilit korumalı yay)
            fullLog(string.Format("[3/5] {0} adet sıralı Use-Surface silindirik yazı üretiliyor (Yükseklik={1:F2}mm)...", count, textHeightWorld));
            List<SolidMesh> textMeshes = new List<SolidMesh>();
            for (int i = 0; i < labels.Count; i++)
            {
                string lbl = labels[i];
                SolidMesh tm = GenerateDistributedCylindricalText(lbl, fontName, textHeightWorld, splitZ, 236.0, embedMm, embossMm, dynZScale, charGapMm, 0, widthScale, 258.0, stretchY / 100.0, false, 0.0, 0.0, true, surfaceSampler);
                textMeshes.Add(tm);
                fullLog(string.Format("  -> [{0}/{1}] Yazı '{2}': {3} köşe, {4} üçgen", i + 1, count, lbl, tm.Vertices.Count, tm.Triangles.Count));
            }

            // 4. Geçici çalışma dizini oluştur
            fullLog("[4/5] Multi-plate 3MF proje yapısı inşa ediliyor...");
            string tempDir = Path.Combine(Path.GetTempPath(), "pigeon_seq_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(tempDir);

            // Şablon dosyaları çıkar
            using (ZipArchive zip = ZipFile.OpenRead(backup3mf))
            {
                foreach (var e in zip.Entries)
                {
                    if (string.IsNullOrEmpty(e.Name) || e.FullName.EndsWith("/") || e.FullName.EndsWith(@"\"))
                        continue;
                    if (e.FullName.StartsWith("3D/Objects/", StringComparison.OrdinalIgnoreCase) ||
                        e.FullName.StartsWith("3D/_rels/", StringComparison.OrdinalIgnoreCase))
                        continue;

                    string destPath = Path.Combine(tempDir, e.FullName.Replace('/', Path.DirectorySeparatorChar));
                    Directory.CreateDirectory(Path.GetDirectoryName(destPath));
                    e.ExtractToFile(destPath, true);
                }
            }

            // Izgara hesabı: her tabla için dinamik kolon ve satır
            int cols = (int)Math.Ceiling(Math.Sqrt(count * 1.3));
            if (cols < 4) cols = 4;
            int rows = (int)Math.Ceiling((double)count / cols);

            double stepX = 20.0;
            double stepY = 20.0;

            double startX1 = 130.0 - (cols - 1) * stepX * 0.5;
            double startY1 = 130.0 - (rows - 1) * stepY * 0.5;

            double startX2 = 480.0 - (cols - 1) * stepX * 0.5;
            double startY2 = 130.0 - (rows - 1) * stepY * 0.5;

            // 5. 3D/3dmodel.model dosyasını oluştur
            string model3DPath = Path.Combine(tempDir, "3D", "3dmodel.model");
            using (StreamWriter sw = new StreamWriter(model3DPath, false, Encoding.UTF8))
            {
                sw.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
                sw.WriteLine("<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">");
                sw.WriteLine(" <metadata name=\"Application\">Snapmaker_Orca-1.10.1</metadata>");
                sw.WriteLine(" <metadata name=\"Snapmaker_Orca:3mfVersion\">1</metadata>");
                sw.WriteLine(" <resources>");

                // Object 1: ring_whole (Kesintisiz tek parça 9 mm gövde)
                sw.WriteLine("  <object id=\"1\" type=\"model\">");
                sw.WriteLine("   <mesh>");
                sw.WriteLine("    <vertices>");
                foreach (var v in wholeRing.Vertices)
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "     <vertex x=\"{0:F6}\" y=\"{1:F6}\" z=\"{2:F6}\"/>", v.x, v.y, v.z));
                sw.WriteLine("    </vertices>");
                sw.WriteLine("    <triangles>");
                foreach (var t in wholeRing.Triangles)
                    sw.WriteLine(string.Format("     <triangle v1=\"{0}\" v2=\"{1}\" v3=\"{2}\"/>", t.v1, t.v2, t.v3));
                sw.WriteLine("    </triangles>");
                sw.WriteLine("   </mesh>");
                sw.WriteLine("  </object>");

                // Objects 4..(4 + count - 1): text meshes
                for (int i = 0; i < count; i++)
                {
                    int objId = 4 + i;
                    sw.WriteLine(string.Format("  <object id=\"{0}\" type=\"model\">", objId));
                    sw.WriteLine("   <mesh>");
                    sw.WriteLine("    <vertices>");
                    foreach (var v in textMeshes[i].Vertices)
                        sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "     <vertex x=\"{0:F6}\" y=\"{1:F6}\" z=\"{2:F6}\"/>", v.x, v.y, v.z));
                    sw.WriteLine("    </vertices>");
                    sw.WriteLine("    <triangles>");
                    foreach (var t in textMeshes[i].Triangles)
                        sw.WriteLine(string.Format("     <triangle v1=\"{0}\" v2=\"{1}\" v3=\"{2}\"/>", t.v1, t.v2, t.v3));
                    sw.WriteLine("    </triangles>");
                    sw.WriteLine("   </mesh>");
                    sw.WriteLine("  </object>");
                }

                // Assembly Objects:
                // Plate 1 (tek parça gövde id=1 + yazı): 4+count .. 4+2*count-1
                // Plate 2 (alt/üst gövde id=2,3 + yazı): 4+2*count .. 4+3*count-1
                int nextObjId = 4 + count;
                for (int i = 0; i < count; i++)
                {
                    int textObjId = 4 + i;
                    sw.WriteLine(string.Format("  <object id=\"{0}\" type=\"model\">", nextObjId++));
                    sw.WriteLine("   <components>");
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "    <component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 -0.23797 -0.07058 {0:F6}\"/>", compTz));
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "    <component objectid=\"{0}\" transform=\"1 0 0 0 1 0 0 0 1 -0.23797 -0.07058 {1:F6}\"/>", textObjId, compTz));
                    sw.WriteLine("   </components>");
                    sw.WriteLine("  </object>");
                }

                for (int i = 0; i < count; i++)
                {
                    int textObjId = 4 + i;
                    sw.WriteLine(string.Format("  <object id=\"{0}\" type=\"model\">", nextObjId++));
                    sw.WriteLine("   <components>");
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "    <component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 -0.23797 -0.07058 {0:F6}\"/>", compTz));
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "    <component objectid=\"{0}\" transform=\"1 0 0 0 1 0 0 0 1 -0.23797 -0.07058 {1:F6}\"/>", textObjId, compTz));
                    sw.WriteLine("   </components>");
                    sw.WriteLine("  </object>");
                }

                sw.WriteLine(" </resources>");
                sw.WriteLine(" <build>");

                // Plate 1 build items
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + count + i;
                    int row = i / cols;
                    int col = i % cols;
                    double posX = startX1 + col * stepX;
                    double posY = startY1 + row * stepY;

                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "  <item objectid=\"{0}\" transform=\"0.8209808 0.5709558 0 -0.5709558 0.8209808 0 0 0 {1:F6} {2:F6} {3:F6} {4:F6}\" printable=\"1\"/>",
                        assId, dynZScale, posX, posY, seqPosZ));
                }

                // Plate 2 build items
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + 2 * count + i;
                    int row = i / cols;
                    int col = i % cols;
                    double posX = startX2 + col * stepX;
                    double posY = startY2 + row * stepY;

                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "  <item objectid=\"{0}\" transform=\"0.8209808 0.5709558 0 -0.5709558 0.8209808 0 0 0 {1:F6} {2:F6} {3:F6} {4:F6}\" printable=\"1\"/>",
                        assId, dynZScale, posX, posY, seqPosZ));
                }

                sw.WriteLine(" </build>");
                sw.WriteLine("</model>");
            }

            // 6. Metadata/model_settings.config oluştur
            string modelSettingsPath = Path.Combine(tempDir, "Metadata", "model_settings.config");
            using (StreamWriter sw = new StreamWriter(modelSettingsPath, false, Encoding.UTF8))
            {
                sw.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
                sw.WriteLine("<config>");

                // Plate 1 Assemblies: Tek parça gövde (id=1 -> Ext 1) + Yazı (id=4+i -> Ext 3)
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + count + i;
                    int textObjId = 4 + i;
                    string name = "Klips_" + MakeSafeFileName(labels[i]) + "_Duz";

                    sw.WriteLine(string.Format("  <object id=\"{0}\">", assId));
                    sw.WriteLine(string.Format("    <metadata key=\"name\" value=\"{0}\"/>", name));
                    sw.WriteLine("    <part id=\"1\" subtype=\"normal_part\">");
                    sw.WriteLine("      <metadata key=\"name\" value=\"ring_whole\"/>");
                    sw.WriteLine("      <metadata key=\"extruder\" value=\"1\"/>");
                    sw.WriteLine("    </part>");
                    sw.WriteLine(string.Format("    <part id=\"{0}\" subtype=\"normal_part\">", textObjId));
                    sw.WriteLine(string.Format("      <metadata key=\"name\" value=\"text_{0}\"/>", MakeSafeFileName(labels[i])));
                    sw.WriteLine("      <metadata key=\"extruder\" value=\"3\"/>");
                    sw.WriteLine("    </part>");
                    sw.WriteLine("  </object>");
                }

                // Plate 2 Assemblies: Tek Parça Gövde (id=1 -> Ext 1), Yazı (id=4+i -> Ext 3)
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + 2 * count + i;
                    int textObjId = 4 + i;
                    string name = "Klips_" + MakeSafeFileName(labels[i]) + "_CiftRenk";

                    sw.WriteLine(string.Format("  <object id=\"{0}\">", assId));
                    sw.WriteLine(string.Format("    <metadata key=\"name\" value=\"{0}\"/>", name));
                    sw.WriteLine("    <part id=\"1\" subtype=\"normal_part\">");
                    sw.WriteLine("      <metadata key=\"name\" value=\"ring_whole\"/>");
                    sw.WriteLine("      <metadata key=\"extruder\" value=\"1\"/>");
                    sw.WriteLine("    </part>");
                    sw.WriteLine(string.Format("    <part id=\"{0}\" subtype=\"normal_part\">", textObjId));
                    sw.WriteLine(string.Format("      <metadata key=\"name\" value=\"text_{0}\"/>", MakeSafeFileName(labels[i])));
                    sw.WriteLine("      <metadata key=\"extruder\" value=\"3\"/>");
                    sw.WriteLine("    </part>");
                    sw.WriteLine("  </object>");
                }

                sw.WriteLine("  <plate>");
                sw.WriteLine("    <metadata key=\"plater_id\" value=\"1\"/>");
                sw.WriteLine("    <metadata key=\"plater_name\" value=\"1-Duz Renk\"/>");
                sw.WriteLine("    <metadata key=\"locked\" value=\"false\"/>");
                sw.WriteLine("    <metadata key=\"filament_maps\" value=\"1 3\"/>");
                sw.WriteLine("    <metadata key=\"thumbnail_file\" value=\"Metadata/plate_1.png\"/>");
                sw.WriteLine("    <metadata key=\"top_file\" value=\"Metadata/top_1.png\"/>");
                sw.WriteLine("    <metadata key=\"pick_file\" value=\"Metadata/pick_1.png\"/>");
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + count + i;
                    sw.WriteLine("    <model_instance>");
                    sw.WriteLine(string.Format("      <metadata key=\"object_id\" value=\"{0}\"/>", assId));
                    sw.WriteLine("      <metadata key=\"instance_id\" value=\"0\"/>");
                    sw.WriteLine(string.Format("      <metadata key=\"identify_id\" value=\"{0}\"/>", 100 + assId));
                    sw.WriteLine("    </model_instance>");
                }
                sw.WriteLine("  </plate>");

                sw.WriteLine("  <plate>");
                sw.WriteLine("    <metadata key=\"plater_id\" value=\"2\"/>");
                sw.WriteLine("    <metadata key=\"plater_name\" value=\"2-Cift Renk\"/>");
                sw.WriteLine("    <metadata key=\"locked\" value=\"false\"/>");
                sw.WriteLine("    <metadata key=\"filament_maps\" value=\"1 2 3\"/>");
                sw.WriteLine("    <metadata key=\"thumbnail_file\" value=\"Metadata/plate_1.png\"/>");
                sw.WriteLine("    <metadata key=\"top_file\" value=\"Metadata/top_1.png\"/>");
                sw.WriteLine("    <metadata key=\"pick_file\" value=\"Metadata/pick_1.png\"/>");
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + 2 * count + i;
                    sw.WriteLine("    <model_instance>");
                    sw.WriteLine(string.Format("      <metadata key=\"object_id\" value=\"{0}\"/>", assId));
                    sw.WriteLine("      <metadata key=\"instance_id\" value=\"0\"/>");
                    sw.WriteLine(string.Format("      <metadata key=\"identify_id\" value=\"{0}\"/>", 100 + assId));
                    sw.WriteLine("    </model_instance>");
                }
                sw.WriteLine("  </plate>");

                sw.WriteLine("  <assemble>");
                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + count + i;
                    int row = i / cols;
                    int col = i % cols;
                    double posX = startX1 + col * stepX;
                    double posY = startY1 + row * stepY;
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "   <assemble_item object_id=\"{0}\" instance_id=\"0\" transform=\"0.8209808 0.5709558 0 -0.5709558 0.8209808 0 0 0 {1:F6} {2:F6} {3:F6} {4:F6}\" offset=\"0 0 0.6118\" />",
                        assId, dynZScale, posX, posY, seqPosZ));
                }

                for (int i = 0; i < count; i++)
                {
                    int assId = 4 + 2 * count + i;
                    int row = i / cols;
                    int col = i % cols;
                    double posX = startX2 + col * stepX;
                    double posY = startY2 + row * stepY;
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "   <assemble_item object_id=\"{0}\" instance_id=\"0\" transform=\"0.8209808 0.5709558 0 -0.5709558 0.8209808 0 0 0 {1:F6} {2:F6} {3:F6} {4:F6}\" offset=\"0 0 0.6118\" />",
                        assId, dynZScale, posX, posY, seqPosZ));
                }
                sw.WriteLine("  </assemble>");
                sw.WriteLine("</config>");
            }

            // 7. Slicer ve Filament Ayarları
            fullLog(string.Format("[5/5] Dilimleme parametreleri ({0}) ve 3 renkli {1} profili yapılandırılıyor...", isFastSpeed ? "HIZLI BASKI" : "YAVAŞ BASKI", matType));
            string filProfileName = string.Format("Generic {0} (Klips)({1})", matType, Path.GetFileName(output3mf));
            string inheritsFil = isPla ? "Generic PLA" : "Generic PETG";

            string sOuterWallSpeed = isFastSpeed ? "80" : "30";
            string sInnerWallSpeed = isFastSpeed ? "300" : "45";
            string sSmallPerimeterSpeed = isFastSpeed ? "50" : "20";
            string sSparseInfillSpeed = isFastSpeed ? "270" : "50";
            string sTopSurfaceSpeed = isFastSpeed ? "200" : "30";
            string sInitialLayerSpeed = isFastSpeed ? "50" : "18";
            string sInitialLayerInfillSpeed = isFastSpeed ? "105" : "25";
            string sTravelSpeed = isFastSpeed ? "500" : "250";
            string sInitialLayerTravelSpeed = isFastSpeed ? "500" : "150";

            string nozzleTemp = isPla ? "205" : "240";
            string nozzleInitialTemp = isPla ? "215" : "235";
            string rangeHigh = isPla ? "230" : "250";
            string rangeLow = isPla ? "190" : "220";
            string bedTemp = isPla ? (isFastSpeed ? "65" : "60") : "75";
            string bedInitialTemp = isPla ? "65" : "80";
            string retLength = "1.0";
            string retSpeed = "45";
            string deretSpeed = "40";
            string wipeDist = "3";
            string fanMin = isPla ? "100" : "10";
            string fanMax = isPla ? "100" : "60";
            string flowRatio = "0.98";
            string maxVolSpeed = isPla ? "16" : "12";

            string layerHeightStr = layerHeight.ToString("F2", CultureInfo.InvariantCulture);
            string procInherits = "fdm_process_U1_" + layerHeightStr;
            string settingIdStr = (Math.Abs(layerHeight - 0.16) < 0.01) ? "GP107" : "GP004";
            string btmShell = (layerHeight >= 0.15) ? "4" : "5";
            string topShell = (Math.Abs(layerHeight - 0.16) < 0.01) ? "6" : "5";
            string topThick = (layerHeight >= 0.15) ? "1.0" : "0.6";
            string procName = isFastSpeed 
                ? string.Format("{0}mm Standard @Snapmaker U1 (0.4 nozzle)", layerHeightStr) 
                : string.Format("{0}mm Color Mixing @Snapmaker U1 (0.4 nozzle)", layerHeightStr);

            Action<string> updateProcess = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonVal(json, "name", procName);
                json = SetJsonVal(json, "print_settings_id", procName);
                json = SetJsonArray(json, "compatible_printers", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonVal(json, "from", "project");
                json = SetJsonVal(json, "inherits", procInherits);
                json = SetJsonVal(json, "layer_height", layerHeightStr);
                json = SetJsonVal(json, "initial_layer_print_height", "0.20");
                json = SetJsonVal(json, "bottom_shell_layers", btmShell);
                json = SetJsonVal(json, "top_shell_layers", topShell);
                json = SetJsonVal(json, "top_shell_thickness", topThick);
                json = SetJsonVal(json, "support_top_z_distance", layerHeightStr);
                json = SetJsonVal(json, "support_bottom_z_distance", layerHeightStr);
                json = SetJsonVal(json, "wall_generator", "classic");
                json = SetJsonVal(json, "detect_thin_wall", "0");
                json = SetJsonVal(json, "enable_arc_fitting", "0");
                json = SetJsonVal(json, "line_width", "0.42");
                json = SetJsonVal(json, "outer_wall_line_width", "0.42");
                json = SetJsonVal(json, "inner_wall_line_width", "0.45");
                json = SetJsonVal(json, "initial_layer_line_width", "0.50");
                json = SetJsonVal(json, "wall_loops", "2");
                json = SetJsonVal(json, "sparse_infill_density", "100%");
                json = SetJsonVal(json, "sparse_infill_pattern", "rectilinear");
                json = SetJsonVal(json, "outer_wall_speed", sOuterWallSpeed);
                json = SetJsonVal(json, "inner_wall_speed", sInnerWallSpeed);
                json = SetJsonVal(json, "small_perimeter_speed", sSmallPerimeterSpeed);
                json = SetJsonVal(json, "sparse_infill_speed", sSparseInfillSpeed);
                json = SetJsonVal(json, "top_surface_speed", sTopSurfaceSpeed);
                json = SetJsonVal(json, "initial_layer_speed", sInitialLayerSpeed);
                json = SetJsonVal(json, "initial_layer_infill_speed", sInitialLayerInfillSpeed);
                json = SetJsonVal(json, "travel_speed", sTravelSpeed);
                json = SetJsonVal(json, "initial_layer_travel_speed", sInitialLayerTravelSpeed);
                json = SetJsonVal(json, "reduce_infill_retraction", "0");
                json = SetJsonVal(json, "seam_position", "back");
                json = SetJsonVal(json, "wipe_on_loops", "1");
                json = SetJsonVal(json, "role_based_wipe_speed", "1");

                // Auto brim (Kullanıcı talebi: outer 2mm / 0.14 gap)
                json = SetJsonVal(json, "brim_type", "outer_only");
                json = SetJsonVal(json, "brim_width", "2");
                json = SetJsonVal(json, "brim_object_gap", "0.14");
                json = SetJsonVal(json, "enable_auto_brim", "0");
                json = SetJsonVal(json, "elefant_foot_compensation", "0");
                if (json.Contains("brim_object_gap;brim_width;"))
                {
                    json = json.Replace("brim_object_gap;brim_width;", "brim_object_gap;brim_type;brim_width;");
                }

                // Prime tower (Resmi Snapmaker U1 çoklu kafa: kaburgalı 30mm kule, 5mm brim, %120 aralık, 27mm3 prime)
                json = ApplyU1PrimeTowerProcessSettings(json, true);

                // İpliklenme ve duvar aşma önleme (Dış hatlardan kestirme geçişi engeller)
                json = SetJsonVal(json, "reduce_crossing_wall", "1");
                json = SetJsonVal(json, "max_travel_detour_distance", "30");
                File.WriteAllText(path, json, Encoding.UTF8);
            };

            Action<string> updateConfig = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonVal(json, "print_settings_id", procName);
                json = SetJsonVal(json, "name", procName);
                json = SetJsonVal(json, "setting_id", settingIdStr);
                json = SetJsonVal(json, "inherits", procInherits);
                json = SetJsonVal(json, "layer_height", layerHeightStr);
                json = SetJsonVal(json, "initial_layer_print_height", "0.20");
                json = SetJsonVal(json, "bottom_shell_layers", btmShell);
                json = SetJsonVal(json, "top_shell_layers", topShell);
                json = SetJsonVal(json, "top_shell_thickness", topThick);
                json = SetJsonVal(json, "support_top_z_distance", layerHeightStr);
                json = SetJsonVal(json, "support_bottom_z_distance", layerHeightStr);
                json = SetJsonVal(json, "wall_generator", "classic");
                json = SetJsonVal(json, "detect_thin_wall", "0");
                json = SetJsonVal(json, "enable_arc_fitting", "0");
                json = SetJsonVal(json, "line_width", "0.42");
                json = SetJsonVal(json, "outer_wall_line_width", "0.42");
                json = SetJsonVal(json, "inner_wall_line_width", "0.45");
                json = SetJsonVal(json, "initial_layer_line_width", "0.50");
                json = SetJsonVal(json, "wall_loops", "2");
                json = SetJsonVal(json, "sparse_infill_density", "100%");
                json = SetJsonVal(json, "sparse_infill_pattern", "rectilinear");
                json = SetJsonVal(json, "outer_wall_speed", sOuterWallSpeed);
                json = SetJsonVal(json, "inner_wall_speed", sInnerWallSpeed);
                json = SetJsonVal(json, "small_perimeter_speed", sSmallPerimeterSpeed);
                json = SetJsonVal(json, "sparse_infill_speed", sSparseInfillSpeed);
                json = SetJsonVal(json, "top_surface_speed", sTopSurfaceSpeed);
                json = SetJsonVal(json, "initial_layer_speed", sInitialLayerSpeed);
                json = SetJsonVal(json, "initial_layer_infill_speed", sInitialLayerInfillSpeed);
                json = SetJsonVal(json, "travel_speed", sTravelSpeed);
                json = SetJsonVal(json, "initial_layer_travel_speed", sInitialLayerTravelSpeed);
                json = SetJsonVal(json, "reduce_infill_retraction", "0");
                json = SetJsonVal(json, "seam_position", "back");
                json = SetJsonVal(json, "wipe_on_loops", "1");
                json = SetJsonVal(json, "role_based_wipe_speed", "1");

                // Auto brim (Kullanıcı talebi: outer 2mm / 0.14 gap)
                json = SetJsonVal(json, "brim_type", "outer_only");
                json = SetJsonVal(json, "brim_width", "2");
                json = SetJsonVal(json, "brim_object_gap", "0.14");
                json = SetJsonVal(json, "enable_auto_brim", "0");
                json = SetJsonVal(json, "elefant_foot_compensation", "0");

                // Prime tower (Resmi Snapmaker U1 çoklu kafa kulesi + çakışmasız yatak konumu)
                json = ApplyU1PrimeTowerProcessSettings(json, false);
                json = ApplyU1PrimeTowerMachineSettings(json);
                json = ApplyU1PrimeTowerFilamentSettings(json, 3, isPla);
                json = SetJsonArray(json, "wipe_tower_x", "25", "25");
                json = SetJsonArray(json, "wipe_tower_y", "195", "195");

                // İpliklenme ve duvar aşma önleme
                json = SetJsonVal(json, "reduce_crossing_wall", "1");
                json = SetJsonVal(json, "max_travel_detour_distance", "30");

                // TAM 3 Renk: 1=Renk1, 2=Renk2, 3=Renk3 (Kesinlikle 4. renk / sabit siyah YOK)
                json = SetJsonArray(json, "filament_colour", color1Hex, color2Hex, color3Hex);
                json = SetJsonArray(json, "filament_type", matType, matType, matType);
                json = SetJsonArray(json, "filament_settings_id", filProfileName, filProfileName, filProfileName);
                json = SetJsonArray(json, "nozzle_temperature", nozzleTemp, nozzleTemp, nozzleTemp);
                json = SetJsonArray(json, "nozzle_temperature_initial_layer", nozzleInitialTemp, nozzleInitialTemp, nozzleInitialTemp);
                json = SetJsonArray(json, "nozzle_temperature_range_high", rangeHigh, rangeHigh, rangeHigh);
                json = SetJsonArray(json, "nozzle_temperature_range_low", rangeLow, rangeLow, rangeLow);
                json = SetJsonArray(json, "hot_plate_temp", bedTemp, bedTemp, bedTemp);
                json = SetJsonArray(json, "hot_plate_temp_initial_layer", bedInitialTemp, bedInitialTemp, bedInitialTemp);
                json = SetJsonArray(json, "textured_plate_temp", bedTemp, bedTemp, bedTemp);
                json = SetJsonArray(json, "textured_plate_temp_initial_layer", bedInitialTemp, bedInitialTemp, bedInitialTemp);
                json = SetJsonArray(json, "filament_max_volumetric_speed", maxVolSpeed, maxVolSpeed, maxVolSpeed);
                json = SetJsonArray(json, "fan_min_speed", fanMin, fanMin, fanMin);
                json = SetJsonArray(json, "fan_max_speed", fanMax, fanMax, fanMax);
                json = SetJsonArray(json, "filament_flow_ratio", flowRatio, flowRatio, flowRatio);
                json = SetJsonArray(json, "filament_retraction_length", retLength, retLength, retLength);
                json = SetJsonArray(json, "filament_retraction_speed", retSpeed, retSpeed, retSpeed);
                json = SetJsonArray(json, "filament_deretraction_speed", deretSpeed, deretSpeed, deretSpeed);
                json = SetJsonArray(json, "filament_retract_when_changing_layer", "1", "1", "1");
                json = SetJsonArray(json, "filament_wipe", "1", "1", "1");
                json = SetJsonArray(json, "filament_wipe_distance", wipeDist, wipeDist, wipeDist);
                json = SetJsonArray(json, "retraction_length", retLength, retLength, retLength);
                json = SetJsonArray(json, "retraction_speed", retSpeed, retSpeed, retSpeed);
                json = SetJsonArray(json, "deretraction_speed", deretSpeed, deretSpeed, deretSpeed);
                json = SetJsonArray(json, "retract_before_wipe", "70%", "70%", "70%");
                json = SetJsonArray(json, "wipe", "1", "1", "1");
                json = SetJsonArray(json, "wipe_distance", wipeDist, wipeDist, wipeDist);
                json = SetJsonArray(json, "z_hop", "0", "0", "0");

                // Renk Geçiş Hacimleri (3x3 Flush & Purge Matrisi - Tam 3 Renk İçin Optimize Edildi)
                json = SetJsonArray(json, "flush_volumes_matrix", 
                    "0",   "180", "280",
                    "280", "0",   "280",
                    "220", "220", "0");
                json = SetJsonArray(json, "flush_volumes_vector", "180", "280", "220");
                json = SetJsonArray(json, "wiping_volumes_extruders", "180", "280", "220");
                json = SetJsonVal(json, "flush_multiplier", "1");

                File.WriteAllText(path, json, Encoding.UTF8);
            };

            Action<string> updateFilament = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonArray(json, "filament_settings_id", filProfileName);
                json = SetJsonVal(json, "name", filProfileName);
                json = SetJsonVal(json, "from", "project");
                json = SetJsonVal(json, "inherits", inheritsFil);
                json = SetJsonArray(json, "filament_type", matType);
                json = SetJsonArray(json, "filament_vendor", "Generic");
                json = SetJsonArray(json, "nozzle_temperature", nozzleTemp);
                json = SetJsonArray(json, "nozzle_temperature_initial_layer", nozzleInitialTemp);
                json = SetJsonArray(json, "nozzle_temperature_range_high", rangeHigh);
                json = SetJsonArray(json, "nozzle_temperature_range_low", rangeLow);
                json = SetJsonArray(json, "hot_plate_temp", bedTemp);
                json = SetJsonArray(json, "hot_plate_temp_initial_layer", bedInitialTemp);
                json = SetJsonArray(json, "textured_plate_temp", bedTemp);
                json = SetJsonArray(json, "textured_plate_temp_initial_layer", bedInitialTemp);
                json = SetJsonArray(json, "filament_max_volumetric_speed", maxVolSpeed);
                json = SetJsonArray(json, "fan_min_speed", fanMin);
                json = SetJsonArray(json, "fan_max_speed", fanMax);
                json = SetJsonArray(json, "filament_flow_ratio", flowRatio);
                json = SetJsonArray(json, "filament_retraction_length", retLength);
                json = SetJsonArray(json, "filament_retraction_speed", retSpeed);
                json = SetJsonArray(json, "filament_deretraction_speed", deretSpeed);
                json = SetJsonArray(json, "filament_retract_when_changing_layer", "1");
                json = SetJsonArray(json, "filament_wipe", "1");
                json = SetJsonArray(json, "filament_wipe_distance", wipeDist);
                json = SetJsonArray(json, "filament_retract_before_wipe", "70%");
                json = SetJsonArray(json, "filament_z_hop", "0");
                json = ApplyU1PrimeTowerFilamentSettings(json, 1, isPla);
                File.WriteAllText(path, json, Encoding.UTF8);
            };

            Action<string> updateMachine = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonVal(json, "name", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonVal(json, "printer_model", "Snapmaker U1");
                json = SetJsonVal(json, "printer_settings_id", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonVal(json, "from", "project");
                json = SetJsonVal(json, "default_print_profile", procName);
                json = SetJsonArray(json, "compatible_printers", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonArray(json, "default_filament_profile", filProfileName, filProfileName, filProfileName);
                json = SetJsonArray(json, "retraction_length", retLength, retLength, retLength, retLength);
                json = SetJsonArray(json, "retraction_speed", retSpeed, retSpeed, retSpeed, retSpeed);
                json = SetJsonArray(json, "deretraction_speed", deretSpeed, deretSpeed, deretSpeed, deretSpeed);
                json = SetJsonArray(json, "retract_before_wipe", "70%", "70%", "70%", "70%");
                json = SetJsonArray(json, "wipe", "1", "1", "1", "1");
                json = SetJsonArray(json, "wipe_distance", wipeDist, wipeDist, wipeDist, wipeDist);
                json = SetJsonArray(json, "z_hop", "0", "0", "0", "0");
                json = SetJsonArray(json, "retract_when_changing_layer", "1", "1", "1", "1");
                json = ApplyU1PrimeTowerMachineSettings(json);
                File.WriteAllText(path, json, Encoding.UTF8);
            };

            updateProcess(Path.Combine(tempDir, "Metadata", "process_settings_1.config"));
            updateConfig(Path.Combine(tempDir, "Metadata", "project_settings.config"));
            updateFilament(Path.Combine(tempDir, "Metadata", "filament_settings_1.config"));
            updateFilament(Path.Combine(tempDir, "Metadata", "filament_settings_2.config"));

            // 3. filament config oluştur (Varsa 1'den kopyala ve güncelle)
            string fil1Path = Path.Combine(tempDir, "Metadata", "filament_settings_1.config");
            string fil3Path = Path.Combine(tempDir, "Metadata", "filament_settings_3.config");
            if (File.Exists(fil1Path))
            {
                File.Copy(fil1Path, fil3Path, true);
            }
            updateFilament(fil3Path);
            updateMachine(Path.Combine(tempDir, "Metadata", "machine_settings_1.config"));

            // 4..16 gereksiz filamentleri sil (Projeyi TAM 3 RENK ile sınırla)
            for (int fi = 4; fi <= 16; fi++)
            {
                string extraFil = Path.Combine(tempDir, "Metadata", string.Format("filament_settings_{0}.config", fi));
                if (File.Exists(extraFil)) File.Delete(extraFil);
            }

            // JSON Metadata kaydet
            RingConfig seqCfg = new RingConfig();
            seqCfg.Mode = GeneratorMode.SiraliKlips;
            seqCfg.StartNum = startNum;
            seqCfg.EndNum = endNum;
            seqCfg.IncludeGender = includeGender;
            seqCfg.SeqFont = fontName;
            seqCfg.SeqTextHeightMm = textHeightWorld;
            seqCfg.SeqStretch = stretchY;
            seqCfg.SeqCharGapMm = charGapMm;
            seqCfg.SeqWidthScale = widthScale;
            seqCfg.SeqLayerHeight = layerHeight;
            seqCfg.EmbossMm = embossMm;
            seqCfg.EmbedMm = embedMm;
            seqCfg.Material = material;
            seqCfg.Color1Hex = color1Hex;
            seqCfg.Color2Hex = color2Hex;
            seqCfg.CreatedDate = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            string configJsonPath = Path.Combine(tempDir, "Metadata", "pigeon_ring_config.json");
            File.WriteAllText(configJsonPath, seqCfg.ToJson(), Encoding.UTF8);

            // Ensure _rels/.rels
            string rootRelsPath = Path.Combine(tempDir, "_rels", ".rels");
            Directory.CreateDirectory(Path.GetDirectoryName(rootRelsPath));
            using (StreamWriter sw = new StreamWriter(rootRelsPath, false, Encoding.UTF8))
            {
                sw.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
                sw.WriteLine("<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">");
                sw.WriteLine(" <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>");
                sw.WriteLine("</Relationships>");
            }

            // Özel font göm
            try
            {
                string metaFontsDir = Path.Combine(tempDir, "Metadata", "Fonts");
                string fontFile = FontManager.GetFontFilePath(fontName);
                if (!string.IsNullOrEmpty(fontFile) && File.Exists(fontFile))
                {
                    Directory.CreateDirectory(metaFontsDir);
                    string dest = Path.Combine(metaFontsDir, Path.GetFileName(fontFile));
                    File.Copy(fontFile, dest, true);
                    fullLog(string.Format("  -> Özel font 3MF içine gömüldü: {0}", Path.GetFileName(fontFile)));
                }
            }
            catch { }

            // ZIP paketi oluştur
            CleanResidualMetadata(tempDir);

            if (File.Exists(output3mf)) File.Delete(output3mf);
            CreateZipArchive(tempDir, output3mf);

            // Bilgi raporu kaydet
            try
            {
                string infoPath = Path.Combine(Path.GetDirectoryName(output3mf), Path.GetFileNameWithoutExtension(output3mf) + "_bilgi.txt");
                File.WriteAllText(infoPath, seqCfg.ToReportText(Path.GetFileName(output3mf)), Encoding.UTF8);
                fullLog(string.Format("Ayar ve Bilgi Raporu Kaydedildi: {0}", Path.GetFileName(infoPath)));
            }
            catch (Exception exInfo)
            {
                fullLog("Bilgi raporu kaydedilemedi: " + exInfo.Message);
            }

            try { Directory.Delete(tempDir, true); } catch { }

            fullLog(string.Format("BAŞARILI: '{0}' multi-plate sıralı projesi kusursuz oluşturuldu!\n", output3mf));
            return true;
        }
        catch (Exception ex)
        {
            fullLog(string.Format("KRİTİK HATA: {0}\n{1}", ex.Message, ex.StackTrace));
            return false;
        }
    }

    // ==========================================
    // 6. CORE 3MF GENERATION ENGINE (İSİMLİ KLİPS & MARKA TİPİ)
    // ==========================================
    public static bool ExecuteGeneration(string topText, string botText, string backup3mf, string output3mf, 
                                         double embossMm, double embedMm, 
                                         string fontTop, string fontBot, 
                                         double textHeightTopMm, double textHeightBotMm, 
                                         double charGapTopMm, double charGapBotMm, 
                                         double widthScaleTop, double widthScaleBot,
                                         Action<string> log, string material = "PLA",
                                         string color1Hex = "#FFFF00", string color2Hex = "#FF0000",
                                         double stretchTop = 100.0, double stretchBot = 100.0,
                                         bool isMarkaTipi = false,
                                         string speedProfile = "fast",
                                         bool useTurkishFix = true)
    {
        string logPath = Path.Combine(GetAppDataDir(), "generator.log");
        Action<string> fullLog = s =>
        {
            try { File.AppendAllText(logPath, s + "\r\n", Encoding.UTF8); } catch { }
            if (log != null) log(s);
            try { Console.WriteLine(s); } catch { }
        };

        try
        {
            if (string.IsNullOrEmpty(fontTop)) fontTop = "Trebuchet MS";
            if (string.IsNullOrEmpty(fontBot)) fontBot = fontTop;

            bool isPla = string.Equals(material, "PLA", StringComparison.OrdinalIgnoreCase);
            string matType = isPla ? "PLA" : "PETG";
            bool isFastSpeed = string.Equals(speedProfile, "fast", StringComparison.OrdinalIgnoreCase) || string.IsNullOrEmpty(speedProfile);

            topText = ConvertSpecialTurkishUpperLetters(topText, useTurkishFix);
            botText = ConvertSpecialTurkishUpperLetters(botText, useTurkishFix);
            string dOnly = "";
            foreach (char c in botText) if (char.IsDigit(c)) dOnly += c;
            if (dOnly.Length == 10 || dOnly.Length == 11 || (dOnly.Length >= 12 && dOnly.StartsWith("90")) || dOnly.Length == 15)
            {
                botText = FormatPhoneNumber(dOnly);
            }

            fullLog(string.Format("=== {0} Çift Tablalı 3MF Üretici Başlatıldı ===", isMarkaTipi ? "MARKA TİPİ (Dairesel Halka)" : "GÜVERCİN İSİMLİ KLİPS"));
            fullLog(string.Format("Tarih/Saat: {0}", DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss")));
            fullLog(string.Format("Seçilen Malzeme (Filament): {0}", matType));
            fullLog(string.Format("Baskı Renkleri (Tam 2 Renk): 1={0} (Renk 1) | 2={1} (Renk 2)", color1Hex, color2Hex));
            fullLog("Hedef: 2 Ayrı Tabla (Tabla 1: Düz Renk • Tabla 2: Çift Renk)");
            fullLog(string.Format("Baskı Hız Profili: {0} ({1})", isFastSpeed ? "HIZLI BASKI" : "YAVAŞ BASKI", isFastSpeed ? "Standart Hızlar: 50 / 80 / 300 mm/s" : "Güvenli İlk Katman: 18 / 30 / 45 mm/s"));
            fullLog(string.Format("Üst Yazı: '{0}' (Font: {1}, Boy: {2:F2}mm, Dikey: %{3:0}, Gap: {4:F2}mm, Boldness: {5:F2}mm)", topText, fontTop, textHeightTopMm, stretchTop, charGapTopMm, widthScaleTop));
            fullLog(string.Format("Alt Yazı: '{0}' (Font: {1}, Boy: {2:F2}mm, Dikey: %{3:0}, Gap: {4:F2}mm, Boldness: {5:F2}mm)", botText, fontBot, textHeightBotMm, stretchBot, charGapBotMm, widthScaleBot));
            fullLog(string.Format("Geometri: Emboss={0:F2}mm, Embed={1:F2}mm, Toplam Yükseklik=9.00mm (Z=0.00mm Tabla)", embossMm, embedMm));

            if (!File.Exists(backup3mf))
            {
                fullLog(string.Format("HATA: Şablon dosya bulunamadı: {0}", backup3mf));
                return false;
            }

            RingConfig currentConfig = new RingConfig();
            currentConfig.Mode = isMarkaTipi ? GeneratorMode.MarkaTipi : GeneratorMode.IsimliKlips;
            currentConfig.TopText = topText;
            currentConfig.BotText = botText;
            currentConfig.FontTop = fontTop;
            currentConfig.FontBot = fontBot;
            currentConfig.TextHeightTopMm = textHeightTopMm;
            currentConfig.CharGapTopMm = charGapTopMm;
            currentConfig.WidthScaleTop = widthScaleTop;
            currentConfig.StretchTop = stretchTop;
            currentConfig.TextHeightBotMm = textHeightBotMm;
            currentConfig.CharGapBotMm = charGapBotMm;
            currentConfig.WidthScaleBot = widthScaleBot;
            currentConfig.StretchBot = stretchBot;
            currentConfig.EmbossMm = embossMm;
            currentConfig.EmbedMm = embedMm;
            currentConfig.Material = matType;
            currentConfig.SpeedProfile = isFastSpeed ? "fast" : "slow";
            currentConfig.Color1Hex = color1Hex;
            currentConfig.Color2Hex = color2Hex;
            currentConfig.CreatedDate = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            // Extract template ring XML
            fullLog("[1/5] Şablon dosyasından halka geometrisi okunuyor...");
            string ringXml = "";
            using (ZipArchive zip = ZipFile.OpenRead(backup3mf))
            {
                foreach (var e in zip.Entries)
                {
                    if (e.FullName.EndsWith(".model", StringComparison.OrdinalIgnoreCase) && !e.FullName.Contains("3dmodel.model"))
                    {
                        using (StreamReader sr = new StreamReader(e.Open(), Encoding.UTF8))
                        {
                            ringXml = sr.ReadToEnd();
                        }
                        break;
                    }
                }
                if (string.IsNullOrEmpty(ringXml))
                {
                    var mEntry = zip.GetEntry("3D/3dmodel.model");
                    if (mEntry != null)
                    {
                        using (StreamReader sr = new StreamReader(mEntry.Open(), Encoding.UTF8))
                        {
                            ringXml = sr.ReadToEnd();
                        }
                    }
                }
            }

            if (string.IsNullOrEmpty(ringXml))
            {
                fullLog("HATA: Şablon zip içinde halka model XML verisi bulunamadı.");
                return false;
            }

            // Ham tek parça gövdeyi oku, dinamik Z ölçeğini hesapla ve Use-Surface örnekleyici kur
            SolidMesh wholeRing = ParseMeshFromModelXml(ringXml);
            double origMinZ = double.MaxValue, origMaxZ = double.MinValue;
            foreach (var v in wholeRing.Vertices) { if (v.z < origMinZ) origMinZ = v.z; if (v.z > origMaxZ) origMaxZ = v.z; }
            double origH = (origMaxZ > origMinZ) ? (origMaxZ - origMinZ) : (isMarkaTipi ? 8.000 : 7.168);
            double splitZ = (origMinZ + origMaxZ) * 0.5;
            double dynZScale = 9.000 / origH;
            double zHalfLocal = origH * 0.5;
            double centerZTopLocal = splitZ + zHalfLocal * 0.5;
            double centerZBotLocal = splitZ - zHalfLocal * 0.5;
            double compTz = isMarkaTipi ? 0.0 : -0.616014361;
            double isimPosZ = -(origMinZ + compTz) * dynZScale;

            RingSurfaceSampler surfaceSampler = new RingSurfaceSampler(wholeRing, 0.0, isMarkaTipi ? 0.0 : -0.2754);

            fullLog("[2/5] Tek parça 9 mm gövde korunuyor (bölme yok)...");
            fullLog(string.Format("  -> Tek Parça Gövde: {0} köşe, {1} üçgen | Z=[{2:F3}, {3:F3}] (dynZScale={4:F6})", wholeRing.Vertices.Count, wholeRing.Triangles.Count, origMinZ, origMaxZ, dynZScale));

            // Generate cylindrical text meshes (Use-Surface + <= 0.15mm Per-Glyph)
            fullLog("[3/5] Use-Surface silindirik kabartma yazı geometrisi üretiliyor...");
            SolidMesh textTop, textBot;

            if (isMarkaTipi)
            {
                const double maxSpanMarka = 310.0;
                const double centerAngleMarka = 234.082;
                textTop = GenerateDistributedCylindricalText(topText, fontTop, textHeightTopMm, centerZTopLocal, centerAngleMarka, embedMm, embossMm, dynZScale, charGapTopMm, 0, widthScaleTop, maxSpanMarka, stretchTop / 100.0, true, 4.50, 0.0, useTurkishFix, surfaceSampler);
                textBot = GenerateDistributedCylindricalText(botText, fontBot, textHeightBotMm, centerZBotLocal, centerAngleMarka, embedMm, embossMm, dynZScale, charGapBotMm, 0, widthScaleBot, maxSpanMarka, stretchBot / 100.0, true, 4.50, 0.0, useTurkishFix, surfaceSampler);
            }
            else
            {
                const double maxSpanIsimli = 258.0;
                const double centerAngleIsimli = 236.0;
                textTop = GenerateDistributedCylindricalText(topText, fontTop, textHeightTopMm, centerZTopLocal, centerAngleIsimli, embedMm, embossMm, dynZScale, charGapTopMm, 0, widthScaleTop, maxSpanIsimli, stretchTop / 100.0, false, 0.0, 0.0, useTurkishFix, surfaceSampler);
                textBot = GenerateDistributedCylindricalText(botText, fontBot, textHeightBotMm, centerZBotLocal, centerAngleIsimli, embedMm, embossMm, dynZScale, charGapBotMm, 0, widthScaleBot, maxSpanIsimli, stretchBot / 100.0, false, 0.0, 0.0, useTurkishFix, surfaceSampler);
            }

            double zMinTop = double.MaxValue, zMaxTop = double.MinValue;
            foreach (var v in textTop.Vertices) { if (v.z < zMinTop) zMinTop = v.z; if (v.z > zMaxTop) zMaxTop = v.z; }
            fullLog(string.Format("  -> DEBUG textTop Z bounds: min={0:F3}, max={1:F3}, center={2:F3}", zMinTop, zMaxTop, (zMinTop + zMaxTop) * 0.5));
            double zMinBot = double.MaxValue, zMaxBot = double.MinValue;
            foreach (var v in textBot.Vertices) { if (v.z < zMinBot) zMinBot = v.z; if (v.z > zMaxBot) zMaxBot = v.z; }
            fullLog(string.Format("  -> DEBUG textBot Z bounds: min={0:F3}, max={1:F3}, center={2:F3}", zMinBot, zMaxBot, (zMinBot + zMaxBot) * 0.5));

            fullLog(string.Format("  -> Üst Yazı '{0}': {1} köşe, {2} üçgen", topText, textTop.Vertices.Count, textTop.Triangles.Count));
            fullLog(string.Format("  -> Alt Yazı '{0}': {1} köşe, {2} üçgen", botText, textBot.Vertices.Count, textBot.Triangles.Count));

            // Setup temporary build directory
            fullLog("[4/5] Multi-plate (Çift Tabla) 3MF proje yapısı inşa ediliyor...");
            string tempDir = Path.Combine(Path.GetTempPath(), "pigeon_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(tempDir);

            using (ZipArchive zip = ZipFile.OpenRead(backup3mf))
            {
                foreach (var e in zip.Entries)
                {
                    if (string.IsNullOrEmpty(e.Name) || e.FullName.EndsWith("/") || e.FullName.EndsWith(@"\"))
                        continue;
                    if (e.FullName.StartsWith("3D/Objects/", StringComparison.OrdinalIgnoreCase) ||
                        e.FullName.StartsWith("3D/_rels/", StringComparison.OrdinalIgnoreCase))
                        continue;

                    string destPath = Path.Combine(tempDir, e.FullName.Replace('/', Path.DirectorySeparatorChar));
                    Directory.CreateDirectory(Path.GetDirectoryName(destPath));
                    e.ExtractToFile(destPath, true);
                }
            }

            // Write 3D/3dmodel.model: Object 1=ring_whole, 2=ring_lower, 3=ring_upper, 4=text_top, 5=text_bot, 6=Plate1, 7=Plate2
            string model3DPath = Path.Combine(tempDir, "3D", "3dmodel.model");
            using (StreamWriter sw = new StreamWriter(model3DPath, false, Encoding.UTF8))
            {
                sw.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
                sw.WriteLine("<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">");
                sw.WriteLine(" <metadata name=\"Application\">Snapmaker_Orca-1.10.1</metadata>");
                sw.WriteLine(" <metadata name=\"Snapmaker_Orca:3mfVersion\">1</metadata>");
                sw.WriteLine(string.Format(" <metadata name=\"PigeonRingConfig\">{0}</metadata>", System.Net.WebUtility.HtmlEncode(currentConfig.ToJson())));
                sw.WriteLine(" <resources>");

                // Object 1: ring_whole (Kesintisiz tek parça 9 mm gövde)
                sw.WriteLine("  <object id=\"1\" type=\"model\">");
                sw.WriteLine("   <mesh>");
                sw.WriteLine("    <vertices>");
                foreach (var v in wholeRing.Vertices)
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "     <vertex x=\"{0:F6}\" y=\"{1:F6}\" z=\"{2:F6}\"/>", v.x, v.y, v.z));
                sw.WriteLine("    </vertices>");
                sw.WriteLine("    <triangles>");
                foreach (var t in wholeRing.Triangles)
                    sw.WriteLine(string.Format("     <triangle v1=\"{0}\" v2=\"{1}\" v3=\"{2}\"/>", t.v1, t.v2, t.v3));
                sw.WriteLine("    </triangles>");
                sw.WriteLine("   </mesh>");
                sw.WriteLine("  </object>");

                // Object 4: text_top
                sw.WriteLine("  <object id=\"4\" type=\"model\">");
                sw.WriteLine("   <mesh>");
                sw.WriteLine("    <vertices>");
                foreach (var v in textTop.Vertices)
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "     <vertex x=\"{0:F6}\" y=\"{1:F6}\" z=\"{2:F6}\"/>", v.x, v.y, v.z));
                sw.WriteLine("    </vertices>");
                sw.WriteLine("    <triangles>");
                foreach (var t in textTop.Triangles)
                    sw.WriteLine(string.Format("     <triangle v1=\"{0}\" v2=\"{1}\" v3=\"{2}\"/>", t.v1, t.v2, t.v3));
                sw.WriteLine("    </triangles>");
                sw.WriteLine("   </mesh>");
                sw.WriteLine("  </object>");

                // Object 5: text_bot
                sw.WriteLine("  <object id=\"5\" type=\"model\">");
                sw.WriteLine("   <mesh>");
                sw.WriteLine("    <vertices>");
                foreach (var v in textBot.Vertices)
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "     <vertex x=\"{0:F6}\" y=\"{1:F6}\" z=\"{2:F6}\"/>", v.x, v.y, v.z));
                sw.WriteLine("    </vertices>");
                sw.WriteLine("    <triangles>");
                foreach (var t in textBot.Triangles)
                    sw.WriteLine(string.Format("     <triangle v1=\"{0}\" v2=\"{1}\" v3=\"{2}\"/>", t.v1, t.v2, t.v3));
                sw.WriteLine("    </triangles>");
                sw.WriteLine("   </mesh>");
                sw.WriteLine("  </object>");

                string compTransStr = isMarkaTipi
                    ? "1 0 0 0 1 0 0 0 1 0 0 0"
                    : string.Format(CultureInfo.InvariantCulture, "1 0 0 0 1 0 0 0 1 -0.237974882 -0.0705842972 {0:F9}", compTz);

                // Object 6: Assembly (Tabla 1 - Düz Renk, Tek Parça Gövde)
                sw.WriteLine("  <object id=\"6\" type=\"model\">");
                sw.WriteLine("   <components>");
                sw.WriteLine(string.Format("    <component objectid=\"1\" transform=\"{0}\"/>", compTransStr));
                sw.WriteLine(string.Format("    <component objectid=\"4\" transform=\"{0}\"/>", compTransStr));
                sw.WriteLine(string.Format("    <component objectid=\"5\" transform=\"{0}\"/>", compTransStr));
                sw.WriteLine("   </components>");
                sw.WriteLine("  </object>");

                // Object 7: Assembly (Tabla 2 - Tek Parça 9 mm Gövde)
                sw.WriteLine("  <object id=\"7\" type=\"model\">");
                sw.WriteLine("   <components>");
                sw.WriteLine(string.Format("    <component objectid=\"1\" transform=\"{0}\"/>", compTransStr));
                sw.WriteLine(string.Format("    <component objectid=\"4\" transform=\"{0}\"/>", compTransStr));
                sw.WriteLine(string.Format("    <component objectid=\"5\" transform=\"{0}\"/>", compTransStr));
                sw.WriteLine("   </components>");
                sw.WriteLine("  </object>");
                sw.WriteLine(" </resources>");

                // Build Section: Tabla 1 (X ~135) ve Tabla 2 (X ~485)
                sw.WriteLine(" <build>");
                if (isMarkaTipi)
                {
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "  <item objectid=\"6\" transform=\"1.01230882 0.733301334 0 -0.733301334 1.01230882 0 0 0 {0:F6} 135.5 136.000001 {1:F6}\" printable=\"1\"/>", dynZScale, isimPosZ));
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "  <item objectid=\"7\" transform=\"1.01230882 0.733301334 0 -0.733301334 1.01230882 0 0 0 {0:F6} 485.5 136.000001 {1:F6}\" printable=\"1\"/>", dynZScale, isimPosZ));
                }
                else
                {
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "  <item objectid=\"6\" transform=\"0.99526140220630832 0.09723549392239929 0 -0.09723549392239929 0.99526140220630832 0 0 0 {0:F6} 160.40744893373503 135.99137609914587 {1:F8}\" printable=\"1\"/>", dynZScale, isimPosZ));
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "  <item objectid=\"7\" transform=\"0.99526140220630832 0.09723549392239929 0 -0.09723549392239929 0.99526140220630832 0 0 0 {0:F6} 510.40744893373503 135.99137609914587 {1:F8}\" printable=\"1\"/>", dynZScale, isimPosZ));
                }
                sw.WriteLine(" </build>");
                sw.WriteLine("</model>");
            }

            // Update Metadata/model_settings.config
            string modelSettingsPath = Path.Combine(tempDir, "Metadata", "model_settings.config");
            using (StreamWriter sw = new StreamWriter(modelSettingsPath, false, Encoding.UTF8))
            {
                sw.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
                sw.WriteLine("<config>");

                // Object 6: Tabla 1 - Düz Renk (Tek Parça Gövde id=1 -> Ext 1, Yazılar id=4,5 -> Ext 2)
                sw.WriteLine("  <object id=\"6\">");
                sw.WriteLine(string.Format("    <metadata key=\"name\" value=\"{0}_DuzRenk\"/>", isMarkaTipi ? "MarkaTipi" : "PigeonRing"));
                sw.WriteLine("    <part id=\"1\" subtype=\"normal_part\">");
                sw.WriteLine("      <metadata key=\"name\" value=\"ring_whole\"/>");
                sw.WriteLine("      <metadata key=\"extruder\" value=\"1\"/>");
                sw.WriteLine("    </part>");
                sw.WriteLine("    <part id=\"4\" subtype=\"normal_part\">");
                sw.WriteLine("      <metadata key=\"name\" value=\"text_top\"/>");
                sw.WriteLine("      <metadata key=\"extruder\" value=\"2\"/>");
                sw.WriteLine("    </part>");
                sw.WriteLine("    <part id=\"5\" subtype=\"normal_part\">");
                sw.WriteLine("      <metadata key=\"name\" value=\"text_bot\"/>");
                sw.WriteLine("      <metadata key=\"extruder\" value=\"2\"/>");
                sw.WriteLine("    </part>");
                sw.WriteLine("  </object>");

                // Object 7: Tabla 2 - Tek Parça 9 mm Gövde (id=1 -> Ext 1, Üst Yazı id=4 -> Ext 2, Alt Yazı id=5 -> Ext 2)
                sw.WriteLine("  <object id=\"7\">");
                sw.WriteLine(string.Format("    <metadata key=\"name\" value=\"{0}_CiftRenk\"/>", isMarkaTipi ? "MarkaTipi" : "PigeonRing"));
                sw.WriteLine("    <part id=\"1\" subtype=\"normal_part\">");
                sw.WriteLine("      <metadata key=\"name\" value=\"ring_whole\"/>");
                sw.WriteLine("      <metadata key=\"extruder\" value=\"1\"/>");
                sw.WriteLine("    </part>");
                sw.WriteLine("    <part id=\"4\" subtype=\"normal_part\">");
                sw.WriteLine("      <metadata key=\"name\" value=\"text_top\"/>");
                sw.WriteLine("      <metadata key=\"extruder\" value=\"2\"/>");
                sw.WriteLine("    </part>");
                sw.WriteLine("    <part id=\"5\" subtype=\"normal_part\">");
                sw.WriteLine("      <metadata key=\"name\" value=\"text_bot\"/>");
                sw.WriteLine("      <metadata key=\"extruder\" value=\"2\"/>");
                sw.WriteLine("    </part>");
                sw.WriteLine("  </object>");

                sw.WriteLine("  <plate>");
                sw.WriteLine("    <metadata key=\"plater_id\" value=\"1\"/>");
                sw.WriteLine("    <metadata key=\"plater_name\" value=\"1-Duz Renk\"/>");
                sw.WriteLine("    <metadata key=\"locked\" value=\"false\"/>");
                sw.WriteLine("    <metadata key=\"filament_maps\" value=\"1 2\"/>");
                sw.WriteLine("    <metadata key=\"thumbnail_file\" value=\"Metadata/plate_1.png\"/>");
                sw.WriteLine("    <metadata key=\"top_file\" value=\"Metadata/top_1.png\"/>");
                sw.WriteLine("    <metadata key=\"pick_file\" value=\"Metadata/pick_1.png\"/>");
                sw.WriteLine("    <model_instance>");
                sw.WriteLine("      <metadata key=\"object_id\" value=\"6\"/>");
                sw.WriteLine("      <metadata key=\"instance_id\" value=\"0\"/>");
                sw.WriteLine("      <metadata key=\"identify_id\" value=\"444\"/>");
                sw.WriteLine("    </model_instance>");
                sw.WriteLine("  </plate>");

                sw.WriteLine("  <plate>");
                sw.WriteLine("    <metadata key=\"plater_id\" value=\"2\"/>");
                sw.WriteLine("    <metadata key=\"plater_name\" value=\"2-Cift Renk\"/>");
                sw.WriteLine("    <metadata key=\"locked\" value=\"false\"/>");
                sw.WriteLine("    <metadata key=\"filament_maps\" value=\"1 2\"/>");
                sw.WriteLine("    <metadata key=\"thumbnail_file\" value=\"Metadata/plate_1.png\"/>");
                sw.WriteLine("    <metadata key=\"top_file\" value=\"Metadata/top_1.png\"/>");
                sw.WriteLine("    <metadata key=\"pick_file\" value=\"Metadata/pick_1.png\"/>");
                sw.WriteLine("    <model_instance>");
                sw.WriteLine("      <metadata key=\"object_id\" value=\"7\"/>");
                sw.WriteLine("      <metadata key=\"instance_id\" value=\"0\"/>");
                sw.WriteLine("      <metadata key=\"identify_id\" value=\"445\"/>");
                sw.WriteLine("    </model_instance>");
                sw.WriteLine("  </plate>");

                sw.WriteLine("  <assemble>");
                if (isMarkaTipi)
                {
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "   <assemble_item object_id=\"6\" instance_id=\"0\" transform=\"1.01230882 0.733301334 0 -0.733301334 1.01230882 0 0 0 {0:F6} 135.5 136.000001 {1:F6}\" offset=\"0 0 0\" />", dynZScale, isimPosZ));
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "   <assemble_item object_id=\"7\" instance_id=\"0\" transform=\"1.01230882 0.733301334 0 -0.733301334 1.01230882 0 0 0 {0:F6} 485.5 136.000001 {1:F6}\" offset=\"0 0 0\" />", dynZScale, isimPosZ));
                }
                else
                {
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "   <assemble_item object_id=\"6\" instance_id=\"0\" transform=\"0.99526140220630832 0.09723549392239929 0 -0.09723549392239929 0.99526140220630832 0 0 0 {0:F6} 160.40744893373503 135.99137609914587 {1:F8}\" offset=\"0 4.7683715799999999e-07 0\" />", dynZScale, isimPosZ));
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "   <assemble_item object_id=\"7\" instance_id=\"0\" transform=\"0.99526140220630832 0.09723549392239929 0 -0.09723549392239929 0.99526140220630832 0 0 0 {0:F6} 510.40744893373503 135.99137609914587 {1:F8}\" offset=\"0 4.7683715799999999e-07 0\" />", dynZScale, isimPosZ));
                }
                sw.WriteLine("  </assemble>");
                sw.WriteLine("</config>");
            }

            // Apply optimal process settings inside 3MF (STRICTLY 2 FILAMENTS, COMPACT 22MM PRIME TOWER)
            fullLog(string.Format("[5/5] Dilimleme parametreleri ({0}) ve 2 renkli {1} profili uygulanıyor...", isFastSpeed ? "HIZLI BASKI" : "YAVAŞ BASKI", matType));
            string filProfileName = string.Format("Generic {0} (Klips)({1})", matType, Path.GetFileName(output3mf));
            string inheritsFil = isPla ? "Generic PLA" : "Generic PETG";

            string sOuterWallSpeed = isFastSpeed ? "80" : "30";
            string sInnerWallSpeed = isFastSpeed ? "300" : "45";
            string sSmallPerimeterSpeed = isFastSpeed ? "50" : "20";
            string sSparseInfillSpeed = isFastSpeed ? "270" : "50";
            string sTopSurfaceSpeed = isFastSpeed ? "200" : "30";
            string sInitialLayerSpeed = isFastSpeed ? "50" : "18";
            string sInitialLayerInfillSpeed = isFastSpeed ? "105" : "25";
            string sTravelSpeed = isFastSpeed ? "500" : "250";
            string sInitialLayerTravelSpeed = isFastSpeed ? "500" : "150";

            string nozzleTemp = isPla ? "205" : "240";
            string nozzleInitialTemp = isPla ? "215" : "235";
            string rangeHigh = isPla ? "230" : "250";
            string rangeLow = isPla ? "190" : "220";
            string bedTemp = isPla ? (isFastSpeed ? "65" : "60") : "75";
            string bedInitialTemp = isPla ? "65" : "80";
            string retLength = "1.0";
            string retSpeed = "45";
            string deretSpeed = "40";
            string wipeDist = "3";
            string fanMin = isPla ? "100" : "10";
            string fanMax = isPla ? "100" : "60";
            string flowRatio = "0.98";
            string maxVolSpeed = isPla ? "16" : "12";

            double layerHeight = 0.12;
            string layerHeightStr = layerHeight.ToString("F2", CultureInfo.InvariantCulture);
            string procInherits = "fdm_process_U1_" + layerHeightStr;
            string settingIdStr = "GP004";
            string btmShell = "5";
            string topShell = "5";
            string topThick = "0.6";
            string procName = isFastSpeed 
                ? string.Format("{0}mm Standard @Snapmaker U1 (0.4 nozzle)", layerHeightStr) 
                : string.Format("{0}mm Color Mixing @Snapmaker U1 (0.4 nozzle)", layerHeightStr);

            Action<string> updateConfig = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonVal(json, "print_settings_id", procName);
                json = SetJsonVal(json, "name", procName);
                json = SetJsonVal(json, "setting_id", settingIdStr);
                json = SetJsonVal(json, "inherits", procInherits);
                json = SetJsonVal(json, "layer_height", layerHeightStr);
                json = SetJsonVal(json, "initial_layer_print_height", "0.20");
                json = SetJsonVal(json, "bottom_shell_layers", btmShell);
                json = SetJsonVal(json, "top_shell_layers", topShell);
                json = SetJsonVal(json, "top_shell_thickness", topThick);
                json = SetJsonVal(json, "support_top_z_distance", layerHeightStr);
                json = SetJsonVal(json, "support_bottom_z_distance", layerHeightStr);
                json = SetJsonVal(json, "wall_generator", "classic");
                json = SetJsonVal(json, "detect_thin_wall", "0");
                json = SetJsonVal(json, "enable_arc_fitting", "0");
                json = SetJsonVal(json, "line_width", "0.42");
                json = SetJsonVal(json, "outer_wall_line_width", "0.42");
                json = SetJsonVal(json, "inner_wall_line_width", "0.45");
                json = SetJsonVal(json, "initial_layer_line_width", "0.50");
                json = SetJsonVal(json, "wall_loops", "2");
                json = SetJsonVal(json, "sparse_infill_density", "100%");
                json = SetJsonVal(json, "sparse_infill_pattern", "rectilinear");
                json = SetJsonVal(json, "outer_wall_speed", sOuterWallSpeed);
                json = SetJsonVal(json, "inner_wall_speed", sInnerWallSpeed);
                json = SetJsonVal(json, "small_perimeter_speed", sSmallPerimeterSpeed);
                json = SetJsonVal(json, "sparse_infill_speed", sSparseInfillSpeed);
                json = SetJsonVal(json, "top_surface_speed", sTopSurfaceSpeed);
                json = SetJsonVal(json, "initial_layer_speed", sInitialLayerSpeed);
                json = SetJsonVal(json, "initial_layer_infill_speed", sInitialLayerInfillSpeed);
                json = SetJsonVal(json, "travel_speed", sTravelSpeed);
                json = SetJsonVal(json, "initial_layer_travel_speed", sInitialLayerTravelSpeed);
                json = SetJsonVal(json, "reduce_infill_retraction", "0");
                json = SetJsonVal(json, "seam_position", "back");
                json = SetJsonVal(json, "wipe_on_loops", "1");
                json = SetJsonVal(json, "role_based_wipe_speed", "1");

                // Auto brim ayarları (Kullanıcı talebi: outer 2mm / 0.14 gap)
                json = SetJsonVal(json, "brim_type", "outer_only");
                json = SetJsonVal(json, "brim_width", "2");
                json = SetJsonVal(json, "brim_object_gap", "0.14");
                json = SetJsonVal(json, "enable_auto_brim", "0");
                json = SetJsonVal(json, "elefant_foot_compensation", "0");
                // OrcaSlicer / Bambu Studio override listesi: Proje ayarlarının dilimleyicide ZORUNLU olarak geçerli kılınması
                if (json.Contains("brim_object_gap;"))
                {
                    if (!json.Contains("brim_type;"))
                        json = json.Replace("brim_object_gap;", "brim_object_gap;brim_type;");
                }

                // Prime tower (Resmi Snapmaker U1 çoklu kafa: kaburgalı 30mm kule, 5mm brim, %120 aralık, 27mm3 prime)
                json = ApplyU1PrimeTowerProcessSettings(json, false);
                json = ApplyU1PrimeTowerMachineSettings(json);
                json = ApplyU1PrimeTowerFilamentSettings(json, 2, isPla);
                json = SetJsonArray(json, "wipe_tower_x", "25", "25");
                json = SetJsonArray(json, "wipe_tower_y", "195", "195");

                // İpliklenme ve duvar aşma önleme
                json = SetJsonVal(json, "reduce_crossing_wall", "1");
                json = SetJsonVal(json, "max_travel_detour_distance", "30");

                // TAM VE KESİNLİKLE 2 RENK (Kullanıcı talebi: 3 renk hatası kesin giderildi!)
                json = SetJsonArray(json, "filament_colour", color1Hex, color2Hex);
                json = SetJsonArray(json, "filament_type", matType, matType);
                json = SetJsonArray(json, "filament_settings_id", filProfileName, filProfileName);
                json = SetJsonArray(json, "nozzle_temperature", nozzleTemp, nozzleTemp);
                json = SetJsonArray(json, "nozzle_temperature_initial_layer", nozzleInitialTemp, nozzleInitialTemp);
                json = SetJsonArray(json, "nozzle_temperature_range_high", rangeHigh, rangeHigh);
                json = SetJsonArray(json, "nozzle_temperature_range_low", rangeLow, rangeLow);
                json = SetJsonArray(json, "hot_plate_temp", bedTemp, bedTemp);
                json = SetJsonArray(json, "hot_plate_temp_initial_layer", bedInitialTemp, bedInitialTemp);
                json = SetJsonArray(json, "textured_plate_temp", bedTemp, bedTemp);
                json = SetJsonArray(json, "textured_plate_temp_initial_layer", bedInitialTemp, bedInitialTemp);
                json = SetJsonArray(json, "filament_max_volumetric_speed", maxVolSpeed, maxVolSpeed);
                json = SetJsonArray(json, "fan_min_speed", fanMin, fanMin);
                json = SetJsonArray(json, "fan_max_speed", fanMax, fanMax);
                json = SetJsonArray(json, "filament_flow_ratio", flowRatio, flowRatio);
                json = SetJsonArray(json, "filament_retraction_length", retLength, retLength);
                json = SetJsonArray(json, "filament_retraction_speed", retSpeed, retSpeed);
                json = SetJsonArray(json, "filament_deretraction_speed", deretSpeed, deretSpeed);
                json = SetJsonArray(json, "filament_retract_when_changing_layer", "1", "1");
                json = SetJsonArray(json, "filament_wipe", "1", "1");
                json = SetJsonArray(json, "filament_wipe_distance", wipeDist, wipeDist);
                json = SetJsonArray(json, "retraction_length", retLength, retLength);
                json = SetJsonArray(json, "retraction_speed", retSpeed, retSpeed);
                json = SetJsonArray(json, "deretraction_speed", deretSpeed, deretSpeed);
                json = SetJsonArray(json, "retract_before_wipe", "70%", "70%");
                json = SetJsonArray(json, "wipe", "1", "1");
                json = SetJsonArray(json, "wipe_distance", wipeDist, wipeDist);
                json = SetJsonArray(json, "z_hop", "0", "0");

                // Renk Geçiş Hacimleri (Kırmızı -> Sarı bulaşmasını %100 önler)
                json = SetJsonArray(json, "flush_volumes_matrix", "0", "180", "320", "0");
                json = SetJsonArray(json, "flush_volumes_vector", "180", "320");
                json = SetJsonArray(json, "wiping_volumes_extruders", "180", "320");
                json = SetJsonVal(json, "flush_multiplier", "1");

                File.WriteAllText(path, json, Encoding.UTF8);
            };

            Action<string> updateFilament = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonArray(json, "filament_settings_id", filProfileName);
                json = SetJsonVal(json, "name", filProfileName);
                json = SetJsonVal(json, "from", "project");
                json = SetJsonVal(json, "inherits", inheritsFil);
                json = SetJsonArray(json, "filament_type", matType);
                json = SetJsonArray(json, "filament_vendor", "Generic");
                json = SetJsonArray(json, "nozzle_temperature", nozzleTemp);
                json = SetJsonArray(json, "nozzle_temperature_initial_layer", nozzleInitialTemp);
                json = SetJsonArray(json, "nozzle_temperature_range_high", rangeHigh);
                json = SetJsonArray(json, "nozzle_temperature_range_low", rangeLow);
                json = SetJsonArray(json, "hot_plate_temp", bedTemp);
                json = SetJsonArray(json, "hot_plate_temp_initial_layer", bedInitialTemp);
                json = SetJsonArray(json, "textured_plate_temp", bedTemp);
                json = SetJsonArray(json, "textured_plate_temp_initial_layer", bedInitialTemp);
                json = SetJsonArray(json, "filament_max_volumetric_speed", maxVolSpeed);
                json = SetJsonArray(json, "fan_min_speed", fanMin);
                json = SetJsonArray(json, "fan_max_speed", fanMax);
                json = SetJsonArray(json, "filament_flow_ratio", flowRatio);
                json = SetJsonArray(json, "filament_retraction_length", retLength);
                json = SetJsonArray(json, "filament_retraction_speed", retSpeed);
                json = SetJsonArray(json, "filament_deretraction_speed", deretSpeed);
                json = SetJsonArray(json, "filament_retract_when_changing_layer", "1");
                json = SetJsonArray(json, "filament_wipe", "1");
                json = SetJsonArray(json, "filament_wipe_distance", wipeDist);
                json = SetJsonArray(json, "filament_retract_before_wipe", "70%");
                json = SetJsonArray(json, "filament_z_hop", "0");
                json = ApplyU1PrimeTowerFilamentSettings(json, 1, isPla);
                File.WriteAllText(path, json, Encoding.UTF8);
            };

            Action<string> updateProcess = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonVal(json, "name", procName);
                json = SetJsonVal(json, "print_settings_id", procName);
                json = SetJsonArray(json, "compatible_printers", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonVal(json, "from", "project");
                json = SetJsonVal(json, "inherits", "fdm_process_U1_0.12");
                json = SetJsonVal(json, "layer_height", layerHeightStr);
                json = SetJsonVal(json, "initial_layer_print_height", "0.20");
                json = SetJsonVal(json, "wall_generator", "classic");
                json = SetJsonVal(json, "detect_thin_wall", "0");
                json = SetJsonVal(json, "enable_arc_fitting", "0");
                json = SetJsonVal(json, "line_width", "0.42");
                json = SetJsonVal(json, "outer_wall_line_width", "0.42");
                json = SetJsonVal(json, "inner_wall_line_width", "0.45");
                json = SetJsonVal(json, "initial_layer_line_width", "0.50");
                json = SetJsonVal(json, "wall_loops", "2");
                json = SetJsonVal(json, "sparse_infill_density", "100%");
                json = SetJsonVal(json, "sparse_infill_pattern", "rectilinear");
                json = SetJsonVal(json, "outer_wall_speed", sOuterWallSpeed);
                json = SetJsonVal(json, "inner_wall_speed", sInnerWallSpeed);
                json = SetJsonVal(json, "small_perimeter_speed", sSmallPerimeterSpeed);
                json = SetJsonVal(json, "sparse_infill_speed", sSparseInfillSpeed);
                json = SetJsonVal(json, "top_surface_speed", sTopSurfaceSpeed);
                json = SetJsonVal(json, "initial_layer_speed", sInitialLayerSpeed);
                json = SetJsonVal(json, "initial_layer_infill_speed", sInitialLayerInfillSpeed);
                json = SetJsonVal(json, "travel_speed", sTravelSpeed);
                json = SetJsonVal(json, "initial_layer_travel_speed", sInitialLayerTravelSpeed);
                json = SetJsonVal(json, "seam_position", "back");

                // Auto brim (Kullanıcı talebi: outer 2mm / 0.14 gap)
                json = SetJsonVal(json, "brim_type", "outer_only");
                json = SetJsonVal(json, "brim_width", "2");
                json = SetJsonVal(json, "brim_object_gap", "0.14");
                json = SetJsonVal(json, "enable_auto_brim", "0");
                json = SetJsonVal(json, "elefant_foot_compensation", "0");

                // Prime tower (Resmi Snapmaker U1 çoklu kafa: kaburgalı 30mm kule, 5mm brim, %120 aralık, 27mm3 prime)
                json = ApplyU1PrimeTowerProcessSettings(json, true);

                // İpliklenme ve duvar aşma önleme
                json = SetJsonVal(json, "reduce_crossing_wall", "1");
                json = SetJsonVal(json, "max_travel_detour_distance", "30");
                json = SetJsonVal(json, "wipe_on_loops", "1");
                json = SetJsonVal(json, "role_based_wipe_speed", "1");
                File.WriteAllText(path, json, Encoding.UTF8);
            };

            Action<string> updateMachine = path =>
            {
                if (!File.Exists(path)) return;
                string json = File.ReadAllText(path, Encoding.UTF8);
                json = SetJsonVal(json, "name", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonVal(json, "printer_model", "Snapmaker U1");
                json = SetJsonVal(json, "printer_settings_id", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonVal(json, "from", "project");
                json = SetJsonVal(json, "default_print_profile", procName);
                json = SetJsonArray(json, "compatible_printers", "Snapmaker U1 (0.4 nozzle)");
                json = SetJsonArray(json, "default_filament_profile", filProfileName, filProfileName);
                json = SetJsonArray(json, "retraction_length", retLength, retLength);
                json = SetJsonArray(json, "retraction_speed", retSpeed, retSpeed);
                json = SetJsonArray(json, "deretraction_speed", deretSpeed, deretSpeed);
                json = SetJsonArray(json, "retract_before_wipe", "70%", "70%");
                json = SetJsonArray(json, "wipe", "1", "1");
                json = SetJsonArray(json, "wipe_distance", wipeDist, wipeDist);
                json = SetJsonArray(json, "z_hop", "0", "0");
                json = SetJsonArray(json, "retract_when_changing_layer", "1", "1");
                json = ApplyU1PrimeTowerMachineSettings(json);
                File.WriteAllText(path, json, Encoding.UTF8);
            };

            updateProcess(Path.Combine(tempDir, "Metadata", "process_settings_1.config"));
            updateConfig(Path.Combine(tempDir, "Metadata", "project_settings.config"));
            updateFilament(Path.Combine(tempDir, "Metadata", "filament_settings_1.config"));
            updateFilament(Path.Combine(tempDir, "Metadata", "filament_settings_2.config"));
            updateMachine(Path.Combine(tempDir, "Metadata", "machine_settings_1.config"));

            // 3. ve sonraki gereksiz filament dosyalarını sil (Sadece 2 renk kalmasını sağlar)
            for (int fi = 3; fi <= 16; fi++)
            {
                string extraFil = Path.Combine(tempDir, "Metadata", string.Format("filament_settings_{0}.config", fi));
                if (File.Exists(extraFil)) File.Delete(extraFil);
            }

            // Save embedded JSON metadata in Metadata/pigeon_ring_config.json
            string configJsonPath = Path.Combine(tempDir, "Metadata", "pigeon_ring_config.json");
            File.WriteAllText(configJsonPath, currentConfig.ToJson(), Encoding.UTF8);

            // Clean relations and ensure root _rels/.rels
            string rootRelsPath = Path.Combine(tempDir, "_rels", ".rels");
            Directory.CreateDirectory(Path.GetDirectoryName(rootRelsPath));
            using (StreamWriter sw = new StreamWriter(rootRelsPath, false, Encoding.UTF8))
            {
                sw.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
                sw.WriteLine("<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">");
                sw.WriteLine(" <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>");
                sw.WriteLine("</Relationships>");
            }

            // 3MF içine özel fontları göm
            try
            {
                string metaFontsDir = Path.Combine(tempDir, "Metadata", "Fonts");
                string topFontFile = FontManager.GetFontFilePath(fontTop);
                if (!string.IsNullOrEmpty(topFontFile) && File.Exists(topFontFile))
                {
                    Directory.CreateDirectory(metaFontsDir);
                    string dest = Path.Combine(metaFontsDir, Path.GetFileName(topFontFile));
                    File.Copy(topFontFile, dest, true);
                    fullLog(string.Format("  -> Özel font 3MF içine gömüldü: {0}", Path.GetFileName(topFontFile)));
                }
                string botFontFile = FontManager.GetFontFilePath(fontBot);
                if (!string.IsNullOrEmpty(botFontFile) && File.Exists(botFontFile))
                {
                    Directory.CreateDirectory(metaFontsDir);
                    string dest = Path.Combine(metaFontsDir, Path.GetFileName(botFontFile));
                    if (!File.Exists(dest))
                    {
                        File.Copy(botFontFile, dest, true);
                        fullLog(string.Format("  -> Özel font 3MF içine gömüldü: {0}", Path.GetFileName(botFontFile)));
                    }
                }
            }
            catch { }

            // Pack into final 3MF archive
            CleanResidualMetadata(tempDir);

            if (File.Exists(output3mf)) File.Delete(output3mf);
            CreateZipArchive(tempDir, output3mf);

            // Save companion _bilgi.txt next to the .3mf file
            try
            {
                string infoPath = Path.Combine(Path.GetDirectoryName(output3mf), Path.GetFileNameWithoutExtension(output3mf) + "_bilgi.txt");
                File.WriteAllText(infoPath, currentConfig.ToReportText(Path.GetFileName(output3mf)), Encoding.UTF8);
                fullLog(string.Format("Ayar ve Bilgi Raporu Kaydedildi: {0}", Path.GetFileName(infoPath)));
            }
            catch (Exception exInfo)
            {
                fullLog("Bilgi raporu kaydedilemedi: " + exInfo.Message);
            }

            // Cleanup temp dir
            try { Directory.Delete(tempDir, true); } catch { }

            fullLog(string.Format("BAŞARILI: '{0}' çift tablalı projesi kusursuz şekilde üretildi!\n", output3mf));
            return true;
        }
        catch (Exception ex)
        {
            fullLog(string.Format("KRİTİK HATA: {0}\n{1}", ex.Message, ex.StackTrace));
            return false;
        }
    }

    public static bool ExecuteGeneration(string topText, string botText, string backup3mf, string output3mf, 
                                         double embossMm, double embedMm, 
                                         string fontTop, string fontBot, 
                                         double textHeightMm, double charGapMm, double widthScale,
                                         Action<string> log, string material = "PLA",
                                         string color1Hex = "#FFFF00", string color2Hex = "#FF0000")
    {
        return ExecuteGeneration(topText, botText, backup3mf, output3mf, embossMm, embedMm, fontTop, fontBot, textHeightMm, textHeightMm, charGapMm, charGapMm, widthScale, widthScale, log, material, color1Hex, color2Hex);
    }

    public static bool ExecuteGeneration(string topText, string botText, string backup3mf, string output3mf, 
                                         double embossMm, double embedMm, 
                                         string fontName, double textHeightMm, double charGapMm, double widthScale,
                                         Action<string> log, string material = "PLA",
                                         string color1Hex = "#FFFF00", string color2Hex = "#FF0000")
    {
        return ExecuteGeneration(topText, botText, backup3mf, output3mf, embossMm, embedMm, fontName, fontName, textHeightMm, textHeightMm, charGapMm, charGapMm, widthScale, widthScale, log, material, color1Hex, color2Hex);
    }

    private static string SetJsonVal(string json, string key, string val)
    {
        string pattern = string.Format("\"{0}\"\\s*:\\s*(\"[^\"]*\"|\\[[^\\]]*\\]|[0-9\\.]+|true|false)", key);
        var match = System.Text.RegularExpressions.Regex.Match(json, pattern);
        if (match.Success)
        {
            string matchedVal = match.Groups[1].Value.Trim();
            string replacement;
            if (matchedVal.StartsWith("["))
            {
                var count = System.Text.RegularExpressions.Regex.Matches(matchedVal, "\"[^\"]*\"|[-0-9\\.]+").Count;
                if (count <= 1)
                {
                    replacement = string.Format("\"{0}\": [\n        \"{1}\"\n    ]", key, val);
                }
                else
                {
                    StringBuilder sb = new StringBuilder();
                    sb.Append("[\n");
                    for (int i = 0; i < count; i++)
                    {
                        sb.Append("        \"").Append(val).Append("\"");
                        if (i < count - 1) sb.Append(",");
                        sb.Append("\n");
                    }
                    sb.Append("    ]");
                    replacement = string.Format("\"{0}\": {1}", key, sb.ToString());
                }
            }
            else
            {
                replacement = string.Format("\"{0}\": \"{1}\"", key, val);
            }
            return json.Substring(0, match.Index) + replacement + json.Substring(match.Index + match.Length);
        }
        else
        {
            int idx = json.LastIndexOf('}');
            if (idx > 0)
            {
                return json.Substring(0, idx) + ",\n  \"" + key + "\": \"" + val + "\"\n}";
            }
            return json;
        }
    }

    private static string SetJsonArray(string json, string key, params string[] values)
    {
        StringBuilder sb = new StringBuilder();
        sb.Append("[\n");
        for (int i = 0; i < values.Length; i++)
        {
            sb.Append("        \"").Append(values[i]).Append("\"");
            if (i < values.Length - 1) sb.Append(",");
            sb.Append("\n");
        }
        sb.Append("    ]");
        string pattern = string.Format("\"{0}\"\\s*:\\s*\\[[^\\]]*\\]", key);
        string replacement = string.Format("\"{0}\": {1}", key, sb.ToString());
        if (System.Text.RegularExpressions.Regex.IsMatch(json, pattern))
        {
            return System.Text.RegularExpressions.Regex.Replace(json, pattern, replacement);
        }
        else
        {
            int idx = json.LastIndexOf('}');
            if (idx > 0)
            {
                return json.Substring(0, idx) + ",\n  \"" + key + "\": " + sb.ToString() + "\n}";
            }
            return json;
        }
    }

    private static string RemoveJsonKey(string json, string key)
    {
        // Removes "key": <value> where value is string, array, object, number, bool, or null.
        string pattern = string.Format("\\s*\"{0}\"\\s*:\\s*(\"(?:[^\"\\\\]|\\\\.)*\"|\\[[^\\]]*\\]|\\{{[^\\}}]*\\}}|-?[0-9.]+|true|false|null)\\s*,?", System.Text.RegularExpressions.Regex.Escape(key));
        string result = System.Text.RegularExpressions.Regex.Replace(json, pattern, string.Empty, System.Text.RegularExpressions.RegexOptions.Singleline);
        // Clean up possible trailing comma before closing brace
        result = System.Text.RegularExpressions.Regex.Replace(result, ",(\\s*\\})", "$1");
        return result;
    }

    private static string[] RepeatStr(string val, int count)
    {
        string[] arr = new string[count];
        for (int i = 0; i < count; i++) arr[i] = val;
        return arr;
    }

    private static string ApplyU1PrimeTowerProcessSettings(string json, bool isProcessFileOnly)
    {
        if (isProcessFileOnly)
        {
            json = RemoveJsonKey(json, "purge_in_prime_tower");
            json = RemoveJsonKey(json, "timelapse_type");
        }

        json = SetJsonVal(json, "enable_prime_tower", "1");
        json = SetJsonVal(json, "prime_tower_width", "30");
        json = SetJsonVal(json, "prime_tower_brim_width", "5");
        json = SetJsonVal(json, "prime_tower_brim_chamfer", "1");
        json = SetJsonVal(json, "prime_tower_brim_chamfer_max_width", "4");
        json = SetJsonVal(json, "prime_tower_enable_framework", "0");
        json = SetJsonVal(json, "prime_volume", "27");
        json = SetJsonVal(json, "wipe_tower_wall_type", "rib");
        json = SetJsonVal(json, "wipe_tower_extra_rib_length", "8");
        json = SetJsonVal(json, "wipe_tower_rib_width", "8");
        json = SetJsonVal(json, "wipe_tower_cone_angle", "15");
        json = SetJsonVal(json, "wipe_tower_extra_spacing", "120%");
        json = SetJsonVal(json, "wipe_tower_extra_flow", "100%");
        json = SetJsonVal(json, "wipe_tower_fillet_wall", "1");
        json = SetJsonVal(json, "wipe_tower_bridging", "10");
        json = SetJsonVal(json, "wipe_tower_max_purge_speed", "90");
        json = SetJsonVal(json, "wipe_tower_no_sparse_layers", "0");
        json = SetJsonVal(json, "wipe_tower_rotation_angle", "0");
        json = SetJsonVal(json, "wipe_tower_wall_gap", "1");
        json = SetJsonVal(json, "wipe_tower_filament", "0");
        json = SetJsonVal(json, "local_z_wipe_tower_purge_lines", "3");
        json = SetJsonVal(json, "ooze_prevention", "1");
        json = SetJsonVal(json, "flush_into_infill", "0");
        json = SetJsonVal(json, "flush_into_objects", "0");
        json = SetJsonVal(json, "flush_into_support", "0");
        if (!isProcessFileOnly)
            json = SetJsonVal(json, "timelapse_type", "0");
        return json;
    }

    private static string ApplyU1PrimeTowerMachineSettings(string json)
    {
        json = SetJsonVal(json, "purge_in_prime_tower", "0");
        json = SetJsonVal(json, "enable_filament_ramming", "0");
        json = SetJsonVal(json, "ramming_line_width_ratio", "2");
        json = SetJsonVal(json, "ramming_pressure_advance_value", "0.02");
        json = SetJsonArray(json, "retract_length_toolchange", "10", "10", "10", "10");
        json = SetJsonArray(json, "retract_restart_extra_toolchange", "0", "0", "0", "0");
        json = SetJsonArray(json, "z_hop_when_prime", "0", "0", "0", "0");
        return json;
    }

    private static string ApplyU1PrimeTowerFilamentSettings(string json, int count, bool isPla)
    {
        string ramFlow = isPla ? "20" : "15";
        json = SetJsonArray(json, "filament_multitool_ramming", RepeatStr("1", count));
        json = SetJsonArray(json, "filament_multitool_ramming_flow", RepeatStr(ramFlow, count));
        json = SetJsonArray(json, "filament_multitool_ramming_volume", RepeatStr("0.5", count));
        json = SetJsonArray(json, "filament_retract_length_toolchange", RepeatStr("10", count));
        json = SetJsonArray(json, "filament_minimal_purge_on_wipe_tower", RepeatStr("15", count));
        json = SetJsonArray(json, "filament_prime_volume", RepeatStr("45", count));
        json = SetJsonArray(json, "filament_toolchange_delay", RepeatStr("0", count));
        return json;
    }

    private static void CleanResidualMetadata(string tempDir)
    {
        try
        {
            // Remove known Bambu/Orca residual keys from config files
            string[] configFiles = Directory.GetFiles(Path.Combine(tempDir, "Metadata"), "*.config", SearchOption.TopDirectoryOnly);
            string[] residualKeys = new string[] {
                "bbl_calib_mark_logo",
                "bbl_use_printhost",
                "bbl_bed_temperature_gcode"
            };
            foreach (string cfg in configFiles)
            {
                try
                {
                    string json = File.ReadAllText(cfg, Encoding.UTF8);
                    string cleaned = json;
                    foreach (string k in residualKeys) cleaned = RemoveJsonKey(cleaned, k);
                    if (!ReferenceEquals(cleaned, json) && cleaned != json)
                        File.WriteAllText(cfg, cleaned, Encoding.UTF8);
                }
                catch { }
            }

            // Remove residual auxiliary pictures inherited from old Bambu templates
            string[] auxDirs = new string[] {
                Path.Combine(tempDir, "Auxiliaries", "Model Pictures"),
                Path.Combine(tempDir, "Auxiliaries", "Profile Pictures")
            };
            foreach (string d in auxDirs)
            {
                try { if (Directory.Exists(d)) Directory.Delete(d, true); } catch { }
            }
        }
        catch { }
    }

    private static void CreateZipArchive(string sourceDir, string zipPath)
    {
        using (FileStream zipStream = new FileStream(zipPath, FileMode.Create))
        {
            using (ZipArchive archive = new ZipArchive(zipStream, ZipArchiveMode.Create))
            {
                string[] files = Directory.GetFiles(sourceDir, "*", SearchOption.AllDirectories);
                int baseLen = sourceDir.TrimEnd('\\', '/').Length + 1;
                foreach (string f in files)
                {
                    string entryName = f.Substring(baseLen).Replace('\\', '/');
                    archive.CreateEntryFromFile(f, entryName, CompressionLevel.Optimal);
                }
            }
        }
    }

    // 7. WINDOWS FORMS GUI INTERFACE
    // ==========================================
    // 7. HELPER: SAFE FILE NAME, TURKISH LETTERS & PHONE FORMAT
    // ==========================================
    public static string MakeSafeFileName(string name)
    {
        if (string.IsNullOrEmpty(name)) return "Halka";
        char[] invalid = Path.GetInvalidFileNameChars();
        StringBuilder sb = new StringBuilder();
        foreach (char c in name.Trim())
        {
            if (Array.IndexOf(invalid, c) < 0)
                sb.Append(c);
            else
                sb.Append('_');
        }
        string result = sb.ToString().Trim();
        return string.IsNullOrEmpty(result) ? "Halka" : result;
    }

    public static string ConvertToTurkishUpper(string text, bool useTurkishFix = true)
    {
        return ProcessTextWithSoftG(text, useTurkishFix);
    }

    public static string ConvertSpecialTurkishUpperLetters(string text, bool useTurkishFix = true)
    {
        return ProcessTextWithSoftG(text, useTurkishFix);
    }

    public static string FormatPhoneNumber(string input)
    {
        if (string.IsNullOrEmpty(input)) return input;

        string digits = "";
        foreach (char c in input)
        {
            if (char.IsDigit(c)) digits += c;
        }

        if (digits.Length == 0) return input;

        // +90 ile başlamışsa (905064228149 -> 12 hane), 90'ı 0 yap
        if (digits.Length >= 12 && digits.StartsWith("90"))
        {
            digits = "0" + digits.Substring(2);
        }
        // 10 hane ise ve 0 ile başlamıyorsa (5064228149), başına 0 ekle -> 05064228149
        else if (digits.Length == 10 && digits[0] != '0')
        {
            digits = "0" + digits;
        }

        // 11 haneli standart Türkiye cep telefonu: 0000 000 00 00
        if (digits.Length == 11)
        {
            return string.Format("{0} {1} {2} {3}",
                digits.Substring(0, 4),
                digits.Substring(4, 3),
                digits.Substring(7, 2),
                digits.Substring(9, 2));
        }

        // 15 hane ise: 11 hane telefon + 4 hane yıl (örn: 0506 938 19 49 - 2026)
        if (digits.Length == 15)
        {
            return string.Format("{0} {1} {2} {3} - {4}",
                digits.Substring(0, 4),
                digits.Substring(4, 3),
                digits.Substring(7, 2),
                digits.Substring(9, 2),
                digits.Substring(11, 4));
        }

        return input;
    }

    // ==========================================
    public static string GetAppDataDir()
    {
        string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "PigeonRingGenerator");
        if (!Directory.Exists(dir))
        {
            try { Directory.CreateDirectory(dir); } catch { }
        }
        return dir;
    }

    public static string GetTemplatesDir()
    {
        string dir = Path.Combine(GetAppDataDir(), "Templates");
        if (!Directory.Exists(dir))
        {
            try { Directory.CreateDirectory(dir); } catch { }
        }
        return dir;
    }

    public static void EnsureEmbeddedTemplatesExtracted()
    {
        try
        {
            var asm = System.Reflection.Assembly.GetExecutingAssembly();
            string tdir = GetTemplatesDir();
            foreach (string resName in asm.GetManifestResourceNames())
            {
                if (resName.EndsWith(".3mf.bak", StringComparison.OrdinalIgnoreCase))
                {
                    string outPath = Path.Combine(tdir, Path.GetFileName(resName));
                    using (var s = asm.GetManifestResourceStream(resName))
                    {
                        if (s == null) continue;
                        if (!File.Exists(outPath) || new FileInfo(outPath).Length != s.Length)
                        {
                            using (var fs = new FileStream(outPath, FileMode.Create, FileAccess.Write))
                            {
                                s.CopyTo(fs);
                            }
                        }
                    }
                }
            }
        }
        catch { }
    }

    // 7.5 FONT MANAGER (ÖZEL & TAŞINABİLİR FONT YÖNETİCİSİ)
    // ==========================================
    public static class FontManager
    {
        private static PrivateFontCollection pfc = new PrivateFontCollection();
        private static Dictionary<string, FontFamily> customFamilies = new Dictionary<string, FontFamily>(StringComparer.OrdinalIgnoreCase);
        private static Dictionary<string, string> customFontFilePaths = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        private static HashSet<string> loadedFiles = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        private static bool isInitialized = false;

        public static string FontsDirectory
        {
            get
            {
                string dir = Path.Combine(GetAppDataDir(), "Fonts");
                if (!Directory.Exists(dir))
                {
                    try { Directory.CreateDirectory(dir); } catch { }
                }
                return dir;
            }
        }

        public static void EnsureInitialized()
        {
            if (isInitialized) return;
            isInitialized = true;
            ScanAndLoadFonts();
        }

        public static void ScanAndLoadFonts()
        {
            // 1. Gömülü assembly kaynaklarını tara (eğer exe içine gömülmüşse)
            try
            {
                var asm = System.Reflection.Assembly.GetExecutingAssembly();
                foreach (string resName in asm.GetManifestResourceNames())
                {
                    if (resName.EndsWith(".ttf", StringComparison.OrdinalIgnoreCase) || 
                        resName.EndsWith(".otf", StringComparison.OrdinalIgnoreCase))
                    {
                        string targetPath = Path.Combine(FontsDirectory, Path.GetFileName(resName));
                        using (var s = asm.GetManifestResourceStream(resName))
                        {
                            if (s != null && (!File.Exists(targetPath) || new FileInfo(targetPath).Length != s.Length))
                            {
                                using (var fs = new FileStream(targetPath, FileMode.Create, FileAccess.Write))
                                {
                                    s.CopyTo(fs);
                                }
                            }
                        }
                    }
                }
            }
            catch { }

            // 2. Yerel uygulama Fonts klasöründeki (ve varsa .exe yanındaki Fonts) fontları yükle
            string extDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Fonts");
            if (Directory.Exists(extDir))
            {
                foreach (string file in Directory.GetFiles(extDir, "*.*"))
                {
                    if (file.EndsWith(".ttf", StringComparison.OrdinalIgnoreCase) ||
                        file.EndsWith(".otf", StringComparison.OrdinalIgnoreCase))
                        LoadFontFile(file);
                }
            }
            string dir = FontsDirectory;
            if (Directory.Exists(dir))
            {
                string[] files = Directory.GetFiles(dir, "*.*");
                foreach (string file in files)
                {
                    if (file.EndsWith(".ttf", StringComparison.OrdinalIgnoreCase) || 
                        file.EndsWith(".otf", StringComparison.OrdinalIgnoreCase))
                    {
                        LoadFontFile(file);
                    }
                }
            }
        }

        public static List<FontFamily> LoadFontFile(string filePath)
        {
            List<FontFamily> newFamilies = new List<FontFamily>();
            if (!File.Exists(filePath)) return newFamilies;

            try
            {
                string norm = Path.GetFullPath(filePath);
                if (loadedFiles.Contains(norm))
                {
                    foreach (var kvp in customFontFilePaths)
                    {
                        if (string.Equals(kvp.Value, norm, StringComparison.OrdinalIgnoreCase))
                        {
                            if (customFamilies.ContainsKey(kvp.Key))
                                newFamilies.Add(customFamilies[kvp.Key]);
                        }
                    }
                    return newFamilies;
                }

                // GDI+ FontFamily listesini alfabetik tutar; yeni eklenenleri önce/sonra farkından buluruz
                HashSet<string> beforeNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                foreach (FontFamily ff in pfc.Families)
                {
                    beforeNames.Add(ff.Name);
                }

                pfc.AddFontFile(norm);
                loadedFiles.Add(norm);

                foreach (FontFamily ff in pfc.Families)
                {
                    if (!beforeNames.Contains(ff.Name))
                    {
                        customFamilies[ff.Name] = ff;
                        customFontFilePaths[ff.Name] = norm;
                        newFamilies.Add(ff);
                    }
                }

                // Eğer font mevcut bir ailenin ek stili ise (family sayısı artmadıysa) ilişkilendir
                if (newFamilies.Count == 0)
                {
                    foreach (FontFamily ff in pfc.Families)
                    {
                        if (!customFontFilePaths.ContainsKey(ff.Name))
                        {
                            customFamilies[ff.Name] = ff;
                            customFontFilePaths[ff.Name] = norm;
                            newFamilies.Add(ff);
                        }
                    }
                }
            }
            catch { }

            return newFamilies;
        }

        public static List<string> GetCustomFontNames()
        {
            EnsureInitialized();
            return new List<string>(customFamilies.Keys);
        }

        public static string GetFontFilePath(string fontName)
        {
            EnsureInitialized();
            string clean = CleanFontName(fontName);
            if (customFontFilePaths.ContainsKey(clean)) return customFontFilePaths[clean];
            return null;
        }

        public static string CleanFontName(string fontName)
        {
            if (string.IsNullOrEmpty(fontName)) return "Trebuchet MS";
            return fontName.Replace(" [Özel]", "").Replace("[Özel] ", "").Replace(" (Özel)", "").Trim();
        }

        public static FontFamily GetFontFamily(string fontName)
        {
            EnsureInitialized();
            string clean = CleanFontName(fontName);

            if (customFamilies.ContainsKey(clean))
            {
                return customFamilies[clean];
            }

            try
            {
                return new FontFamily(clean);
            }
            catch
            {
                try
                {
                    return new FontFamily("Trebuchet MS");
                }
                catch
                {
                    return FontFamily.GenericSansSerif;
                }
            }
        }

        public static Font CreateFont(string fontName, float size, FontStyle preferredStyle = FontStyle.Bold)
        {
            FontFamily ff = GetFontFamily(fontName);
            FontStyle style = ff.IsStyleAvailable(preferredStyle) ? preferredStyle : FontStyle.Regular;
            return new Font(ff, size, style);
        }

        public static Font CreateFont(string fontName, float size, FontStyle preferredStyle, GraphicsUnit unit)
        {
            FontFamily ff = GetFontFamily(fontName);
            FontStyle style = ff.IsStyleAvailable(preferredStyle) ? preferredStyle : FontStyle.Regular;
            return new Font(ff, size, style, unit);
        }
    }

    // ==========================================
    // 8. WINDOWS FORMS GUI INTERFACE

    public static string GetDefaultBakPath(string bakFileName)
    {
        EnsureEmbeddedTemplatesExtracted();

        string baseDir = AppDomain.CurrentDomain.BaseDirectory;
        string p = Path.Combine(baseDir, bakFileName);
        if (File.Exists(p)) return p;

        string emb = Path.Combine(GetTemplatesDir(), bakFileName);
        if (File.Exists(emb)) return emb;

        string gPath = Path.Combine(@"G:\Drive'ım\3D MODEL\Güvercin\OTO\İSİM MARKA KLİPS", bakFileName);
        if (File.Exists(gPath)) return gPath;

        string dPath = Path.Combine(@"D:\OTO-KLIPS", bakFileName);
        if (File.Exists(dPath)) return dPath;

        return emb;
    }
    // ==========================================
    // 8. WINDOWS FORMS GUI INTERFACE (MAINFORM)
    // ==========================================
    public class MainForm : Form
    {
        private GeneratorMode currentMode = GeneratorMode.IsimliKlips;

        // Mod Seçim Butonları
        private Button btnModeIsimli;
        private Button btnModeSirali;
        private Button btnModeMarka;
        private Label lblModeDesc;

        // İsimli & Marka Klips Kontrolleri
        private GroupBox grpTop;
        private GroupBox grpBot;
        private TextBox txtTopText;
        private CheckBox chkSoftG;
        private TextBox txtBotText;
        private ComboBox cmbFontTop;
        private ComboBox cmbFontBot;
        private TextBox txtFontFilter;
        private NumericUpDown numTextHeightTop;
        private NumericUpDown numTextHeightBot;
        private NumericUpDown numStretchTop;
        private NumericUpDown numStretchBot;
        private NumericUpDown numCharGapTop;
        private NumericUpDown numCharGapBot;
        private NumericUpDown numWidthScaleTop;
        private NumericUpDown numWidthScaleBot;
        private Button btnSyncFonts;
        private Button btnLoadCustomFont;
        private Button btnOpenFontsFolder;

        // Sıralı Klips Kontrolleri
        private GroupBox grpSeqNums;
        private GroupBox grpSeqFont;
        private Button btnPreset10;
        private Button btnPreset20;
        private Button btnPreset30;
        private Button btnPreset40;
        private Button btnPreset50;
        private NumericUpDown numSeqStart;
        private NumericUpDown numSeqEnd;
        private CheckBox chkIncludeGender;
        private Label lblSeqTotalCount;
        private ComboBox cmbSeqFont;
        private TextBox txtSeqFontFilter;
        private Button btnSeqLoadCustomFont;
        private NumericUpDown numSeqHeight;
        private NumericUpDown numSeqStretch;
        private NumericUpDown numSeqGap;
        private NumericUpDown numSeqWidth;
        private ComboBox cmbSeqLayerHeight;

        // Ortak Geometri, Malzeme & Renk Kontrolleri
        private GroupBox grpParams;
        private NumericUpDown numEmboss;
        private NumericUpDown numEmbed;
        private ComboBox cmbMaterial;
        private TextBox txtBakPath;
        private Button btnBrowseBak;

        // Önizleme & İşlem Butonları
        private DoubleBufferedPanel pnlFontPreview;
        private GroupBox grpPreview;
        private Label lblScaleInfo;
        private ComboBox cmbColorPresets;
        private Button btnColor1;
        private Button btnColor2;
        private Label lblColorHint;
        private Button btnColor3;
        private Button btnSwapColors;
        private Color currentColor1 = Color.FromArgb(234, 179, 8); // Sarı (Extruder 1 - Alt Gövde)
        private Color currentColor2 = Color.FromArgb(220, 38, 38); // Kırmızı (Extruder 2 - Üst Gövde)
        private Color currentColor3 = Color.FromArgb(255, 255, 255); // Beyaz (Extruder 3 - Sıralı Yazı Rengi)

        private Button btnGenerate;
        private RadioButton rbSpeedFast;
        private RadioButton rbSpeedSlow;
        private Button btnOpenOrca;
        private Button btnOpenFolder;
        private Button btnLoad3mf;
        private Button btnViewInfo;
        private TextBox txtLog;
        private string lastGeneratedPath = "";

        private List<string> allSystemFonts = new List<string>();
        private bool isUpdatingTopText = false;
        private bool isUpdatingBotText = false;

        public class DoubleBufferedPanel : Panel
        {
            public DoubleBufferedPanel()
            {
                this.DoubleBuffered = true;
                this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);
                this.UpdateStyles();
            }
        }

        public MainForm()
        {
            InitializeComponent();
            try
            {
                string icoPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "app.ico");
                if (File.Exists(icoPath))
                {
                    this.Icon = new Icon(icoPath);
                }
                else
                {
                    Icon exeIcon = Icon.ExtractAssociatedIcon(Application.ExecutablePath);
                    if (exeIcon != null) this.Icon = exeIcon;
                }
            }
            catch { }
        }



        private void InitializeComponent()
        {
            this.Text = "Snapmaker U1 Güvercin Halkası 3MF Oluşturucu [PER-GLYPH]";
            this.ClientSize = new Size(960, 690);
            this.MinimumSize = new Size(960, 690);
            this.StartPosition = FormStartPosition.CenterScreen;
            this.Font = new Font("Segoe UI", 9f, FontStyle.Regular);
            this.BackColor = Color.FromArgb(248, 250, 252);
            this.ForeColor = Color.FromArgb(15, 23, 42);

            // ==========================================
            // 1. HEADER BAR (Kompakt koyu lacivert bar)
            // ==========================================
            Panel pnlHeader = new Panel();
            pnlHeader.Dock = DockStyle.Top;
            pnlHeader.Height = 38;
            pnlHeader.BackColor = Color.FromArgb(15, 23, 42);
            pnlHeader.Padding = new Padding(14, 0, 14, 0);

            Label lblTitle = new Label();
            lblTitle.Text = "GÜVERCİN HALKASI 3MF MODEL ÜRETİCİ";
            lblTitle.Font = new Font("Segoe UI", 10.5f, FontStyle.Bold);
            lblTitle.ForeColor = Color.White;
            lblTitle.AutoSize = true;
            lblTitle.Location = new Point(14, 9);
            pnlHeader.Controls.Add(lblTitle);

            Label lblSubTitle = new Label();
            lblSubTitle.Text = "Snapmaker U1 / OrcaSlicer • Manifold 3D Katı Model • Z = 0.00 mm Tabla Hizalı";
            lblSubTitle.Font = new Font("Segoe UI", 8f, FontStyle.Regular);
            lblSubTitle.ForeColor = Color.FromArgb(148, 163, 184);
            lblSubTitle.AutoSize = false;
            lblSubTitle.Dock = DockStyle.Right;
            lblSubTitle.Width = 520;
            lblSubTitle.TextAlign = ContentAlignment.MiddleRight;
            lblSubTitle.Padding = new Padding(0, 0, 14, 0);
            pnlHeader.Controls.Add(lblSubTitle);

            // ==========================================
            // 2. TOP MODE BAR (İSİMLİ KLİPS - SIRALI KLİPS - MARKA TİPİ)
            // ==========================================
            Panel pnlModeBar = new Panel();
            pnlModeBar.Dock = DockStyle.Top;
            pnlModeBar.Height = 46;
            pnlModeBar.BackColor = Color.FromArgb(241, 245, 249);
            pnlModeBar.Padding = new Padding(10, 6, 10, 6);

            btnModeIsimli = new Button
            {
                Text = "●  İSİMLİ KLİPS",
                Location = new Point(10, 7),
                Size = new Size(150, 32),
                Font = new Font("Segoe UI", 9f, FontStyle.Bold),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnModeIsimli.FlatAppearance.BorderSize = 0;
            btnModeIsimli.Click += (s, e) => SetMode(GeneratorMode.IsimliKlips);
            pnlModeBar.Controls.Add(btnModeIsimli);

            btnModeSirali = new Button
            {
                Text = "●  SIRALI KLİPS",
                Location = new Point(166, 7),
                Size = new Size(150, 32),
                Font = new Font("Segoe UI", 9f, FontStyle.Bold),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnModeSirali.FlatAppearance.BorderSize = 0;
            btnModeSirali.Click += (s, e) => SetMode(GeneratorMode.SiraliKlips);
            pnlModeBar.Controls.Add(btnModeSirali);

            btnModeMarka = new Button
            {
                Text = "●  MARKA TİPİ (YUVARLAK)",
                Location = new Point(322, 7),
                Size = new Size(200, 32),
                Font = new Font("Segoe UI", 9f, FontStyle.Bold),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnModeMarka.FlatAppearance.BorderSize = 0;
            btnModeMarka.Click += (s, e) => SetMode(GeneratorMode.MarkaTipi);
            pnlModeBar.Controls.Add(btnModeMarka);

            lblModeDesc = new Label
            {
                Location = new Point(530, 13),
                Size = new Size(415, 20),
                Font = new Font("Segoe UI", 8.25f, FontStyle.Regular),
                ForeColor = Color.FromArgb(71, 85, 105),
                Text = "Kişiye özel tekli çift renkli klipsli halka (İsim Soyisim + Telefon No)"
            };
            pnlModeBar.Controls.Add(lblModeDesc);

            // ==========================================
            // 3. MAIN BODY CONTAINER
            // ==========================================
            Panel pnlBody = new Panel();
            pnlBody.Dock = DockStyle.Fill;
            pnlBody.Padding = new Padding(10);
            pnlBody.AutoScroll = false;

            this.Controls.Add(pnlBody);
            this.Controls.Add(pnlModeBar);
            this.Controls.Add(pnlHeader);
            pnlBody.BringToFront();

            // ------------------------------------------
            // SOL KOLON: GİRDİ VE AYARLAR (Width: 460)
            // ------------------------------------------

            // [A] İSİMLİ & MARKA İÇİN: 1. ÜST BÖLÜM
            grpTop = new GroupBox();
            grpTop.Text = " 1. Üst Bölüm (İsim Soyisim) Ayarları ";
            grpTop.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpTop.Location = new Point(10, 6);
            grpTop.Size = new Size(460, 172);
            pnlBody.Controls.Add(grpTop);

            Label lblTop = new Label { Text = "Üst Yazı (İsim / Marka):", Location = new Point(12, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            txtTopText = new TextBox { Location = new Point(12, 40), Size = new Size(300, 24), Font = new Font("Segoe UI", 10f, FontStyle.Bold), Text = "G.CIVAN" };
            txtTopText.TextChanged += TxtTopText_TextChanged;
            grpTop.Controls.Add(lblTop);
            grpTop.Controls.Add(txtTopText);

            chkSoftG = new CheckBox
            {
                Text = "Türkçe Harf Desteği (Kopmayan Manifold Glifler: Ğ, Ş, Ç, İ, Ö, Ü)",
                Location = new Point(145, 20),
                AutoSize = true,
                Font = new Font("Segoe UI", 8.25f, FontStyle.Bold),
                ForeColor = Color.FromArgb(220, 38, 38),
                Checked = false,
                Cursor = Cursors.Hand
            };
            chkSoftG.CheckedChanged += (s, e) =>
            {
                UpdateFontPreview();
            };
            grpTop.Controls.Add(chkSoftG);

            Label lblFilter = new Label { Text = "Font Filtrele:", Location = new Point(320, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            txtFontFilter = new TextBox { Location = new Point(320, 40), Size = new Size(126, 23), Font = new Font("Segoe UI", 8.5f) };
            grpTop.Controls.Add(lblFilter);
            grpTop.Controls.Add(txtFontFilter);

            Label lblFontTop = new Label { Text = "Yazı Tipi (Font):", Location = new Point(12, 68), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            cmbFontTop = new ComboBox { Location = new Point(12, 86), Size = new Size(310, 24), DropDownStyle = ComboBoxStyle.DropDownList, Font = new Font("Segoe UI", 8.5f) };
            btnLoadCustomFont = new Button
            {
                Location = new Point(330, 85),
                Size = new Size(116, 26),
                Text = "➕ Font Yükle",
                Font = new Font("Segoe UI", 8f, FontStyle.Bold),
                BackColor = Color.FromArgb(241, 245, 249),
                ForeColor = Color.FromArgb(15, 23, 42),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnLoadCustomFont.Click += BtnAddCustomFont_Click;
            grpTop.Controls.Add(lblFontTop);
            grpTop.Controls.Add(cmbFontTop);
            grpTop.Controls.Add(btnLoadCustomFont);

            Label lblHeightTop = new Label { Text = "Harf Boyu (mm):", Location = new Point(10, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numTextHeightTop = new NumericUpDown
            {
                Location = new Point(10, 134),
                Size = new Size(98, 22),
                DecimalPlaces = 2,
                Increment = 0.05m,
                Minimum = 2.00m,
                Maximum = 9.00m,
                Value = 3.40m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpTop.Controls.Add(lblHeightTop);
            grpTop.Controls.Add(numTextHeightTop);

            Label lblStretchTop = new Label { Text = "Dikey Boy (%):", Location = new Point(118, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numStretchTop = new NumericUpDown
            {
                Location = new Point(118, 134),
                Size = new Size(98, 22),
                DecimalPlaces = 0,
                Increment = 5m,
                Minimum = 70m,
                Maximum = 180m,
                Value = 100m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpTop.Controls.Add(lblStretchTop);
            grpTop.Controls.Add(numStretchTop);

            Label lblGapTop = new Label { Text = "Boşluk Gap (mm):", Location = new Point(226, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numCharGapTop = new NumericUpDown
            {
                Location = new Point(226, 134),
                Size = new Size(102, 22),
                DecimalPlaces = 2,
                Increment = 0.05m,
                Minimum = 0.00m,
                Maximum = 2.50m,
                Value = 0.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpTop.Controls.Add(lblGapTop);
            grpTop.Controls.Add(numCharGapTop);

            Label lblWidthTop = new Label { Text = "Kalınlık (mm):", Location = new Point(338, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numWidthScaleTop = new NumericUpDown
            {
                Location = new Point(338, 134),
                Size = new Size(108, 22),
                DecimalPlaces = 2,
                Increment = 0.02m,
                Minimum = 0.00m,
                Maximum = 0.50m,
                Value = 0.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpTop.Controls.Add(lblWidthTop);
            grpTop.Controls.Add(numWidthScaleTop);

            // [A] İSİMLİ & MARKA İÇİN: 2. ALT BÖLÜM
            grpBot = new GroupBox();
            grpBot.Text = " 2. Alt Bölüm (Telefon No) Ayarları ";
            grpBot.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpBot.Location = new Point(10, 184);
            grpBot.Size = new Size(460, 172);
            pnlBody.Controls.Add(grpBot);

            Label lblBot = new Label { Text = "Alt Yazı (Telefon / Seri No):", Location = new Point(12, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            txtBotText = new TextBox { Location = new Point(12, 40), Size = new Size(300, 24), Font = new Font("Segoe UI", 10f, FontStyle.Bold), Text = "0506 938 19 49" };
            txtBotText.TextChanged += TxtBotText_TextChanged;
            grpBot.Controls.Add(lblBot);
            grpBot.Controls.Add(txtBotText);

            btnSyncFonts = new Button
            {
                Location = new Point(320, 39),
                Size = new Size(126, 26),
                Text = "Üst Fontu Al",
                Font = new Font("Segoe UI", 8f, FontStyle.Bold),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnSyncFonts.Click += (s, e) =>
            {
                if (cmbFontTop.SelectedItem != null)
                {
                    cmbFontBot.SelectedItem = cmbFontTop.SelectedItem;
                    UpdateFontPreview();
                }
            };
            grpBot.Controls.Add(btnSyncFonts);

            Label lblFontBot = new Label { Text = "Yazı Tipi (Font):", Location = new Point(12, 68), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            cmbFontBot = new ComboBox { Location = new Point(12, 86), Size = new Size(310, 24), DropDownStyle = ComboBoxStyle.DropDownList, Font = new Font("Segoe UI", 8.5f) };
            btnOpenFontsFolder = new Button
            {
                Location = new Point(330, 85),
                Size = new Size(116, 26),
                Text = "Font Klasörü",
                Font = new Font("Segoe UI", 8f),
                BackColor = Color.FromArgb(241, 245, 249),
                ForeColor = Color.FromArgb(51, 65, 85),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnOpenFontsFolder.Click += BtnOpenFontsFolder_Click;
            grpBot.Controls.Add(lblFontBot);
            grpBot.Controls.Add(cmbFontBot);
            grpBot.Controls.Add(btnOpenFontsFolder);

            Label lblHeightBot = new Label { Text = "Harf Boyu (mm):", Location = new Point(10, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numTextHeightBot = new NumericUpDown
            {
                Location = new Point(10, 134),
                Size = new Size(98, 22),
                DecimalPlaces = 2,
                Increment = 0.05m,
                Minimum = 2.00m,
                Maximum = 9.00m,
                Value = 3.40m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpBot.Controls.Add(lblHeightBot);
            grpBot.Controls.Add(numTextHeightBot);

            Label lblStretchBot = new Label { Text = "Dikey Boy (%):", Location = new Point(118, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numStretchBot = new NumericUpDown
            {
                Location = new Point(118, 134),
                Size = new Size(98, 22),
                DecimalPlaces = 0,
                Increment = 5m,
                Minimum = 70m,
                Maximum = 180m,
                Value = 100m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpBot.Controls.Add(lblStretchBot);
            grpBot.Controls.Add(numStretchBot);

            Label lblGapBot = new Label { Text = "Boşluk Gap (mm):", Location = new Point(226, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numCharGapBot = new NumericUpDown
            {
                Location = new Point(226, 134),
                Size = new Size(102, 22),
                DecimalPlaces = 2,
                Increment = 0.05m,
                Minimum = 0.00m,
                Maximum = 2.50m,
                Value = 0.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpBot.Controls.Add(lblGapBot);
            grpBot.Controls.Add(numCharGapBot);

            Label lblWidthBot = new Label { Text = "Kalınlık (mm):", Location = new Point(338, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numWidthScaleBot = new NumericUpDown
            {
                Location = new Point(338, 134),
                Size = new Size(108, 22),
                DecimalPlaces = 2,
                Increment = 0.02m,
                Minimum = 0.00m,
                Maximum = 0.50m,
                Value = 0.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            grpBot.Controls.Add(lblWidthBot);
            grpBot.Controls.Add(numWidthScaleBot);

            // [B] SIRALI KLİPS İÇİN: NUMARALAR & PRESETLER KARTI
            grpSeqNums = new GroupBox();
            grpSeqNums.Text = " 1. Sıralı Numaralar & Hazır Paketler ";
            grpSeqNums.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpSeqNums.Location = new Point(10, 6);
            grpSeqNums.Size = new Size(460, 172);
            grpSeqNums.Visible = false;
            pnlBody.Controls.Add(grpSeqNums);

            btnPreset10 = new Button { Text = "01-10", Location = new Point(10, 24), Size = new Size(82, 28), Font = new Font("Segoe UI", 8.5f, FontStyle.Bold), FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnPreset20 = new Button { Text = "01-20", Location = new Point(98, 24), Size = new Size(82, 28), Font = new Font("Segoe UI", 8.5f, FontStyle.Bold), FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnPreset30 = new Button { Text = "01-30", Location = new Point(186, 24), Size = new Size(82, 28), Font = new Font("Segoe UI", 8.5f, FontStyle.Bold), FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnPreset40 = new Button { Text = "01-40", Location = new Point(274, 24), Size = new Size(82, 28), Font = new Font("Segoe UI", 8.5f, FontStyle.Bold), FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnPreset50 = new Button { Text = "01-50", Location = new Point(362, 24), Size = new Size(88, 28), Font = new Font("Segoe UI", 8.5f, FontStyle.Bold), FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };

            btnPreset10.Click += (s, e) => SetSeqRange(1, 10);
            btnPreset20.Click += (s, e) => SetSeqRange(1, 20);
            btnPreset30.Click += (s, e) => SetSeqRange(1, 30);
            btnPreset40.Click += (s, e) => SetSeqRange(1, 40);
            btnPreset50.Click += (s, e) => SetSeqRange(1, 50);

            grpSeqNums.Controls.Add(btnPreset10);
            grpSeqNums.Controls.Add(btnPreset20);
            grpSeqNums.Controls.Add(btnPreset30);
            grpSeqNums.Controls.Add(btnPreset40);
            grpSeqNums.Controls.Add(btnPreset50);

            Label lblSeqStart = new Label { Text = "Başlangıç No:", Location = new Point(12, 60), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numSeqStart = new NumericUpDown { Location = new Point(12, 78), Size = new Size(88, 23), Minimum = 1, Maximum = 999, Value = 1, Font = new Font("Segoe UI", 9f, FontStyle.Bold) };
            numSeqStart.ValueChanged += (s, e) => UpdateSeqTotal();

            Label lblSeqEnd = new Label { Text = "Bitiş No:", Location = new Point(110, 60), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numSeqEnd = new NumericUpDown { Location = new Point(110, 78), Size = new Size(88, 23), Minimum = 1, Maximum = 999, Value = 10, Font = new Font("Segoe UI", 9f, FontStyle.Bold) };
            numSeqEnd.ValueChanged += (s, e) => UpdateSeqTotal();

            chkIncludeGender = new CheckBox
            {
                Text = "DiŞi & ERKEK Ekstra Ekle (+2 Klips)",
                Location = new Point(216, 77),
                AutoSize = true,
                Checked = true,
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold)
            };
            chkIncludeGender.CheckedChanged += (s, e) => UpdateSeqTotal();

            lblSeqTotalCount = new Label
            {
                Location = new Point(12, 118),
                Size = new Size(436, 40),
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
                ForeColor = Color.FromArgb(15, 118, 110),
                Text = "Toplam: 12 Adet Sıralı Klips • Çift Tabla (Tabla 1: Siyah/Beyaz • Tabla 2: Çift Renk)"
            };

            grpSeqNums.Controls.Add(lblSeqStart);
            grpSeqNums.Controls.Add(numSeqStart);
            grpSeqNums.Controls.Add(lblSeqEnd);
            grpSeqNums.Controls.Add(numSeqEnd);
            grpSeqNums.Controls.Add(chkIncludeGender);
            grpSeqNums.Controls.Add(lblSeqTotalCount);

            // [B] SIRALI KLİPS İÇİN: FONT VE BOYUT KARTI
            grpSeqFont = new GroupBox();
            grpSeqFont.Text = " 2. Sıralı Yazı Tipi (Font) & Boyut Ayarları ";
            grpSeqFont.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpSeqFont.Location = new Point(10, 184);
            grpSeqFont.Size = new Size(460, 172);
            grpSeqFont.Visible = false;
            pnlBody.Controls.Add(grpSeqFont);

            Label lblSeqFont = new Label { Text = "Yazı Tipi (Font):", Location = new Point(12, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            cmbSeqFont = new ComboBox { Location = new Point(12, 40), Size = new Size(310, 24), DropDownStyle = ComboBoxStyle.DropDownList, Font = new Font("Segoe UI", 8.5f) };
            cmbSeqFont.SelectedIndexChanged += (s, e) => UpdateFontPreview();

            Label lblSeqFilter = new Label { Text = "Filtrele:", Location = new Point(330, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            txtSeqFontFilter = new TextBox { Location = new Point(330, 40), Size = new Size(116, 23), Font = new Font("Segoe UI", 8.5f) };

            btnSeqLoadCustomFont = new Button
            {
                Location = new Point(12, 72),
                Size = new Size(140, 26),
                Text = "➕ Özel Font Yükle",
                Font = new Font("Segoe UI", 8f, FontStyle.Bold),
                BackColor = Color.FromArgb(241, 245, 249),
                FlatStyle = FlatStyle.Flat,
                Cursor = Cursors.Hand
            };
            btnSeqLoadCustomFont.Click += BtnAddCustomFont_Click;

            Label lblSeqH = new Label { Text = "Harf Boyu (mm):", Location = new Point(10, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numSeqHeight = new NumericUpDown
            {
                Location = new Point(10, 134),
                Size = new Size(98, 22),
                DecimalPlaces = 2,
                Increment = 0.05m,
                Minimum = 2.00m,
                Maximum = 9.00m,
                Value = 7.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            numSeqHeight.ValueChanged += (s, e) => UpdateFontPreview();

            Label lblSeqStr = new Label { Text = "Dikey Boy (%):", Location = new Point(118, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numSeqStretch = new NumericUpDown
            {
                Location = new Point(118, 134),
                Size = new Size(98, 22),
                DecimalPlaces = 0,
                Increment = 5m,
                Minimum = 70m,
                Maximum = 180m,
                Value = 100m,
                Font = new Font("Segoe UI", 8.5f)
            };
            numSeqStretch.ValueChanged += (s, e) => UpdateFontPreview();

            Label lblSeqG = new Label { Text = "Boşluk Gap (mm):", Location = new Point(226, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numSeqGap = new NumericUpDown
            {
                Location = new Point(226, 134),
                Size = new Size(102, 22),
                DecimalPlaces = 2,
                Increment = 0.05m,
                Minimum = 0.00m,
                Maximum = 2.50m,
                Value = 0.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            numSeqGap.ValueChanged += (s, e) => UpdateFontPreview();

            Label lblSeqW = new Label { Text = "Kalınlık (mm):", Location = new Point(338, 116), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numSeqWidth = new NumericUpDown
            {
                Location = new Point(338, 134),
                Size = new Size(108, 22),
                DecimalPlaces = 2,
                Increment = 0.02m,
                Minimum = 0.00m,
                Maximum = 0.50m,
                Value = 0.00m,
                Font = new Font("Segoe UI", 8.5f)
            };
            numSeqWidth.ValueChanged += (s, e) => UpdateFontPreview();

            Label lblSeqLH = new Label { Text = "Katman (mm):", Location = new Point(226, 76), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            cmbSeqLayerHeight = new ComboBox
            {
                Location = new Point(338, 72),
                Size = new Size(108, 24),
                DropDownStyle = ComboBoxStyle.DropDownList,
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold)
            };
            cmbSeqLayerHeight.Items.AddRange(new object[] { "0.12 mm", "0.16 mm", "0.20 mm" });
            cmbSeqLayerHeight.SelectedIndex = 0;

            grpSeqFont.Controls.Add(lblSeqFont);
            grpSeqFont.Controls.Add(cmbSeqFont);
            grpSeqFont.Controls.Add(lblSeqFilter);
            grpSeqFont.Controls.Add(txtSeqFontFilter);
            grpSeqFont.Controls.Add(btnSeqLoadCustomFont);
            grpSeqFont.Controls.Add(lblSeqLH);
            grpSeqFont.Controls.Add(cmbSeqLayerHeight);
            grpSeqFont.Controls.Add(lblSeqH);
            grpSeqFont.Controls.Add(numSeqHeight);
            grpSeqFont.Controls.Add(lblSeqStr);
            grpSeqFont.Controls.Add(numSeqStretch);
            grpSeqFont.Controls.Add(lblSeqG);
            grpSeqFont.Controls.Add(numSeqGap);
            grpSeqFont.Controls.Add(lblSeqW);
            grpSeqFont.Controls.Add(numSeqWidth);

            // [C] ORTAK 3. BASKI, RENK VE GEOMETRİ AYARLARI KARTI
            grpParams = new GroupBox();
            grpParams.Text = " 3. Baskı, Renk & Geometri Ayarları ";
            grpParams.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpParams.Location = new Point(10, 362);
            grpParams.Size = new Size(460, 230);
            pnlBody.Controls.Add(grpParams);

            Label lblMat = new Label { Text = "Malzeme:", Location = new Point(12, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            cmbMaterial = new ComboBox { Location = new Point(12, 38), Size = new Size(100, 23), DropDownStyle = ComboBoxStyle.DropDownList, Font = new Font("Segoe UI", 8.5f, FontStyle.Bold) };
            cmbMaterial.Items.Add("PLA");
            cmbMaterial.Items.Add("PETG");
            cmbMaterial.SelectedIndex = 0;
            grpParams.Controls.Add(lblMat);
            grpParams.Controls.Add(cmbMaterial);

            Label lblEmboss = new Label { Text = "Dış Kabartma (Emboss):", Location = new Point(125, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numEmboss = new NumericUpDown { Location = new Point(125, 38), Size = new Size(150, 22), DecimalPlaces = 2, Increment = 0.05m, Minimum = 0.00m, Maximum = 1.00m, Value = 0.15m, Font = new Font("Segoe UI", 8.5f) };
            grpParams.Controls.Add(lblEmboss);
            grpParams.Controls.Add(numEmboss);

            Label lblEmbed = new Label { Text = "İç Gömülme (Embed):", Location = new Point(290, 22), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            numEmbed = new NumericUpDown { Location = new Point(290, 38), Size = new Size(156, 22), DecimalPlaces = 2, Increment = 0.05m, Minimum = 0.30m, Maximum = 1.00m, Value = 0.50m, Font = new Font("Segoe UI", 8.5f) };
            grpParams.Controls.Add(lblEmbed);
            grpParams.Controls.Add(numEmbed);

            Label lblColorTitle = new Label { Text = "Baskı Renkleri (Gövde & Kabartma Yazı):", Location = new Point(12, 70), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            cmbColorPresets = new ComboBox { Location = new Point(12, 88), Size = new Size(240, 23), DropDownStyle = ComboBoxStyle.DropDownList, Font = new Font("Segoe UI", 8.5f) };
            cmbColorPresets.Items.Add("Sarı / Kırmızı (Varsayılan)");
            cmbColorPresets.Items.Add("Beyaz / Siyah");
            cmbColorPresets.Items.Add("Siyah / Beyaz");
            cmbColorPresets.Items.Add("Mavi / Beyaz");
            cmbColorPresets.Items.Add("Turuncu / Siyah");
            cmbColorPresets.Items.Add("Yeşil / Beyaz");
            cmbColorPresets.Items.Add("Kırmızı / Beyaz");
            cmbColorPresets.Items.Add("Yeşil / Kırmızı / Beyaz");
            cmbColorPresets.Items.Add("Altın Sarı / Siyah");
            cmbColorPresets.Items.Add("Özel Renk Seç...");
            cmbColorPresets.SelectedIndex = 0;
            cmbColorPresets.SelectedIndexChanged += CmbColorPresets_SelectedIndexChanged;
            grpParams.Controls.Add(lblColorTitle);
            grpParams.Controls.Add(cmbColorPresets);

            btnColor1 = new Button { Location = new Point(255, 87), Size = new Size(32, 25), BackColor = currentColor1, FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnColor1.Click += BtnColor1_Click;
            grpParams.Controls.Add(btnColor1);

            btnColor2 = new Button { Location = new Point(291, 87), Size = new Size(32, 25), BackColor = currentColor2, FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnColor2.Click += BtnColor2_Click;
            grpParams.Controls.Add(btnColor2);

            btnColor3 = new Button { Location = new Point(327, 87), Size = new Size(32, 25), BackColor = currentColor3, FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnColor3.Click += BtnColor3_Click;
            grpParams.Controls.Add(btnColor3);

            btnSwapColors = new Button { Location = new Point(363, 87), Size = new Size(32, 25), Text = "⇄", Font = new Font("Segoe UI", 9f, FontStyle.Bold), FlatStyle = FlatStyle.Flat, Cursor = Cursors.Hand };
            btnSwapColors.Click += BtnSwapColors_Click;
            grpParams.Controls.Add(btnSwapColors);

            lblColorHint = new Label
            {
                Text = "R1 (Alt Gövde) • R2 (Üst Gövde) • R3 (Yazı Rengi - Tıklayıp Değiştirin)",
                Location = new Point(12, 116),
                AutoSize = true,
                Font = new Font("Segoe UI", 7.5f, FontStyle.Bold),
                ForeColor = Color.FromArgb(71, 85, 105)
            };
            grpParams.Controls.Add(lblColorHint);

            Label lblBak = new Label { Text = "Referans Şablon Dosyası (.3mf.bak):", Location = new Point(12, 140), AutoSize = true, Font = new Font("Segoe UI", 8f) };
            txtBakPath = new TextBox { Location = new Point(12, 158), Size = new Size(365, 23), Font = new Font("Segoe UI", 8f), Text = GetDefaultBakPath("BOS_KLIPS_9MM.3mf.bak") };
            btnBrowseBak = new Button { Location = new Point(383, 157), Size = new Size(63, 25), Text = "Gözat...", Font = new Font("Segoe UI", 8f) };
            btnBrowseBak.Click += BtnBrowseBak_Click;
            grpParams.Controls.Add(lblBak);
            grpParams.Controls.Add(txtBakPath);
            grpParams.Controls.Add(btnBrowseBak);

            Label lblTemplateHint = new Label
            {
                Text = "Not: Z=0.00 mm tabla kotu şablondaki matristen tam hizalanır.",
                Location = new Point(12, 190),
                AutoSize = true,
                Font = new Font("Segoe UI", 7.5f),
                ForeColor = Color.FromArgb(71, 85, 105)
            };
            grpParams.Controls.Add(lblTemplateHint);

            // ------------------------------------------
            // SAĞ KOLON: ÖNİZLEME, BUTONLAR & LOG (Width: 456)
            // ------------------------------------------

            // 4. CANLI ÖNİZLEME KARTI
            grpPreview = new GroupBox();
            grpPreview.Text = " 4. Canlı Önizleme (4.50 mm Halka Bant Ölçekli) ";
            grpPreview.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpPreview.Location = new Point(478, 6);
            grpPreview.Size = new Size(456, 172);
            pnlBody.Controls.Add(grpPreview);

            lblScaleInfo = new Label
            {
                Text = "Ölçü: Üst: 3.40mm (%76) | Alt: 3.40mm (%76) • 4.50mm Bant",
                Location = new Point(10, 20),
                Size = new Size(436, 16),
                Font = new Font("Segoe UI", 8f, FontStyle.Bold),
                ForeColor = Color.FromArgb(15, 118, 110),
                TextAlign = ContentAlignment.TopRight
            };
            grpPreview.Controls.Add(lblScaleInfo);

            pnlFontPreview = new DoubleBufferedPanel();
            pnlFontPreview.Location = new Point(10, 38);
            pnlFontPreview.Size = new Size(436, 122);
            pnlFontPreview.BackColor = Color.FromArgb(15, 23, 42);
            pnlFontPreview.Paint += PnlFontPreview_Paint;
            grpPreview.Controls.Add(pnlFontPreview);

            // 5. İŞLEM BUTONLARI KARTI
            GroupBox grpActions = new GroupBox();
            grpActions.Text = " 5. Üretim, Hız Profili ve Dosya İşlemleri ";
            grpActions.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpActions.Location = new Point(478, 184);
            grpActions.Size = new Size(456, 178);
            pnlBody.Controls.Add(grpActions);

            Label lblSpeedTitle = new Label { Text = "Baskı Hızı:", Location = new Point(10, 18), AutoSize = true, Font = new Font("Segoe UI", 8.5f, FontStyle.Bold) };
            grpActions.Controls.Add(lblSpeedTitle);

            rbSpeedFast = new RadioButton { Text = "🚀 Hızlı Baskı", Location = new Point(95, 17), Size = new Size(130, 20), Checked = true, Font = new Font("Segoe UI", 8.5f, FontStyle.Bold), ForeColor = Color.FromArgb(2, 132, 199), Cursor = Cursors.Hand };
            rbSpeedSlow = new RadioButton { Text = "🛡️ Yavaş Baskı", Location = new Point(235, 17), Size = new Size(130, 20), Font = new Font("Segoe UI", 8.5f), ForeColor = Color.FromArgb(71, 85, 105), Cursor = Cursors.Hand };
            grpActions.Controls.Add(rbSpeedFast);
            grpActions.Controls.Add(rbSpeedSlow);

            btnGenerate = new Button
            {
                Text = "▶  3MF DOSYASINI OLUŞTUR",
                Font = new Font("Segoe UI", 10.5f, FontStyle.Bold),
                BackColor = Color.FromArgb(2, 132, 199),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Location = new Point(10, 42),
                Size = new Size(436, 36),
                Cursor = Cursors.Hand
            };
            btnGenerate.FlatAppearance.BorderSize = 0;
            btnGenerate.Click += BtnGenerate_Click;
            grpActions.Controls.Add(btnGenerate);

            btnOpenOrca = new Button
            {
                Text = "OrcaSlicer'da Aç",
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
                BackColor = Color.FromArgb(241, 245, 249),
                ForeColor = Color.FromArgb(51, 65, 85),
                FlatStyle = FlatStyle.Flat,
                Location = new Point(10, 84),
                Size = new Size(213, 30),
                Cursor = Cursors.Hand
            };
            btnOpenOrca.Click += BtnOpenOrca_Click;
            grpActions.Controls.Add(btnOpenOrca);

            btnOpenFolder = new Button
            {
                Text = "Klasörü Aç",
                Font = new Font("Segoe UI", 8.5f),
                BackColor = Color.FromArgb(241, 245, 249),
                ForeColor = Color.FromArgb(51, 65, 85),
                FlatStyle = FlatStyle.Flat,
                Location = new Point(233, 84),
                Size = new Size(213, 30),
                Cursor = Cursors.Hand
            };
            btnOpenFolder.Click += BtnOpenFolder_Click;
            grpActions.Controls.Add(btnOpenFolder);

            btnLoad3mf = new Button
            {
                Text = "Ayarları .3MF'den Yükle...",
                Font = new Font("Segoe UI", 8f, FontStyle.Bold),
                BackColor = Color.FromArgb(241, 245, 249),
                ForeColor = Color.FromArgb(30, 41, 59),
                FlatStyle = FlatStyle.Flat,
                Location = new Point(10, 120),
                Size = new Size(213, 30),
                Cursor = Cursors.Hand
            };
            btnLoad3mf.Click += BtnLoad3mf_Click;
            grpActions.Controls.Add(btnLoad3mf);

            btnViewInfo = new Button
            {
                Text = "Bilgi Raporunu Aç (_bilgi.txt)",
                Font = new Font("Segoe UI", 8f),
                BackColor = Color.FromArgb(241, 245, 249),
                ForeColor = Color.FromArgb(51, 65, 85),
                FlatStyle = FlatStyle.Flat,
                Location = new Point(233, 120),
                Size = new Size(213, 30),
                Cursor = Cursors.Hand
            };
            btnViewInfo.Click += BtnViewInfo_Click;
            grpActions.Controls.Add(btnViewInfo);

            // 6. İŞLEM GÜNLÜĞÜ (LOG CONSOLE)
            GroupBox grpLog = new GroupBox();
            grpLog.Text = " 6. İşlem Günlüğü ";
            grpLog.Font = new Font("Segoe UI", 9f, FontStyle.Bold);
            grpLog.Location = new Point(478, 368);
            grpLog.Size = new Size(456, 224);
            pnlBody.Controls.Add(grpLog);

            txtLog = new TextBox
            {
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                Font = new Font("Consolas", 8f),
                BackColor = Color.FromArgb(15, 23, 42),
                ForeColor = Color.FromArgb(226, 232, 240),
                Location = new Point(10, 20),
                Size = new Size(436, 228)
            };
            grpLog.Controls.Add(txtLog);

            // Event wiring for instant preview
            cmbFontTop.SelectedIndexChanged += (s, e) => UpdateFontPreview();
            cmbFontBot.SelectedIndexChanged += (s, e) => UpdateFontPreview();
            numTextHeightTop.ValueChanged += (s, e) => UpdateFontPreview();
            numTextHeightBot.ValueChanged += (s, e) => UpdateFontPreview();
            numStretchTop.ValueChanged += (s, e) => UpdateFontPreview();
            numStretchBot.ValueChanged += (s, e) => UpdateFontPreview();
            numCharGapTop.ValueChanged += (s, e) => UpdateFontPreview();
            numCharGapBot.ValueChanged += (s, e) => UpdateFontPreview();
            numWidthScaleTop.ValueChanged += (s, e) => UpdateFontPreview();
            numWidthScaleBot.ValueChanged += (s, e) => UpdateFontPreview();

            // Font listesini baslat
            InitFontLists(cmbFontTop, cmbFontBot, cmbSeqFont, txtFontFilter, txtSeqFontFilter, "Trebuchet MS", "Trebuchet MS");
            
            // Son oturumdaki ayarları otomatik yükle
            LoadLastSettings();
            SetMode(currentMode);
            UpdateFontPreview();

            this.FormClosing += (s, e) => SaveLastSettings();

            AppendLog("Program hazır. Üst kısımdan klips tipini seçiniz: İSİMLİ KLİPS, SIRALI KLİPS veya MARKA TİPİ.");
            AppendLog("Baskı ayarları: Brim = outer 2mm / 0.14 gap, İlk Katman = 18 mm/s, Prime Tower = kompakt 22mm, Tam 2 Filament.");
        }

        private void SetSeqRange(int start, int end)
        {
            numSeqStart.Value = start;
            numSeqEnd.Value = end;
            UpdateSeqTotal();
        }

        private void UpdateSeqTotal()
        {
            int s = (int)numSeqStart.Value;
            int e = (int)numSeqEnd.Value;
            if (e < s) e = s;
            int count = (e - s + 1) + (chkIncludeGender.Checked ? 2 : 0);
            lblSeqTotalCount.Text = string.Format("Toplam: {0} Adet Sıralı Klips • Çift Tabla (Tabla 1: Siyah/Beyaz • Tabla 2: Çift Renk)", count);
            UpdateFontPreview();
        }

        public void SetMode(GeneratorMode newMode)
        {
            currentMode = newMode;

            // Update button styles
            Color activeBg = Color.FromArgb(2, 132, 199);
            Color activeFg = Color.White;
            Color inactiveBg = Color.FromArgb(241, 245, 249);
            Color inactiveFg = Color.FromArgb(51, 65, 85);

            btnModeIsimli.BackColor = (newMode == GeneratorMode.IsimliKlips) ? activeBg : inactiveBg;
            btnModeIsimli.ForeColor = (newMode == GeneratorMode.IsimliKlips) ? activeFg : inactiveFg;

            btnModeSirali.BackColor = (newMode == GeneratorMode.SiraliKlips) ? activeBg : inactiveBg;
            btnModeSirali.ForeColor = (newMode == GeneratorMode.SiraliKlips) ? activeFg : inactiveFg;

            btnModeMarka.BackColor = (newMode == GeneratorMode.MarkaTipi) ? activeBg : inactiveBg;
            btnModeMarka.ForeColor = (newMode == GeneratorMode.MarkaTipi) ? activeFg : inactiveFg;

            if (newMode == GeneratorMode.SiraliKlips)
            {
                grpTop.Visible = false;
                grpBot.Visible = false;
                grpSeqNums.Visible = true;
                grpSeqFont.Visible = true;
                if (btnColor3 != null) btnColor3.Visible = true;
                if (btnSwapColors != null) btnSwapColors.Location = new Point(363, 87);
                if (lblColorHint != null) lblColorHint.Text = "R1 (Alt Gövde) • R2 (Üst Gövde) • R3 (Yazı Rengi - Tıklayıp Değiştirin)";
                grpParams.Text = " 3. Baskı, Renk & Geometri (Sıralı Klips) ";
                lblModeDesc.Text = "Çift tablalı sıralı klips (01-10..50 + Dişi/Erkek • Tabla 1: Siyah/Beyaz, Tabla 2: Çift Renk)";
                btnGenerate.Text = "▶  SIRALI KLİPS 3MF (ÇİFT TABLA) OLUŞTUR";
                txtBakPath.Text = GetDefaultBakPath("BOS_KLIPS_9MM.3mf.bak");
            }
            else if (newMode == GeneratorMode.MarkaTipi)
            {
                grpTop.Visible = true;
                grpBot.Visible = true;
                grpSeqNums.Visible = false;
                grpSeqFont.Visible = false;
                if (btnColor3 != null) btnColor3.Visible = false;
                if (btnSwapColors != null) btnSwapColors.Location = new Point(327, 87);
                if (lblColorHint != null) lblColorHint.Text = "R1 (Alt Gövde) • R2 (Üst Gövde / Kabartma Yazı - Tıklayıp Değiştirin)";
                grpTop.Text = " 1. Üst Bölüm (Marka / İsim) Ayarları ";
                grpBot.Text = " 2. Alt Bölüm (Seri No / Telefon) Ayarları ";
                grpParams.Text = " 3. Baskı, Renk & Geometri (Dairesel Halka) ";
                lblModeDesc.Text = "360° kesintisiz yuvarlak dairesel halka (Dış çap: ~11.25 mm, İç çap: ~8.60 mm)";
                btnGenerate.Text = "▶  MARKA TİPİ 3MF OLUŞTUR";
                txtBakPath.Text = GetDefaultBakPath("MARKA_TIPI.3mf.bak");
            }
            else // IsimliKlips
            {
                grpTop.Visible = true;
                grpBot.Visible = true;
                grpSeqNums.Visible = false;
                grpSeqFont.Visible = false;
                if (btnColor3 != null) btnColor3.Visible = false;
                if (btnSwapColors != null) btnSwapColors.Location = new Point(327, 87);
                if (lblColorHint != null) lblColorHint.Text = "R1 (Alt Gövde) • R2 (Üst Gövde / Kabartma Yazı - Tıklayıp Değiştirin)";
                grpTop.Text = " 1. Üst Bölüm (İsim Soyisim) Ayarları ";
                grpBot.Text = " 2. Alt Bölüm (Telefon No) Ayarları ";
                grpParams.Text = " 3. Baskı, Renk & Geometri Ayarları ";
                lblModeDesc.Text = "Kişiye özel tekli çift renkli klipsli halka (258° simetrik kilit korumalı yay)";
                btnGenerate.Text = "▶  3MF DOSYASINI OLUŞTUR";
                txtBakPath.Text = GetDefaultBakPath("BOS_KLIPS_9MM.3mf.bak");
            }

            UpdateFontPreview();
        }

        private void InitFontLists(ComboBox cmbTop, ComboBox cmbBot, ComboBox cmbSeq, TextBox txtSearch, TextBox txtSeqSearch, string defaultTop, string defaultBot)
        {
            FontManager.EnsureInitialized();
            RefreshFontLists(defaultTop, defaultBot);

            txtSearch.TextChanged += (s, e) =>
            {
                string filter = txtSearch.Text.Trim();
                string curTop = cmbTop.SelectedItem != null ? cmbTop.SelectedItem.ToString() : defaultTop;
                string curBot = cmbBot.SelectedItem != null ? cmbBot.SelectedItem.ToString() : defaultBot;
                PopulateFonts(cmbTop, filter, curTop);
                PopulateFonts(cmbBot, filter, curBot);
                UpdateFontPreview();
            };

            if (txtSeqSearch != null)
            {
                txtSeqSearch.TextChanged += (s, e) =>
                {
                    string filter = txtSeqSearch.Text.Trim();
                    string curSeq = cmbSeq.SelectedItem != null ? cmbSeq.SelectedItem.ToString() : "Trebuchet MS";
                    PopulateFonts(cmbSeq, filter, curSeq);
                    UpdateFontPreview();
                };
            }
        }

        private void RefreshFontLists(string preferTop = null, string preferBot = null)
        {
            allSystemFonts.Clear();

            // 1. Önce özel / gömülü fontlar
            foreach (var cf in FontManager.GetCustomFontNames())
            {
                string displayName = cf + " [Özel]";
                if (!allSystemFonts.Contains(displayName)) allSystemFonts.Add(displayName);
            }

            // 2. Popüler 3D baskı fontları
            string[] topFonts = new string[] {
                "Trebuchet MS", "Arial", "Segoe UI", "Tahoma", "Bahnschrift", "Impact", "Verdana", "Consolas"
            };

            foreach (var tf in topFonts)
            {
                try {
                    using (FontFamily ff = new FontFamily(tf)) {
                        if (!allSystemFonts.Contains(tf)) allSystemFonts.Add(tf);
                    }
                } catch {}
            }

            // 3. Windows'un tüm diğer sistem fontları
            foreach (var ff in FontFamily.Families)
            {
                try {
                    if (!allSystemFonts.Contains(ff.Name))
                    {
                        allSystemFonts.Add(ff.Name);
                    }
                } catch {}
            }

            string curTop = preferTop ?? (cmbFontTop != null && cmbFontTop.SelectedItem != null ? cmbFontTop.SelectedItem.ToString() : "Trebuchet MS");
            string curBot = preferBot ?? (cmbFontBot != null && cmbFontBot.SelectedItem != null ? cmbFontBot.SelectedItem.ToString() : "Trebuchet MS");
            string curSeq = (cmbSeqFont != null && cmbSeqFont.SelectedItem != null) ? cmbSeqFont.SelectedItem.ToString() : "Trebuchet MS";
            string filter = txtFontFilter != null ? txtFontFilter.Text.Trim() : "";

            PopulateFonts(cmbFontTop, filter, curTop);
            PopulateFonts(cmbFontBot, filter, curBot);
            if (cmbSeqFont != null) PopulateFonts(cmbSeqFont, "", curSeq);
            UpdateFontPreview();
        }

        private void PopulateFonts(ComboBox cmb, string filter, string selectedFont)
        {
            if (cmb == null) return;
            cmb.BeginUpdate();
            cmb.Items.Clear();
            foreach (var fn in allSystemFonts)
            {
                if (string.IsNullOrEmpty(filter) || fn.IndexOf(filter, StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    cmb.Items.Add(fn);
                }
            }

            if (cmb.Items.Count > 0)
            {
                int selIdx = -1;
                if (!string.IsNullOrEmpty(selectedFont))
                {
                    string cleanTarget = FontManager.CleanFontName(selectedFont);
                    for (int i = 0; i < cmb.Items.Count; i++)
                    {
                        string itemStr = cmb.Items[i].ToString();
                        if (itemStr.Equals(selectedFont, StringComparison.OrdinalIgnoreCase) ||
                            FontManager.CleanFontName(itemStr).Equals(cleanTarget, StringComparison.OrdinalIgnoreCase))
                        {
                            selIdx = i;
                            break;
                        }
                    }
                }
                cmb.SelectedIndex = (selIdx >= 0) ? selIdx : 0;
            }
            cmb.EndUpdate();
        }

        private static Font CreateSafeFont(string fontName, float size)
        {
            return FontManager.CreateFont(fontName, size, FontStyle.Bold);
        }

        private void CmbColorPresets_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (cmbColorPresets.SelectedIndex)
            {
                case 0: // Sarı / Kırmızı / Siyah (Varsayılan)
                    SetColors(Color.FromArgb(234, 179, 8), Color.FromArgb(220, 38, 38), Color.FromArgb(24, 24, 27));
                    break;
                case 1: // Beyaz / Siyah / Kırmızı
                    SetColors(Color.FromArgb(248, 250, 252), Color.FromArgb(24, 24, 27), Color.FromArgb(220, 38, 38));
                    break;
                case 2: // Siyah / Beyaz / Sarı
                    SetColors(Color.FromArgb(24, 24, 27), Color.FromArgb(248, 250, 252), Color.FromArgb(234, 179, 8));
                    break;
                case 3: // Mavi / Beyaz / Kırmızı
                    SetColors(Color.FromArgb(37, 99, 235), Color.FromArgb(248, 250, 252), Color.FromArgb(220, 38, 38));
                    break;
                case 4: // Turuncu / Siyah / Beyaz
                    SetColors(Color.FromArgb(234, 88, 12), Color.FromArgb(24, 24, 27), Color.FromArgb(248, 250, 252));
                    break;
                case 5: // Yeşil / Beyaz / Siyah
                    SetColors(Color.FromArgb(22, 163, 74), Color.FromArgb(248, 250, 252), Color.FromArgb(24, 24, 27));
                    break;
                case 6: // Kırmızı / Beyaz / Siyah
                    SetColors(Color.FromArgb(220, 38, 38), Color.FromArgb(248, 250, 252), Color.FromArgb(24, 24, 27));
                    break;
                case 7: // Yeşil / Kırmızı / Beyaz
                    SetColors(Color.FromArgb(22, 163, 74), Color.FromArgb(220, 38, 38), Color.FromArgb(248, 250, 252));
                    break;
                case 8: // Altın Sarı / Siyah / Beyaz
                    SetColors(Color.FromArgb(234, 179, 8), Color.FromArgb(24, 24, 27), Color.FromArgb(248, 250, 252));
                    break;
            }
        }

        private void SetColors(Color c1, Color c2, Color c3, bool updateComboToCustom = false)
        {
            currentColor1 = c1;
            currentColor2 = c2;
            currentColor3 = c3;
            if (btnColor1 != null) btnColor1.BackColor = c1;
            if (btnColor2 != null) btnColor2.BackColor = c2;
            if (btnColor3 != null) btnColor3.BackColor = c3;
            if (updateComboToCustom && cmbColorPresets != null)
            {
                cmbColorPresets.SelectedIndex = cmbColorPresets.Items.Count - 1;
            }
            UpdateFontPreview();
        }

        private void SetColors(Color c1, Color c2, bool updateComboToCustom = false)
        {
            SetColors(c1, c2, currentColor3, updateComboToCustom);
        }

        private void BtnColor3_Click(object sender, EventArgs e)
        {
            using (ColorDialog cd = new ColorDialog())
            {
                cd.Color = currentColor3;
                cd.FullOpen = true;
                if (cd.ShowDialog() == DialogResult.OK)
                {
                    SetColors(currentColor1, currentColor2, cd.Color, true);
                }
            }
        }

        private void BtnColor1_Click(object sender, EventArgs e)
        {
            using (ColorDialog cd = new ColorDialog())
            {
                cd.Color = currentColor1;
                cd.FullOpen = true;
                if (cd.ShowDialog() == DialogResult.OK)
                {
                    SetColors(cd.Color, currentColor2, true);
                }
            }
        }

        private void BtnColor2_Click(object sender, EventArgs e)
        {
            using (ColorDialog cd = new ColorDialog())
            {
                cd.Color = currentColor2;
                cd.FullOpen = true;
                if (cd.ShowDialog() == DialogResult.OK)
                {
                    SetColors(currentColor1, cd.Color, true);
                }
            }
        }

        private void BtnSwapColors_Click(object sender, EventArgs e)
        {
            SetColors(currentColor2, currentColor1, true);
        }

        private static string ColorToHex(Color c)
        {
            return string.Format("#{0:X2}{1:X2}{2:X2}", c.R, c.G, c.B);
        }

        private void PnlFontPreview_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

            int w = pnlFontPreview.ClientSize.Width;
            int h = pnlFontPreview.ClientSize.Height;
            int gap = 4;
            int halfH = (h - gap) / 2;

            const double bandHeightMm = 4.50;

            if (currentMode == GeneratorMode.SiraliKlips)
            {
                // SIRALI KLİPS ÖNİZLEMESİ (Tabla 1 ve Tabla 2 Örnekleri)
                // Gerçek klips toplam 9.00 mm yüksekliktedir. Numara tam ortadaki (Z=4.5mm) birleşim hattında yer alır.
                const double klipsHeightMm = 9.00;
                string seqFont = cmbSeqFont != null && cmbSeqFont.SelectedItem != null ? cmbSeqFont.SelectedItem.ToString() : "Trebuchet MS";
                double seqH = numSeqHeight != null ? (double)numSeqHeight.Value : 4.20;
                double seqStr = numSeqStretch != null ? (double)numSeqStretch.Value / 100.0 : 1.0;
                double seqG = numSeqGap != null ? (double)numSeqGap.Value : 0.00;
                double seqW = numSeqWidth != null ? (double)numSeqWidth.Value : 0.00;

                string sampleNum = numSeqStart != null ? ((int)numSeqStart.Value).ToString("D2") : "01";

                // 1. Tabla 1 (Düz Renk Gövde [R1], Yazı [R3] • 9.00 mm Klips)
                Rectangle t1Rect = new Rectangle(0, 0, w, halfH);
                using (Brush bg = new SolidBrush(currentColor1))
                using (Brush txt = new SolidBrush(currentColor3))
                using (Pen borderPen = new Pen(Color.FromArgb(80, 255, 255, 255), 1))
                {
                    g.FillRectangle(bg, t1Rect);
                    g.DrawRectangle(borderPen, t1Rect.X, t1Rect.Y, t1Rect.Width - 1, t1Rect.Height - 1);
                    string stretchBadge = Math.Abs(seqStr - 1.0) > 0.01 ? string.Format(" • %{0:0}", seqStr * 100.0) : "";
                    DrawBadge(g, "TABLA 1 (Düz Gövde [R1] / Yazı [R3] • 9.00 mm Klips) • Örnek: " + sampleNum + " [" + seqFont + " • " + seqH.ToString("F2", CultureInfo.InvariantCulture) + " mm" + stretchBadge + "]", t1Rect.X + 8, t1Rect.Y + 4, currentColor3);
                    DrawScaledProportionalText(g, sampleNum, seqFont, txt, t1Rect, seqH, klipsHeightMm, seqW, seqG, seqStr);
                }

                // 2. Tabla 2 (Çift Renk Gövde: Üst [R2], Alt [R1], Yazı [R3] • 9.00 mm Klips)
                Rectangle t2Rect = new Rectangle(0, halfH + gap, w, halfH);
                int t2SplitY = t2Rect.Y + t2Rect.Height / 2;
                Rectangle t2UpperRect = new Rectangle(t2Rect.X, t2Rect.Y, t2Rect.Width, t2Rect.Height / 2);
                Rectangle t2LowerRect = new Rectangle(t2Rect.X, t2SplitY, t2Rect.Width, t2Rect.Height - t2Rect.Height / 2);

                using (Brush bgUpper = new SolidBrush(currentColor2))
                using (Brush bgLower = new SolidBrush(currentColor1))
                using (Brush txt = new SolidBrush(currentColor3))
                using (Pen borderPen = new Pen(Color.FromArgb(100, 0, 0, 0), 1))
                using (Pen seamPen = new Pen(Color.FromArgb(160, 0, 0, 0), 1))
                {
                    // Üst yarı (4.5 - 9.0 mm): Renk 2 (Örn: Yeşil / Kırmızı)
                    g.FillRectangle(bgUpper, t2UpperRect);
                    // Alt yarı (0.0 - 4.5 mm): Renk 1 (Örn: Yeşil / Kırmızı)
                    g.FillRectangle(bgLower, t2LowerRect);
                    // Orta birleşim çizgisi (Z = 4.5 mm dikiş hattı)
                    g.DrawLine(seamPen, t2Rect.Left, t2SplitY, t2Rect.Right, t2SplitY);
                    g.DrawRectangle(borderPen, t2Rect.X, t2Rect.Y, t2Rect.Width - 1, t2Rect.Height - 1);

                    string stretchBadge = Math.Abs(seqStr - 1.0) > 0.01 ? string.Format(" • %{0:0}", seqStr * 100.0) : "";
                    DrawBadge(g, "TABLA 2 (Çift Renk Gövde [R1/R2] / Yazı [R3] • 9.00 mm Klips) • Örnek: " + sampleNum + " [" + seqFont + " • " + seqH.ToString("F2", CultureInfo.InvariantCulture) + " mm" + stretchBadge + "]", t2Rect.X + 8, t2Rect.Y + 4, currentColor3);
                    DrawScaledProportionalText(g, sampleNum, seqFont, txt, t2Rect, seqH, klipsHeightMm, seqW, seqG, seqStr);
                }
            }
            else
            {
                // İSİMLİ KLİPS VEYA MARKA TİPİ ÖNİZLEMESİ
                string topStr = string.IsNullOrEmpty(txtTopText.Text) ? (currentMode == GeneratorMode.MarkaTipi ? "TK.ŞMP.2026" : "G.CIVAN") : txtTopText.Text;
                string botStr = string.IsNullOrEmpty(txtBotText.Text) ? (currentMode == GeneratorMode.MarkaTipi ? "TR-34-1923" : "0506 938 19 49") : txtBotText.Text;

                bool useSoftG = (chkSoftG != null && chkSoftG.Checked);
                topStr = ProcessTextWithSoftG(topStr, useSoftG);
                botStr = ProcessTextWithSoftG(botStr, useSoftG);

                string fontTop = cmbFontTop != null && cmbFontTop.SelectedItem != null ? cmbFontTop.SelectedItem.ToString() : "Trebuchet MS";
                string fontBot = cmbFontBot != null && cmbFontBot.SelectedItem != null ? cmbFontBot.SelectedItem.ToString() : "Trebuchet MS";
                
                double textHeightTop = numTextHeightTop != null ? (double)numTextHeightTop.Value : 3.40;
                if (textHeightTop <= 0.5) textHeightTop = 3.40;
                double textHeightBot = numTextHeightBot != null ? (double)numTextHeightBot.Value : 3.40;
                if (textHeightBot <= 0.5) textHeightBot = 3.40;

                double boldnessTop = numWidthScaleTop != null ? (double)numWidthScaleTop.Value : 0.00;
                double boldnessBot = numWidthScaleBot != null ? (double)numWidthScaleBot.Value : 0.00;

                double gapTop = numCharGapTop != null ? (double)numCharGapTop.Value : 0.00;
                double gapBot = numCharGapBot != null ? (double)numCharGapBot.Value : 0.00;

                double stretchTop = numStretchTop != null ? (double)numStretchTop.Value / 100.0 : 1.0;
                double stretchBot = numStretchBot != null ? (double)numStretchBot.Value / 100.0 : 1.0;

                string modeTag = currentMode == GeneratorMode.MarkaTipi ? "MARKA TİPİ (360° Dairesel)" : "İSİMLİ KLİPS";

                // 1. Üst Bant (Gövde: Color 2, Yazı: Color 1)
                Rectangle topRect = new Rectangle(0, 0, w, halfH);
                using (Brush bgTop = new SolidBrush(currentColor2))
                using (Brush textTop = new SolidBrush(currentColor1))
                using (Pen borderPen = new Pen(Color.FromArgb(120, currentColor2.R > 40 ? currentColor2.R - 30 : 0, currentColor2.G > 40 ? currentColor2.G - 30 : 0, currentColor2.B > 40 ? currentColor2.B - 30 : 0), 1))
                {
                    g.FillRectangle(bgTop, topRect);
                    g.DrawRectangle(borderPen, topRect.X, topRect.Y, topRect.Width - 1, topRect.Height - 1);

                    string stretchTopStr = Math.Abs(stretchTop - 1.0) > 0.01 ? string.Format(" • %{0:0}", stretchTop * 100.0) : "";
                    DrawBadge(g, modeTag + " • ÜST BANT [" + fontTop + " • " + textHeightTop.ToString("F2", CultureInfo.InvariantCulture) + " mm" + stretchTopStr + "]", topRect.X + 8, topRect.Y + 4, currentColor1);

                    DrawScaledProportionalText(g, topStr, fontTop, textTop, topRect, textHeightTop, bandHeightMm, boldnessTop, gapTop, stretchTop, useSoftG);
                }

                // 2. Alt Bant (Gövde: Color 1, Yazı: Color 2)
                Rectangle botRect = new Rectangle(0, halfH + gap, w, halfH);
                using (Brush bgBot = new SolidBrush(currentColor1))
                using (Brush textBot = new SolidBrush(currentColor2))
                using (Pen borderPen = new Pen(Color.FromArgb(120, currentColor1.R > 40 ? currentColor1.R - 30 : 0, currentColor1.G > 40 ? currentColor1.G - 30 : 0, currentColor1.B > 40 ? currentColor1.B - 30 : 0), 1))
                {
                    g.FillRectangle(bgBot, botRect);
                    g.DrawRectangle(borderPen, botRect.X, botRect.Y, botRect.Width - 1, botRect.Height - 1);

                    string stretchBotStr = Math.Abs(stretchBot - 1.0) > 0.01 ? string.Format(" • %{0:0}", stretchBot * 100.0) : "";
                    DrawBadge(g, modeTag + " • ALT BANT [" + fontBot + " • " + textHeightBot.ToString("F2", CultureInfo.InvariantCulture) + " mm" + stretchBotStr + "]", botRect.X + 8, botRect.Y + 4, currentColor2);

                    DrawScaledProportionalText(g, botStr, fontBot, textBot, botRect, textHeightBot, bandHeightMm, boldnessBot, gapBot, stretchBot, useSoftG);
                }
            }
        }

        private void DrawBadge(Graphics g, string label, float x, float y, Color fg)
        {
            using (Font badgeFont = new Font("Segoe UI", 6.5f, FontStyle.Bold))
            using (Brush badgeBrush = new SolidBrush(Color.FromArgb(200, fg)))
            {
                g.DrawString(label, badgeFont, badgeBrush, new PointF(x, y));
            }
        }

        private void DrawScaledProportionalText(Graphics g, string text, string fontName, Brush brush, Rectangle rect, double textHeightMm, double bandHeightMm, double boldnessMm = 0.0, double charGapMm = 0.0, double verticalStretch = 1.0, bool useTurkishFix = false)
        {
            if (string.IsNullOrEmpty(text)) return;

            float stretchY = (float)verticalStretch;
            if (stretchY <= 0.1f) stretchY = 1.0f;

            float pxPerMm = (float)rect.Height / (float)bandHeightMm;
            float targetHeightPx = (float)(textHeightMm * pxPerMm);
            if (targetHeightPx < 4f) targetHeightPx = 4f;

            float extraGapPx = (float)(charGapMm * pxPerMm);
            float boldnessPx = (float)(boldnessMm * pxPerMm);
            if (boldnessPx < 0f) boldnessPx = 0f;

            FontStyle style = FontStyle.Bold;
            FontFamily ff = FontManager.GetFontFamily(fontName);
            if (!ff.IsStyleAvailable(FontStyle.Bold)) style = FontStyle.Regular;

            float refEmSize = 100f;
            float capHeightAtRef = 70f;
            try
            {
                using (GraphicsPath gp = new GraphicsPath())
                {
                    gp.AddString("H0", ff, (int)style, refEmSize, new PointF(0, 0), StringFormat.GenericTypographic);
                    RectangleF b = gp.GetBounds();
                    if (b.Height > 10f) capHeightAtRef = b.Height;
                }
            }
            catch { }

            float emSize = refEmSize * (targetHeightPx / capHeightAtRef);

            using (Font finalFont = FontManager.CreateFont(fontName, emSize, style, GraphicsUnit.Pixel))
            {
                List<RectangleF> naturalBounds = GetNaturalCharBounds(g, text, finalFont, StringFormat.GenericTypographic);

                List<char> chars = new List<char>();
                List<GraphicsPath> charPaths = new List<GraphicsPath>();
                List<float> charOriginsX = new List<float>();

                float minLeftPx = float.MaxValue;
                float maxRightPx = float.MinValue;
                float textMinY = float.MaxValue;
                float textMaxY = float.MinValue;
                int gIdx = 0;

                for (int i = 0; i < text.Length; i++)
                {
                    char c = text[i];
                    if (c == ' ') continue;
                    if (i >= naturalBounds.Count) break;

                    RectangleF nb = naturalBounds[i];

                    GraphicsPath gpChar = CreateCharPath(c, ff, (int)style, emSize, StringFormat.GenericTypographic, useTurkishFix);
                    RectangleF b = gpChar.GetBounds();

                    float originX = nb.X + (gIdx * extraGapPx);
                    chars.Add(c);
                    charPaths.Add(gpChar);
                    charOriginsX.Add(originX);

                    float cLeft = originX + b.Left;
                    float cRight = originX + b.Right + (boldnessPx * 2.0f);
                    if (cLeft < minLeftPx) minLeftPx = cLeft;
                    if (cRight > maxRightPx) maxRightPx = cRight;

                    if (b.Top < textMinY) textMinY = b.Top;
                    if (b.Bottom > textMaxY) textMaxY = b.Bottom;

                    gIdx++;
                }

                if (chars.Count == 0) return;

                float totalW = maxRightPx - minLeftPx;
                float textCenterY = (textMinY + textMaxY) * 0.5f;

                float startX = rect.X + (rect.Width - totalW) / 2f - minLeftPx;
                float cy = rect.Y + rect.Height / 2f;

                GraphicsState clipState = g.Save();
                g.SetClip(rect);

                try
                {
                    for (int i = 0; i < chars.Count; i++)
                    {
                        char c = chars[i];
                        GraphicsPath gp = charPaths[i];
                        float originX = charOriginsX[i];

                        if (gp != null)
                        {
                            GraphicsState gState = g.Save();
                            try
                            {
                                g.TranslateTransform(startX + originX, cy);
                                if (Math.Abs(stretchY - 1.0f) > 0.001f)
                                {
                                    g.ScaleTransform(1.0f, stretchY);
                                }
                                g.TranslateTransform(0, -textCenterY);

                                g.FillPath(brush, gp);

                                if (boldnessPx > 0.05f)
                                {
                                    using (Pen pen = new Pen(brush, boldnessPx * 2.0f))
                                    {
                                        pen.LineJoin = LineJoin.Round;
                                        g.DrawPath(pen, gp);
                                    }
                                }
                            }
                            finally
                            {
                                g.Restore(gState);
                                gp.Dispose();
                            }
                        }
                    }
                }
                finally
                {
                    g.Restore(clipState);
                }
            }
        }

        private void UpdateFontPreview()
        {
            if (currentMode == GeneratorMode.SiraliKlips)
            {
                string seqFont = cmbSeqFont != null && cmbSeqFont.SelectedItem != null ? cmbSeqFont.SelectedItem.ToString() : "Trebuchet MS";
                double seqH = numSeqHeight != null ? (double)numSeqHeight.Value : 3.80;
                double seqStr = numSeqStretch != null ? (double)numSeqStretch.Value : 100.0;
                if (grpPreview != null) grpPreview.Text = " 4. Sıralı Klips Canlı Önizleme (" + seqFont + " • 9.00 mm Klips / Tabla 1 & 2) ";
                if (lblScaleInfo != null)
                {
                    lblScaleInfo.Text = string.Format(CultureInfo.InvariantCulture,
                "{0} • {1:F2} mm • 9.00 mm Klips • Çift Tabla (1: Düz • 2: Çift)",
                seqFont, seqH);
                }
            }
            else
            {
                string fontTop = cmbFontTop != null && cmbFontTop.SelectedItem != null ? cmbFontTop.SelectedItem.ToString() : "Trebuchet MS";
                string fontBot = cmbFontBot != null && cmbFontBot.SelectedItem != null ? cmbFontBot.SelectedItem.ToString() : "Trebuchet MS";
                double hTop = numTextHeightTop != null ? (double)numTextHeightTop.Value : 3.40;
                double hBot = numTextHeightBot != null ? (double)numTextHeightBot.Value : 3.40;

                try
                {
                    if (txtTopText != null) txtTopText.Font = CreateSafeFont(fontTop, 10.5f);
                    if (txtBotText != null) txtBotText.Font = CreateSafeFont(fontBot, 10.5f);
                }
                catch { }

                if (grpPreview != null)
                {
                    string prefix = currentMode == GeneratorMode.MarkaTipi ? " 4. Canlı Önizleme (360° Dairesel: " : " 4. Canlı Önizleme (Klipsli: ";
                    if (string.Equals(fontTop, fontBot, StringComparison.OrdinalIgnoreCase))
                        grpPreview.Text = prefix + fontTop + " • 4.50 mm Bant) ";
                    else
                        grpPreview.Text = prefix + "Üst: " + fontTop + " | Alt: " + fontBot + ") ";
                }

                if (lblScaleInfo != null)
                {
                    double strTop = numStretchTop != null ? (double)numStretchTop.Value : 100.0;
                    double strBot = numStretchBot != null ? (double)numStretchBot.Value : 100.0;
                    lblScaleInfo.Text = string.Format(CultureInfo.InvariantCulture,
                "Üst: {0:F2} mm | Alt: {1:F2} mm • Çift Tabla (1: Düz • 2: Çift)",
                        hTop, hBot);
                }
            }

            if (pnlFontPreview != null)
            {
                pnlFontPreview.Invalidate();
            }
        }

        private void TxtTopText_TextChanged(object sender, EventArgs e)
        {
            // Kullanıcı ne yazarsa yazsın metin kutusundaki yazısına DOKUNMA!
            UpdateFontPreview();
        }

        private void TxtBotText_TextChanged(object sender, EventArgs e)
        {
            if (isUpdatingBotText) return;

            string original = txtBotText.Text;
            // Metin kutusuna dokunma, sadece rakamlari al
            string converted = original;

            if (currentMode != GeneratorMode.MarkaTipi)
            {
                string digitsOnly = "";
                foreach (char c in converted) if (char.IsDigit(c)) digitsOnly += c;

                if (digitsOnly.Length == 10 || digitsOnly.Length == 11 || (digitsOnly.Length >= 12 && digitsOnly.StartsWith("90")) || digitsOnly.Length == 15)
                {
                    string formatted = FormatPhoneNumber(digitsOnly);
                    if (!string.IsNullOrEmpty(formatted)) converted = formatted;
                }
            }

            if (converted != original)
            {
                isUpdatingBotText = true;
                int origPos = txtBotText.SelectionStart;
                txtBotText.Text = converted;
                txtBotText.SelectionStart = MapCursorPosition(original, converted, origPos);
                isUpdatingBotText = false;
            }

            UpdateFontPreview();
        }

        private static int MapCursorPosition(string original, string converted, int origPos)
        {
            if (string.IsNullOrEmpty(original) || string.IsNullOrEmpty(converted)) return converted != null ? converted.Length : 0;
            if (origPos >= original.Length) return converted.Length;

            int digitsBefore = 0;
            for (int i = 0; i < Math.Min(origPos, original.Length); i++)
            {
                if (char.IsDigit(original[i])) digitsBefore++;
            }
            if (digitsBefore == 0) return Math.Min(origPos, converted.Length);

            int countedDigits = 0;
            for (int i = 0; i < converted.Length; i++)
            {
                if (char.IsDigit(converted[i]))
                {
                    countedDigits++;
                    if (countedDigits == digitsBefore) return Math.Min(i + 1, converted.Length);
                }
            }
            return converted.Length;
        }

        private void AppendLog(string msg)
        {
            if (txtLog.InvokeRequired)
            {
                txtLog.Invoke(new Action<string>(AppendLog), msg);
                return;
            }
            txtLog.AppendText(msg + Environment.NewLine);
        }

        private void BtnBrowseBak_Click(object sender, EventArgs e)
        {
            EnsureEmbeddedTemplatesExtracted();
            using (OpenFileDialog ofd = new OpenFileDialog())
            {
                ofd.Title = "Şablon Dosyasını Seçin (.3mf.bak)";
                ofd.Filter = "3MF Dosyaları (*.3mf;*.3mf.bak)|*.3mf;*.3mf.bak|Tüm Dosyalar (*.*)|*.*";
                string tDir = GetTemplatesDir();
                if (Directory.Exists(tDir)) ofd.InitialDirectory = tDir;
                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    txtBakPath.Text = ofd.FileName;
                }
            }
        }

        private void BtnOpenFolder_Click(object sender, EventArgs e)
        {
            string targetDir = AppDomain.CurrentDomain.BaseDirectory;
            if (!string.IsNullOrEmpty(lastGeneratedPath) && File.Exists(lastGeneratedPath))
            {
                System.Diagnostics.Process.Start("explorer.exe", "/select,\"" + lastGeneratedPath + "\"");
            }
            else
            {
                System.Diagnostics.Process.Start("explorer.exe", targetDir);
            }
        }

        private void BtnOpenOrca_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty(lastGeneratedPath) || !File.Exists(lastGeneratedPath))
            {
                MessageBox.Show("Henüz üretilmiş bir .3mf dosyası bulunmamaktadır. Önce dosyayı oluşturunuz.",
                    "Bilgi", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            try
            {
                System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo
                {
                    FileName = lastGeneratedPath,
                    UseShellExecute = true
                });
            }
            catch (Exception ex)
            {
                MessageBox.Show("Dosya varsayılan dilimleyici ile açılamadı: " + ex.Message, "Hata", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private void BtnGenerate_Click(object sender, EventArgs e)
        {
            string bakPath = txtBakPath.Text.Trim();
            if (!File.Exists(bakPath))
            {
                MessageBox.Show(string.Format("Şablon dosya bulunamadı:\n{0}", bakPath), "Hata", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            string exeDir = Path.GetDirectoryName(Application.ExecutablePath);
            if (string.IsNullOrEmpty(exeDir)) exeDir = AppDomain.CurrentDomain.BaseDirectory;
            string tplDirNorm = "";
            try { tplDirNorm = Path.GetFullPath(GetTemplatesDir()).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar); } catch { }
            string bakDirNorm = "";
            try { bakDirNorm = Path.GetFullPath(Path.GetDirectoryName(bakPath) ?? "").TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar); } catch { }
            bool usingInternalTemplate = string.IsNullOrEmpty(bakDirNorm) || !Directory.Exists(bakDirNorm) ||
                string.Equals(bakDirNorm, tplDirNorm, StringComparison.OrdinalIgnoreCase);
            string baseDir = usingInternalTemplate ? exeDir : bakDirNorm;
            double emboss = (double)numEmboss.Value;
            double embed = (double)numEmbed.Value;
            string material = cmbMaterial.SelectedItem != null ? cmbMaterial.SelectedItem.ToString() : "PLA";
            string color1Hex = ColorToHex(currentColor1);
            string color2Hex = ColorToHex(currentColor2);
            string color3Hex = ColorToHex(currentColor3);

            string defaultFileName = "";
            bool isMarka = (currentMode == GeneratorMode.MarkaTipi);

            if (currentMode == GeneratorMode.SiraliKlips)
            {
                int sNum = (int)numSeqStart.Value;
                int eNum = (int)numSeqEnd.Value;
                if (eNum < sNum) eNum = sNum;
                bool incGen = chkIncludeGender.Checked;
                defaultFileName = string.Format("SIRALI_KLIPS_{0:D2}-{1:D2}{2}.3mf", sNum, eNum, incGen ? "_DISI_ERKEK" : "");
            }
            else
            {
                string topText = txtTopText.Text.Trim();
                string botText = txtBotText.Text.Trim();

                bool useSoftG = (chkSoftG != null && chkSoftG.Checked);
                topText = ProcessTextWithSoftG(topText, useSoftG);
                botText = ProcessTextWithSoftG(botText, useSoftG);

                if (string.IsNullOrEmpty(topText) || string.IsNullOrEmpty(botText))
                {
                    MessageBox.Show("Lütfen üst ve alt yazı alanlarını boş bırakmayınız.", "Uyarı", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                string safeName = MakeSafeFileName(topText);
                if (isMarka)
                    defaultFileName = safeName + " Marka Tipi.3mf";
                else
                    defaultFileName = safeName + " Klips.3mf";
            }

            // Dosya ismi ve kayıt yeri seçimi ekranı (SaveFileDialog)
            string outPath = "";
            using (SaveFileDialog sfd = new SaveFileDialog())
            {
                sfd.Title = isMarka ? "Marka Tipi 3MF Dosyasını Kaydet" : 
                    (currentMode == GeneratorMode.SiraliKlips ? "Sıralı Klips 3MF Dosyasını Kaydet" : "İsimli Klips 3MF Dosyasını Kaydet");
                sfd.Filter = "3MF Proje Dosyası (*.3mf)|*.3mf|Tüm Dosyalar (*.*)|*.*";
                sfd.DefaultExt = "3mf";
                sfd.AddExtension = true;
                string fallbackExeDir = Path.GetDirectoryName(Application.ExecutablePath);
                if (string.IsNullOrEmpty(fallbackExeDir)) fallbackExeDir = AppDomain.CurrentDomain.BaseDirectory;
                sfd.InitialDirectory = Directory.Exists(baseDir) ? baseDir : fallbackExeDir;
                sfd.FileName = defaultFileName;

                if (sfd.ShowDialog(this) != DialogResult.OK)
                {
                    return; // Kullanıcı iptal etti, hiçbir dosya oluşturulmaz veya ezilmez
                }
                outPath = sfd.FileName;
            }

            string chosenSafeName = Path.GetFileNameWithoutExtension(outPath);
            string chosenSpeedProfile = (rbSpeedSlow != null && rbSpeedSlow.Checked) ? "slow" : "fast";

            btnGenerate.Enabled = false;
            btnGenerate.Text = "Oluşturuluyor...";
            txtLog.Clear();

            if (currentMode == GeneratorMode.SiraliKlips)
            {
                int sNum = (int)numSeqStart.Value;
                int eNum = (int)numSeqEnd.Value;
                if (eNum < sNum) eNum = sNum;
                bool incGen = chkIncludeGender.Checked;
                string fontSeq = cmbSeqFont != null && cmbSeqFont.SelectedItem != null ? cmbSeqFont.SelectedItem.ToString() : "Trebuchet MS";
                double seqH = (double)numSeqHeight.Value;
                if (seqH > 9.00) seqH = 9.00;
                double seqStr = (double)numSeqStretch.Value;
                double seqG = (double)numSeqGap.Value;
                double seqW = (double)numSeqWidth.Value;
                double seqLH = GetSelectedSeqLayerHeight();

                System.Threading.ThreadPool.QueueUserWorkItem(_ =>
                {
                    bool success = ExecuteSequentialGeneration(bakPath, outPath, sNum, eNum, incGen, seqH, emboss, embed,
                        fontSeq, seqG, seqW, AppendLog, material, color1Hex, color2Hex, color3Hex, seqStr, chosenSpeedProfile, seqLH);

                    if (success)
                    {
                        lastGeneratedPath = outPath;
                        SaveLastSettings();
                    }

                    this.Invoke(new Action(() =>
                    {
                        btnGenerate.Enabled = true;
                        btnGenerate.Text = "▶  SIRALI KLİPS 3MF (ÇİFT TABLA) OLUŞTUR";

                        if (success)
                        {
                            MessageBox.Show(string.Format("'{0}' multi-plate projesi başarıyla üretildi!\n\nTabla 1: Düz Siyah Gövde / Beyaz Yazı\nTabla 2: Çift Renk Gövde / Siyah Yazı\nAyar Raporu: {1}_bilgi.txt\nKonum: {2}", 
                                Path.GetFileName(outPath), chosenSafeName, outPath),
                                "İşlem Tamamlandı", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        }
                        else
                        {
                            MessageBox.Show("Üretim sırasında bir hata meydana geldi. Detayları işlem günlüğünden inceleyebilirsiniz.",
                                "Hata", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }
                    }));
                });
            }
            else
            {
                string topText = txtTopText.Text.Trim();
                string botText = txtBotText.Text.Trim();

                bool useSoftG = (chkSoftG != null && chkSoftG.Checked);
                topText = ProcessTextWithSoftG(topText, useSoftG);
                botText = ProcessTextWithSoftG(botText, useSoftG);

                string fontTop = cmbFontTop.SelectedItem != null ? cmbFontTop.SelectedItem.ToString() : "Trebuchet MS";
                string fontBot = cmbFontBot.SelectedItem != null ? cmbFontBot.SelectedItem.ToString() : "Trebuchet MS";
                double textHTop = (double)numTextHeightTop.Value;
                double textHBot = (double)numTextHeightBot.Value;
                double gapTop = (double)numCharGapTop.Value;
                double gapBot = (double)numCharGapBot.Value;
                double scaleTop = (double)numWidthScaleTop.Value;
                double scaleBot = (double)numWidthScaleBot.Value;
                double stretchTop = numStretchTop != null ? (double)numStretchTop.Value : 100.0;
                double stretchBot = numStretchBot != null ? (double)numStretchBot.Value : 100.0;

                System.Threading.ThreadPool.QueueUserWorkItem(_ =>
                {
                    bool success = ExecuteGeneration(topText, botText, bakPath, outPath, emboss, embed, fontTop, fontBot, 
                        textHTop, textHBot, gapTop, gapBot, scaleTop, scaleBot, AppendLog, material, color1Hex, color2Hex,
                        stretchTop, stretchBot, isMarka, chosenSpeedProfile, useSoftG);

                    if (success)
                    {
                        lastGeneratedPath = outPath;
                        SaveLastSettings();
                    }

                    this.Invoke(new Action(() =>
                    {
                        btnGenerate.Enabled = true;
                        btnGenerate.Text = isMarka ? "▶  MARKA TİPİ 3MF OLUŞTUR" : "▶  3MF DOSYASINI OLUŞTUR";

                        if (success)
                        {
                            MessageBox.Show(string.Format("'{0}' dosyası başarıyla üretildi!\n\nAyar Raporu: {1}_bilgi.txt\nKonum: {2}\n\nOrcaSlicer veya dilimleyicinizde doğrudan açabilirsiniz.", 
                                Path.GetFileName(outPath), chosenSafeName, outPath),
                                "İşlem Tamamlandı", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        }
                        else
                        {
                            MessageBox.Show("Üretim sırasında bir hata meydana geldi. Detayları işlem günlüğünden inceleyebilirsiniz.",
                                "Hata", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }
                    }));
                });
            }
        }

        private void BtnLoad3mf_Click(object sender, EventArgs e)
        {
            using (OpenFileDialog ofd = new OpenFileDialog())
            {
                ofd.Title = "Ayarlarını Yüklemek İstediğiniz .3mf veya _bilgi.txt Dosyasını Seçin";
                ofd.Filter = "Güvercin Halkası Dosyaları (*.3mf;*_bilgi.txt;*.json)|*.3mf;*_bilgi.txt;*.json|Tüm Dosyalar (*.*)|*.*";
                ofd.InitialDirectory = AppDomain.CurrentDomain.BaseDirectory;
                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    ApplySettingsFromFile(ofd.FileName);
                }
            }
        }

        private void BtnViewInfo_Click(object sender, EventArgs e)
        {
            string targetTxt = "";
            if (!string.IsNullOrEmpty(lastGeneratedPath))
            {
                string baseDir = Path.GetDirectoryName(lastGeneratedPath);
                string baseName = Path.GetFileNameWithoutExtension(lastGeneratedPath);
                string cand = Path.Combine(baseDir, baseName + "_bilgi.txt");
                if (File.Exists(cand)) targetTxt = cand;
            }

            if (string.IsNullOrEmpty(targetTxt))
            {
                string topText = txtTopText.Text.Trim();
                string safe = MakeSafeFileName(topText);
                string cand = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, safe + "_bilgi.txt");
                if (File.Exists(cand)) targetTxt = cand;
            }

            if (!string.IsNullOrEmpty(targetTxt) && File.Exists(targetTxt))
            {
                try
                {
                    System.Diagnostics.Process.Start("notepad.exe", "\"" + targetTxt + "\"");
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Rapor dosyası açılamadı: " + ex.Message, "Hata", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
            }
            else
            {
                MessageBox.Show("Açılacak bir ayar bilgi raporu (_bilgi.txt) bulunamadı. Önce bir halka dosyası oluşturunuz veya yükleyiniz.", "Bilgi", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
        }

        private void BtnAddCustomFont_Click(object sender, EventArgs e)
        {
            using (OpenFileDialog ofd = new OpenFileDialog())
            {
                ofd.Title = "Uygulamaya Gömülecek Font Dosyasını Seçin (.ttf veya .otf)";
                ofd.Filter = "Yazı Tipi Dosyaları (*.ttf;*.otf)|*.ttf;*.otf|TrueType Font (*.ttf)|*.ttf|OpenType Font (*.otf)|*.otf|Tüm Dosyalar (*.*)|*.*";
                ofd.Multiselect = true;

                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    List<string> addedNames = new List<string>();
                    foreach (string sourceFile in ofd.FileNames)
                    {
                        try
                        {
                            string fileName = Path.GetFileName(sourceFile);
                            string destFile = Path.Combine(FontManager.FontsDirectory, fileName);
                            if (!string.Equals(Path.GetFullPath(sourceFile), Path.GetFullPath(destFile), StringComparison.OrdinalIgnoreCase))
                            {
                                File.Copy(sourceFile, destFile, true);
                            }

                            List<FontFamily> loaded = FontManager.LoadFontFile(destFile);
                            foreach (var lf in loaded)
                            {
                                if (!addedNames.Contains(lf.Name))
                                {
                                    addedNames.Add(lf.Name);
                                    AppendLog(string.Format("Özel font başarıyla yüklendi: '{0}' ({1})", lf.Name, fileName));
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show("Font yüklenirken hata: " + ex.Message, "Hata", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        }
                    }

                    if (addedNames.Count > 0)
                    {
                        string lastAdded = addedNames[addedNames.Count - 1];
                        RefreshFontLists(lastAdded, null);

                        MessageBox.Show(string.Format("Seçilen fontlar başarıyla uygulamaya gömüldü ve 'Fonts' klasörüne kaydedildi:\n\n{0}\n\nBu klasörü hangi bilgisayara taşırsanız taşıyın, fontlarınız otomatik olarak çalışacaktır.",
                            string.Join("\n", addedNames.ToArray())),
                            "Fontlar Başarıyla Yüklendi", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    }
                    else
                    {
                        RefreshFontLists();
                    }
                }
            }
        }

        private void BtnOpenFontsFolder_Click(object sender, EventArgs e)
        {
            try
            {
                string dir = FontManager.FontsDirectory;
                System.Diagnostics.Process.Start("explorer.exe", dir);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Font klasörü açılamadı: " + ex.Message, "Hata", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private void ApplySettingsFromFile(string path)
        {
            if (path.EndsWith(".3mf", StringComparison.OrdinalIgnoreCase) && File.Exists(path))
            {
                try
                {
                    using (ZipArchive zip = ZipFile.OpenRead(path))
                    {
                        bool anyExtracted = false;
                        foreach (var entry in zip.Entries)
                        {
                            if (entry.FullName.StartsWith("Metadata/Fonts/", StringComparison.OrdinalIgnoreCase) &&
                                (entry.Name.EndsWith(".ttf", StringComparison.OrdinalIgnoreCase) || entry.Name.EndsWith(".otf", StringComparison.OrdinalIgnoreCase)))
                            {
                                string dest = Path.Combine(FontManager.FontsDirectory, entry.Name);
                                if (!File.Exists(dest))
                                {
                                    entry.ExtractToFile(dest, true);
                                    anyExtracted = true;
                                }
                                FontManager.LoadFontFile(dest);
                            }
                        }
                        if (anyExtracted)
                        {
                            RefreshFontLists();
                            AppendLog("3MF içindeki gömülü özel fontlar yerel 'Fonts' klasörüne aktarıldı.");
                        }
                    }
                }
                catch { }
            }

            RingConfig cfg = RingConfig.LoadFrom3mf(path);
            if (cfg == null)
            {
                MessageBox.Show("Seçilen dosyadan ayar verisi okunamadı:\n" + path, "Uyarı", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            ApplyConfigToUI(cfg);
            if (path.EndsWith(".3mf", StringComparison.OrdinalIgnoreCase))
            {
                lastGeneratedPath = path;
            }
            AppendLog(string.Format("Ayarlar '{0}' dosyasından başarıyla yüklendi.", Path.GetFileName(path)));
            MessageBox.Show(string.Format("'{0}' dosyasının ayarları başarıyla yüklendi!\n\nMod: {1}\nÜst/İsim: {2}\nAlt/Tel: {3}\nMalzeme: {4}",
                Path.GetFileName(path), cfg.Mode, cfg.TopText, cfg.BotText, cfg.Material),
                "Ayarlar Yüklendi", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }

        private void ApplyConfigToUI(RingConfig cfg)
        {
            if (cfg == null) return;
            SetMode(cfg.Mode);

            if (!string.IsNullOrEmpty(cfg.TopText)) txtTopText.Text = cfg.TopText;
            if (!string.IsNullOrEmpty(cfg.BotText)) txtBotText.Text = cfg.BotText;

            // Üst Yazı Ayarları
            if (cfg.TextHeightTopMm >= (double)numTextHeightTop.Minimum && cfg.TextHeightTopMm <= (double)numTextHeightTop.Maximum)
                numTextHeightTop.Value = (decimal)cfg.TextHeightTopMm;
            if (numStretchTop != null && cfg.StretchTop >= (double)numStretchTop.Minimum && cfg.StretchTop <= (double)numStretchTop.Maximum)
                numStretchTop.Value = (decimal)cfg.StretchTop;
            if (cfg.CharGapTopMm >= (double)numCharGapTop.Minimum && cfg.CharGapTopMm <= (double)numCharGapTop.Maximum)
                numCharGapTop.Value = (decimal)cfg.CharGapTopMm;
            if (cfg.WidthScaleTop >= (double)numWidthScaleTop.Minimum && cfg.WidthScaleTop <= (double)numWidthScaleTop.Maximum)
                numWidthScaleTop.Value = (decimal)cfg.WidthScaleTop;

            // Alt Yazı Ayarları
            if (cfg.TextHeightBotMm >= (double)numTextHeightBot.Minimum && cfg.TextHeightBotMm <= (double)numTextHeightBot.Maximum)
                numTextHeightBot.Value = (decimal)cfg.TextHeightBotMm;
            if (numStretchBot != null && cfg.StretchBot >= (double)numStretchBot.Minimum && cfg.StretchBot <= (double)numStretchBot.Maximum)
                numStretchBot.Value = (decimal)cfg.StretchBot;
            if (cfg.CharGapBotMm >= (double)numCharGapBot.Minimum && cfg.CharGapBotMm <= (double)numCharGapBot.Maximum)
                numCharGapBot.Value = (decimal)cfg.CharGapBotMm;
            if (cfg.WidthScaleBot >= (double)numWidthScaleBot.Minimum && cfg.WidthScaleBot <= (double)numWidthScaleBot.Maximum)
                numWidthScaleBot.Value = (decimal)cfg.WidthScaleBot;

            // Sıralı Ayarlar
            if (cfg.StartNum >= (int)numSeqStart.Minimum && cfg.StartNum <= (int)numSeqStart.Maximum)
                numSeqStart.Value = cfg.StartNum;
            if (cfg.EndNum >= (int)numSeqEnd.Minimum && cfg.EndNum <= (int)numSeqEnd.Maximum)
                numSeqEnd.Value = cfg.EndNum;
            chkIncludeGender.Checked = cfg.IncludeGender;
            if (cfg.SeqTextHeightMm >= (double)numSeqHeight.Minimum && cfg.SeqTextHeightMm <= (double)numSeqHeight.Maximum)
                numSeqHeight.Value = (decimal)cfg.SeqTextHeightMm;
            if (cfg.SeqStretch >= (double)numSeqStretch.Minimum && cfg.SeqStretch <= (double)numSeqStretch.Maximum)
                numSeqStretch.Value = (decimal)cfg.SeqStretch;
            if (cfg.SeqCharGapMm >= (double)numSeqGap.Minimum && cfg.SeqCharGapMm <= (double)numSeqGap.Maximum)
                numSeqGap.Value = (decimal)cfg.SeqCharGapMm;
            if (cfg.SeqWidthScale >= (double)numSeqWidth.Minimum && cfg.SeqWidthScale <= (double)numSeqWidth.Maximum)
                numSeqWidth.Value = (decimal)cfg.SeqWidthScale;
            SetSelectedSeqLayerHeight(cfg.SeqLayerHeight);

            // Kabartma & Gömülme
            if (chkSoftG != null)
                chkSoftG.Checked = cfg.UseSoftG;
            if (cfg.EmbossMm >= (double)numEmboss.Minimum && cfg.EmbossMm <= (double)numEmboss.Maximum)
                numEmboss.Value = (decimal)cfg.EmbossMm;
            if (cfg.EmbedMm >= (double)numEmbed.Minimum && cfg.EmbedMm <= (double)numEmbed.Maximum)
                numEmbed.Value = (decimal)cfg.EmbedMm;

            if (!string.IsNullOrEmpty(cfg.Material))
            {
                int idx = cmbMaterial.FindStringExact(cfg.Material);
                if (idx >= 0) cmbMaterial.SelectedIndex = idx;
            }

            if (rbSpeedSlow != null && rbSpeedFast != null)
            {
                if (string.Equals(cfg.SpeedProfile, "slow", StringComparison.OrdinalIgnoreCase))
                    rbSpeedSlow.Checked = true;
                else
                    rbSpeedFast.Checked = true;
            }

            SelectFontInCombo(cmbFontTop, cfg.FontTop);
            SelectFontInCombo(cmbFontBot, cfg.FontBot);
            SelectFontInCombo(cmbSeqFont, cfg.SeqFont);

            if (!string.IsNullOrEmpty(cfg.Color1Hex) && !string.IsNullOrEmpty(cfg.Color2Hex))
            {
                try
                {
                    Color c1 = ColorTranslator.FromHtml(cfg.Color1Hex);
                    Color c2 = ColorTranslator.FromHtml(cfg.Color2Hex);
                    SetColors(c1, c2, true);
                }
                catch { }
            }

            UpdateSeqTotal();
            UpdateFontPreview();
        }

        private void SelectFontInCombo(ComboBox cmb, string fontName)
        {
            if (cmb == null || string.IsNullOrEmpty(fontName)) return;
            string clean = FontManager.CleanFontName(fontName);
            int selIdx = -1;
            for (int i = 0; i < cmb.Items.Count; i++)
            {
                string itm = cmb.Items[i].ToString();
                if (itm.Equals(fontName, StringComparison.OrdinalIgnoreCase) ||
                    FontManager.CleanFontName(itm).Equals(clean, StringComparison.OrdinalIgnoreCase))
                {
                    selIdx = i;
                    break;
                }
            }
            if (selIdx >= 0) cmb.SelectedIndex = selIdx;
            else
            {
                cmb.Items.Add(fontName);
                cmb.SelectedItem = fontName;
            }
        }

        private RingConfig GetCurrentConfig()
        {
            RingConfig cfg = new RingConfig();
            cfg.Mode = currentMode;
            cfg.TopText = txtTopText.Text.Trim();
            cfg.BotText = txtBotText.Text.Trim();
            cfg.FontTop = cmbFontTop.SelectedItem != null ? cmbFontTop.SelectedItem.ToString() : "Trebuchet MS";
            cfg.FontBot = cmbFontBot.SelectedItem != null ? cmbFontBot.SelectedItem.ToString() : "Trebuchet MS";
            cfg.TextHeightTopMm = (double)numTextHeightTop.Value;
            cfg.StretchTop = numStretchTop != null ? (double)numStretchTop.Value : 100.0;
            cfg.CharGapTopMm = (double)numCharGapTop.Value;
            cfg.WidthScaleTop = (double)numWidthScaleTop.Value;
            cfg.TextHeightBotMm = (double)numTextHeightBot.Value;
            cfg.StretchBot = numStretchBot != null ? (double)numStretchBot.Value : 100.0;
            cfg.CharGapBotMm = (double)numCharGapBot.Value;
            cfg.WidthScaleBot = (double)numWidthScaleBot.Value;

            cfg.StartNum = (int)numSeqStart.Value;
            cfg.EndNum = (int)numSeqEnd.Value;
            cfg.IncludeGender = chkIncludeGender.Checked;
            cfg.SeqFont = cmbSeqFont != null && cmbSeqFont.SelectedItem != null ? cmbSeqFont.SelectedItem.ToString() : "Trebuchet MS";
            cfg.SeqTextHeightMm = (double)numSeqHeight.Value;
            cfg.SeqStretch = (double)numSeqStretch.Value;
            cfg.SeqCharGapMm = (double)numSeqGap.Value;
            cfg.SeqWidthScale = (double)numSeqWidth.Value;
            cfg.SeqLayerHeight = GetSelectedSeqLayerHeight();

            cfg.EmbossMm = (double)numEmboss.Value;
            cfg.EmbedMm = (double)numEmbed.Value;
            cfg.UseSoftG = (chkSoftG != null && chkSoftG.Checked);
            cfg.Material = cmbMaterial.SelectedItem != null ? cmbMaterial.SelectedItem.ToString() : "PLA";
            cfg.SpeedProfile = (rbSpeedSlow != null && rbSpeedSlow.Checked) ? "slow" : "fast";
            cfg.Color1Hex = ColorToHex(currentColor1);
            cfg.Color2Hex = ColorToHex(currentColor2);
            cfg.CreatedDate = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
            return cfg;
        }

        private double GetSelectedSeqLayerHeight()
        {
            if (cmbSeqLayerHeight == null || cmbSeqLayerHeight.SelectedItem == null) return 0.12;
            string s = cmbSeqLayerHeight.SelectedItem.ToString();
            if (s.StartsWith("0.16")) return 0.16;
            if (s.StartsWith("0.20")) return 0.20;
            return 0.12;
        }

        private void SetSelectedSeqLayerHeight(double lh)
        {
            if (cmbSeqLayerHeight == null || cmbSeqLayerHeight.Items.Count < 3) return;
            if (lh >= 0.18) cmbSeqLayerHeight.SelectedIndex = 2;
            else if (lh >= 0.14) cmbSeqLayerHeight.SelectedIndex = 1;
            else cmbSeqLayerHeight.SelectedIndex = 0;
        }

        private void SaveLastSettings()
        {
            try
            {
                string path = Path.Combine(GetAppDataDir(), "last_settings.json");
                RingConfig cfg = GetCurrentConfig();
                File.WriteAllText(path, cfg.ToJson(), Encoding.UTF8);
            }
            catch { }
        }

        private void LoadLastSettings()
        {
            try
            {
                string path = Path.Combine(GetAppDataDir(), "last_settings.json");
                if (File.Exists(path))
                {
                    string json = File.ReadAllText(path, Encoding.UTF8);
                    RingConfig cfg = RingConfig.FromJson(json);
                    ApplyConfigToUI(cfg);
                }
            }
            catch { }
        }
    }

    [System.Runtime.InteropServices.DllImport("shcore.dll")]
    private static extern int SetProcessDpiAwareness(int value);

    // ==========================================
    // 9. WINDOWS FORMS APPLICATION ENTRY POINT
    // ==========================================
    [STAThread]
    public static void Main(string[] args)
    {
        try { SetProcessDpiAwareness(0); } catch { }
        if (args != null && args.Length > 0 && args[0] == "--cli")
        {
            string top = "BEKİR ATLI";
            string bot = "0538 368 47 43";
            string cliExeDir = Path.GetDirectoryName(Application.ExecutablePath);
            if (string.IsNullOrEmpty(cliExeDir)) cliExeDir = AppDomain.CurrentDomain.BaseDirectory;
            string bak = GetDefaultBakPath("BOS_KLIPS_9MM.3mf.bak");
            string outP = Path.Combine(cliExeDir, "BEKİR ATLI Klips.3mf");
            double emboss = 0.20;
            double embed = 0.40;
            string fontTop = "Bahnschrift";
            string fontBot = "Bahnschrift";
            double textHTop = 3.50;
            double textHBot = 3.50;
            double gapTop = 0.0;
            double gapBot = 0.0;
            double scaleTop = 0.0;
            double scaleBot = 0.0;
            string material = "PLA";
            string c1 = "#1A4FD8";
            string c2 = "#FFFFFF";
            double stretchTop = 100.0;
            double stretchBot = 100.0;
            bool isMarka = false;
            string speed = "slow";
            bool useSoftG = true;

            bool isSeq = false;
            int seqStart = 1, seqEnd = 10;
            bool seqGender = true;
            double seqH = 7.00;
            double seqLayer = 0.12;
            for (int i = 1; i < args.Length; i++)
            {
                if (args[i] == "--top" && i + 1 < args.Length) top = args[++i];
                else if (args[i] == "--bot" && i + 1 < args.Length) bot = args[++i];
                else if (args[i] == "--bak" && i + 1 < args.Length) bak = args[++i];
                else if (args[i] == "--out" && i + 1 < args.Length) outP = args[++i];
                else if (args[i] == "--font" && i + 1 < args.Length) { fontTop = args[++i]; fontBot = fontTop; }
                else if (args[i] == "--emboss" && i + 1 < args.Length) emboss = double.Parse(args[++i], CultureInfo.InvariantCulture);
                else if (args[i] == "--mat" && i + 1 < args.Length) material = args[++i];
                else if (args[i] == "--layer" && i + 1 < args.Length) seqLayer = double.Parse(args[++i], CultureInfo.InvariantCulture);
                else if (args[i] == "--marka") isMarka = true;
                else if (args[i] == "--seq") isSeq = true;
                else if (args[i] == "--end" && i + 1 < args.Length) seqEnd = int.Parse(args[++i], CultureInfo.InvariantCulture);
            }

            bool ok;
            if (isSeq)
            {
                Console.WriteLine("[CLI] Sıralı Başlatılıyor: " + seqStart + ".." + seqEnd);
                ok = ExecuteSequentialGeneration(bak, outP, seqStart, seqEnd, seqGender, seqH, emboss, embed,
                    fontTop, gapTop, scaleTop, s => Console.WriteLine(s), material, c1, c2, "#FFFFFF", 100.0, speed, seqLayer);
            }
            else
            {
                Console.WriteLine("[CLI] Başlatılıyor: " + top + " / " + bot);
                ok = ExecuteGeneration(top, bot, bak, outP, emboss, embed, fontTop, fontBot,
                    textHTop, textHBot, gapTop, gapBot, scaleTop, scaleBot, s => Console.WriteLine(s),
                    material, c1, c2, stretchTop, stretchBot, isMarka, speed, useSoftG);
            }
            Console.WriteLine("[CLI] Sonuç: " + (ok ? "BAŞARILI" : "BAŞARISIZ"));
            return;
        }

        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new MainForm());
    }
}

