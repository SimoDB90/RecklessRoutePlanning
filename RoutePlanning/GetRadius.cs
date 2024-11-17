using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    internal class GetRadius
    {
        public double GetPerimeter(Vector3D P0, Vector3D P1, Vector3D P2, out double a,
            out double b, out double c)
        {
            a = Vector3D.Distance(P0, P1);
            b = Vector3D.Distance(P1, P2);
            c = Vector3D.Distance(P2, P0);
            return (a + b + c) / 2;
        }
        public double GetArea(Vector3D P0, Vector3D P1, Vector3D P2, out double a,
            out double b, out double c)
        {
            a = 0;
            b = 0;
            c = 0;
            var p = GetPerimeter(P0, P1, P2, out a, out b, out c);
            return Math.Sqrt(p * (p - a) * (p - b) * (p - c));
        }
        public double GetRadius(Vector3D P0, Vector3D P1, Vector3D P2)
        {
            double a = 0;
            double b = 0;
            double c = 0;
            var area = GetArea(P0, P1, P2, out a, out b, out c);
            if (area < 0) { return 0; }
            return (a * b * c) / (4 * area);
        }
    }
}
