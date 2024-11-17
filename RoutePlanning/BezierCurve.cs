using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    public class BezierCurve
    {
        //method to validate curve
        public List<Vector3D> ValidatePath(List<Vector3D> oldPath, double acceleration, 
            Vector3D initialVelocity)
        {
            List<Vector3D> path = new List<Vector3D>();
            Vector3D P0 = oldPath[0];
            path.Add(P0);
            for (int i = 1; i<= oldPath.Count; i++)
            {
                Vector3D previousPoint = oldPath[i-1];
                Vector3D futurePoint = oldPath[i+1];
                Vector3D currentPoint = oldPath[i];
                var distanceFromStart = Vector3D.Distance(P0, futurePoint);
                var predictedSpeed = GetSpeed(initialVelocity, acceleration, distanceFromStart);

                Vector3D delta = futurePoint - currentPoint;
                if(CanAccelerate(futurePoint, currentPoint, predictedSpeed, acceleration))
                {
                    path.Add(oldPath[i]);
                }
                else
                {
                    Vector3D midPoint = FindNewMidPoint(currentPoint, futurePoint, acceleration, predictedSpeed);
                    path.Add(midPoint);
                }
                i++;
            }
            return path;
        }

        //velocity prediction
        private double GetSpeed(Vector3D initialVelocity, double acceleration, double distance)
        {
            //v^2 = sqrt(v0^2 + 2as)
            double initialSpeed = initialVelocity.Length();
            return Math.Sqrt(initialSpeed * initialSpeed + 2 * acceleration * distance);

        }
        // method to calculate a point onto the curve (cubic curve)
        public List<Vector3D> GenerateBezierPath(Vector3D start, Vector3D end, Vector3D control1, Vector3D control2, int resolution)
        {
            List<Vector3D> path = new List<Vector3D>();

            for (int i = 0; i <= resolution; i++)
            {
                double t = (double)i / resolution;

                // equation of Bézier curve
                Vector3D point =
                    Math.Pow(1 - t, 3) * start +
                    3 * Math.Pow(1 - t, 2) * t * control1 +
                    3 * (1 - t) * Math.Pow(t, 2) * control2 +
                    Math.Pow(t, 3) * end;

                path.Add(point);
            }

            return path;
        }
        private bool CanAccelerate(Vector3D futurePoint, Vector3D currentPoint, double currentVelocity,
            double maxAcceleration)
        {
            double distance = Vector3D.Distance(futurePoint, currentPoint);
            double acceleration = currentVelocity * currentVelocity / distance;
            if (acceleration <= maxAcceleration) return true;
            return false;
        }
        private Vector3D FindNewMidPoint(Vector3D currentPoint, Vector3D futurePoint,double maxAcceleration,
            double currentVelocity)
        {
            var direction = Vector3D.Normalize(futurePoint - currentVelocity);
            //find the new vector based on the max acceleration
            double maxDistance = currentVelocity * currentVelocity / ( 2 * maxAcceleration);
            return futurePoint - maxDistance * direction;
        }
        
    }
}
