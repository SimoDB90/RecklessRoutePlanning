using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    public class BezierOptimizer
    {
        public Vector3D P0;  // Starting point of the curve
        public Vector3D P2;  // End point of the curve
        public Vector3D P1;  // Control point of the curve (initially set as the midpoint between P0 and P2)

        public double InitialVelocity;  // Initial velocity in meters per second
        public double ConstantAcceleration = 0.1;  // Constant acceleration along the trajectory
        public double MaxAcceleration;  // Maximum allowable acceleration in meters per second squared

        public List<Sphere> Spheres;

        // Constructor for the optimizer
        public BezierOptimizer(double initialVelocity, double maxAcceleration, List<Sphere> spheres,
            Vector3D start, Vector3D end)
        {
            InitialVelocity = initialVelocity;
            MaxAcceleration = maxAcceleration;
            P0 = start;
            P2 = end;
            Spheres = spheres;

            // Initialize P1 as the midpoint between P0 and P2
            P1 = (P0 + P2) / 2;
        }

        // Function to optimize the trajectory
        public Vector3D Optimize()
        {
            double tolerance = 0.01;  // Convergence tolerance
            double learningRate = 0.1;  // Initial learning rate
            int iterations = 0;
            int maxIterations = 1000; //threshold to stop iterating
            double initialLearningRate = learningRate;  // Store the initial learning rate
            double cost = CalculateCost(P1);  // Calculate the initial cost
            double lastCost;  // Last calculated cost

            // Continue optimizing while the cost is greater than the tolerance
            while (cost > tolerance && iterations <= maxIterations)
            {
                // Calculate the gradient
                Vector3D gradient = CalculateGradient(P1);

                // Update P1 using the gradient and the learning rate
                P1 -= learningRate * gradient;

                // Calculate the new cost
                lastCost = cost;
                cost = CalculateCost(P1);

                // If the cost improves, increase the learning rate (but not too much)
                if (cost < lastCost)
                {
                    learningRate *= 1.2;  // Increase the learning rate to accelerate convergence
                }
                // If the cost does not improve, decrease the learning rate to fine-tune
                else
                {
                    learningRate *= 0.5;  // Reduce the learning rate to make smaller adjustments
                }

                // Limit the learning rate to avoid extreme updates
                learningRate = Math.Min(learningRate, initialLearningRate * 10);
                // Exit if the gradient is very small (close to convergence)
                if (gradient.Length() < 1e-5)
                    break;
                iterations++;
            }
            return P1;
        }

        // Calculate the cost of the trajectory with adaptive sampling and the collision constraint with spheres
        private double CalculateCost(Vector3D P1)
        {
            double cost = 0;
            int numSamples = 100; // Max number of samples
            double gamma = 0.5; // Linear penalty factor for acceleration
            double penaltyFactor = 1000.0; // Penalty factor for collisions
            double tStep = 1.0 / numSamples; // Step size for sampling

            // Array to store curvatures at different time points
            double[] curvatures = new double[numSamples];

            // Sample the trajectory at 100 points (from t = 0 to t = 1)
            for (int i = 0; i < numSamples; i++)
            {
                double t = i * tStep;

                // Calculate the curvature at point t
                curvatures[i] = CalculateCurvature(t, P1);
            }

            // Now calculate the cost with more samples in high-curvature regions
            for (int i = 0; i < numSamples; i++)
            {
                double t = i * tStep;
                double curvature = curvatures[i];

                // Adjust the number of samples based on curvature (more samples where curvature is high)
                int additionalSamples = (int)(curvature * 10); // Scale factor for more samples in high-curvature areas
                additionalSamples = Math.Min(additionalSamples, 5); // Limit the max number of additional samples to prevent too many points

                // Recalculate the cost with higher density of points where needed
                for (int j = 0; j <= additionalSamples; j++)
                {
                    double subT = t + j * tStep / (additionalSamples + 1);  // Divide the segment into smaller sub-segments based on curvature
                    double velocity = InitialVelocity + ConstantAcceleration * subT;
                    double acceleration = velocity * velocity * curvature;

                    // Penalize if the acceleration exceeds the maximum acceleration (linear penalty)
                    if (acceleration > MaxAcceleration)
                    {
                        cost += gamma * (acceleration - MaxAcceleration); // Linear penalty
                    }

                    // Check if the point is inside any of the spheres and apply a penalty if it is
                    Vector3D pointOnCurve = CalculatePointOnCurve(subT, P1);
                    foreach (var sphere in Spheres)
                    {
                        double distance = Vector3D.Distance(pointOnCurve, sphere.Center);
                        if (distance < sphere.Radius)
                        {
                            cost += penaltyFactor * (sphere.Radius - distance);  // Penalize for being inside the sphere
                        }
                    }
                }
            }

            return cost;
        }

        // Calculate the gradient of the cost function
        private Vector3D CalculateGradient(Vector3D P1)
        {
            double delta = 0.01;  // Small variation to calculate the gradient

            // Calculate the cost with a small variation in X, Y, and Z
            Vector3D originalP1 = P1;

            // Increment X and calculate the cost change
            P1.X += delta;
            double costX = CalculateCost(P1);

            // Restore P1 and increment Y to calculate the variation on the Y side
            P1 = originalP1;
            P1.Y += delta;
            double costY = CalculateCost(P1);

            // Restore P1 and increment Z to calculate the variation on the Z side
            P1 = originalP1;
            P1.Z += delta;
            double costZ = CalculateCost(P1);

            // Restore P1
            P1 = originalP1;

            // Return the gradient as the change in cost with respect to X, Y, and Z
            return new Vector3D((costX - CalculateCost(P1)) / delta,
                                (costY - CalculateCost(P1)) / delta,
                                (costZ - CalculateCost(P1)) / delta);
        }

        // Calculate the curvature of the trajectory at time t
        private double CalculateCurvature(double t, Vector3D P1)
        {
            // First derivative
            Vector3D d1 = FirstDerivative(t, P1);
            // Second derivative
            Vector3D d2 = SecondDerivative(t);

            // Calculate the curvature using the formula
            double numerator = Math.Abs(d1.X * d2.Y - d1.Y * d2.X); // Cross product in 2D
            double denominator = Math.Pow(d1.X * d1.X + d1.Y * d1.Y, 1.5);

            return numerator / denominator;
        }

        // Calculate the first derivative of the curve (velocity)
        private Vector3D FirstDerivative(double t, Vector3D P1)
        {
            double x = 2 * (1 - t) * (P1.X - P0.X) + 2 * t * (P2.X - P1.X);
            double y = 2 * (1 - t) * (P1.Y - P0.Y) + 2 * t * (P2.Y - P1.Y);
            double z = 2 * (1 - t) * (P1.Z - P0.Z) + 2 * t * (P2.Z - P1.Z);
            return new Vector3D(x, y, z);
        }

        // Calculate the second derivative of the curve (acceleration)
        private Vector3D SecondDerivative(double t)
        {
            double x = 2 * (P2.X - 2 * P1.X + P0.X);
            double y = 2 * (P2.Y - 2 * P1.Y + P0.Y);
            double z = 2 * (P2.Z - 2 * P1.Z + P0.Z);
            return new Vector3D(x, y, z);
        }

        // Calculate a point on the curve at a given time t
        private Vector3D CalculatePointOnCurve(double t, Vector3D P1)
        {
            double x = (1 - t) * (1 - t) * P0.X + 2 * (1 - t) * t * P1.X + t * t * P2.X;
            double y = (1 - t) * (1 - t) * P0.Y + 2 * (1 - t) * t * P1.Y + t * t * P2.Y;
            double z = (1 - t) * (1 - t) * P0.Z + 2 * (1 - t) * t * P1.Z + t * t * P2.Z;
            return new Vector3D(x, y, z);
        }
        public class Sphere
        {
            public Vector3D Center { get; set; }
            public double Radius { get; set; }

            public Sphere(Vector3D center, double radius)
            {
                Center = center;
                Radius = radius;
            }
        }

        // Genera una lista di waypoint a partire dalla curva ottimizzata
        public List<Vector3D> GenerateWaypoints(int numWaypoints)
        {
            List<Vector3D> waypoints = new List<Vector3D>();

            double tStep = 1.0 / (numWaypoints - 1); // Calcolo del passo tra i punti

            for (int i = 0; i < numWaypoints; i++)
            {
                double t = i * tStep; // Valore di t per il punto corrente
                waypoints.Add(CalculatePointOnCurve(t, P1)); // Calcola il punto sulla curva
            }

            return waypoints;
        }

    }


}
