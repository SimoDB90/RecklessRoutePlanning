using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    public class PathfindingAStar
    {
        private double maxAcceleration;
        private List<Sphere> interdictionSpheres;
        private Vector3D targetPosition;

        public PathfindingAStar(double maxAcceleration, List<Sphere> interdictionSpheres, Vector3D targetPosition)
        {
            this.maxAcceleration = maxAcceleration;
            this.interdictionSpheres = interdictionSpheres;
            this.targetPosition = targetPosition;
        }

        public List<Vector3D> CalculateOptimalPath(Vector3D start)
        {
            List<Vector3D> openList = new List<Vector3D>(); //nodes to explore, aka neighbors of nodes
            HashSet<Vector3D> closedList = new HashSet<Vector3D>(); //already sarched nodes

            //list of best nodes. Indeed, the cameFrom node is the previous best node of the new best neighbors chosen
            Dictionary<Vector3D, Vector3D> cameFrom = new Dictionary<Vector3D, Vector3D>();
            Dictionary<Vector3D, double> gScore = new Dictionary<Vector3D, double>(); //score from start to current node
            Dictionary<Vector3D, double> fScore = new Dictionary<Vector3D, double>(); //total score
                                                                                      //bounding condition from starting node
            openList.Add(start);
            gScore[start] = 0;
            fScore[start] = Heuristic(start);

            while (openList.Count > 0)
            {
                //order the list: iterate through all nodes, and check the total score. If has a value, point it.
                //otherwise, put a very high value (list is ordered from lowerst to highest). Than takes first.
                //this is the best node for creating the path.
                Vector3D current = openList.OrderBy(node => fScore.ContainsKey(node) ? fScore[node] : double.MaxValue).First();
                openList.Remove(current);
                //if we reach the last node, we're good. And we reconstruct the whole path
                if ((current - targetPosition).Length() < 1.0)
                {
                    return ReconstructPath(cameFrom, current);
                }
                //add the node to the close list
                closedList.Add(current);
                //explore neighbors of current node
                foreach (Vector3D neighbor in GetNeighbors(current))
                {
                    //if is located in interdiction sphere or is present in close list, than, skip it.
                    if (IsInInterdictionZone(neighbor)) continue;
                    if (closedList.Contains(neighbor)) continue;

                    double tentativeGScore = gScore[current] + (neighbor - current).Length();
                    //add new point to open list, so, next iteration it will be the start for neighbors
                    if (!openList.Contains(neighbor))
                        openList.Add(neighbor);
                    //save the node if is the best
                    if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                    {
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = tentativeGScore;
                        fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor);
                    }
                }
            }

            return new List<Vector3D>(); // No path found
        }

        private List<Vector3D> ReconstructPath(Dictionary<Vector3D, Vector3D> cameFrom, Vector3D current)
        {
            List<Vector3D> path = new List<Vector3D>();
            while (cameFrom.ContainsKey(current))
            {
                path.Add(current);
                current = cameFrom[current];
            }
            path.Reverse();
            return path;
        }

        private List<Vector3D> GetNeighbors(Vector3D current)
        {
            List<Vector3D> neighbors = new List<Vector3D>();
            //iteration in every direction. A neighbor is 10km
            for (int i = -10000; i <= 10000; i+=10000)
            {
                for (int j = -10000; j <= 10000; j+= 10000)
                {
                    for (int k = -10000; k <= 10000; k+= 10000)
                    {
                        if (i == 0 && j == 0 && k == 0) continue;
                        Vector3D neighbor = current + new Vector3D(i, j, k) * maxAcceleration;
                        neighbors.Add(neighbor);
                    }
                }
            }

            return neighbors;
        }

        private bool IsInInterdictionZone(Vector3D position)
        {
            foreach (Sphere sphere in interdictionSpheres)
            {
                if ((position - sphere.Center).Length() < sphere.Radius)
                    return true;
            }
            return false;
        }

        public double Heuristic(Vector3D position)
        {
            return (targetPosition - position).Length();
        }
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
}
