using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using VRageMath;

namespace IngameScript
{
    partial class Program: MyGridProgram
    {
        public class SpaceNavigationAStar
        {
            private List<Vector3D> waypoints;
            private double maxAcceleration;
            private double maxVelocity;
            private List<Sphere> interdictionSpheres;
            private Vector3D targetPosition;
            private IMyShipController shipController;

            public SpaceNavigationAStar(double maxAcceleration, double maxVelocity, List<Sphere> interdictionSpheres, Vector3D targetPosition, IMyShipController shipController)
            {
                this.maxAcceleration = maxAcceleration;
                this.maxVelocity = maxVelocity;
                this.interdictionSpheres = interdictionSpheres;
                this.targetPosition = targetPosition;
                this.shipController = shipController;
                this.waypoints = new List<Vector3D>();
            }

            // Metodo per calcolare l'accelerazione necessaria per raggiungere un waypoint
            public Vector3D CalculateRequiredAcceleration(Vector3D waypoint)
            {
                // Ottieni la posizione attuale e la velocità attuale della nave
                Vector3D currentPosition = shipController.GetPosition();
                Vector3D currentVelocity = shipController.GetShipVelocities().LinearVelocity;

                // Calcola la direzione verso il waypoint
                Vector3D directionToWaypoint = waypoint - currentPosition;
                directionToWaypoint.Normalize();  // Normalizza il vettore per ottenere solo la direzione

                // Calcola la velocità desiderata (assumiamo velocità massima lungo la direzione)
                Vector3D desiredVelocity = directionToWaypoint * maxVelocity;

                // Calcola l'accelerazione necessaria per raggiungere la velocità desiderata
                Vector3D requiredAcceleration = (desiredVelocity - currentVelocity) / shipController.CalculateShipMass().PhysicalMass;

                // Se l'accelerazione necessaria supera il limite massimo, la normalizziamo
                if (requiredAcceleration.Length() > maxAcceleration)
                {
                    requiredAcceleration = requiredAcceleration.Normalized() * maxAcceleration;
                }

                return requiredAcceleration;
            }

            // Metodo per calcolare il percorso ottimale usando A*
            public List<Vector3D> CalculateOptimalPath(Vector3D start)
            {
                PathfindingAStar pathfinding = new PathfindingAStar(maxAcceleration, interdictionSpheres, targetPosition);
                return pathfinding.CalculateOptimalPath(start);
            }

            // Metodo per far navigare la nave verso i waypoint calcolati
            public void NavigateToWaypoints()
            {
                if (waypoints.Count == 0)
                    return;

                foreach (Vector3D waypoint in waypoints)
                {
                    // Verifica se il waypoint è troppo vicino
                    if (AdjustWaypoint(waypoint))
                        continue;

                    // Se abbiamo raggiunto il waypoint, trova il prossimo
                    if (RequiresAnotherWaypoint())
                    {
                        Vector3D nextWaypoint = FindNextWaypoint();
                        waypoints.Add(nextWaypoint); // Aggiungi il prossimo waypoint alla lista
                    }

                    // Naviga verso il waypoint
                    Vector3D requiredAcceleration = CalculateRequiredAcceleration(waypoint);

                    // Imposta la direzione di spinta in base all'accelerazione necessaria
                    shipController.SetThrustDirection(requiredAcceleration);
                }
            }

            // Metodo per calcolare l'euristica per A* (distanza minima tra la posizione attuale e il target)
            public double Heuristic(Vector3D position)
            {
                return (targetPosition - position).Length();
            }

            // Aggiustamento del waypoint: se il waypoint è troppo vicino, rimuovilo o spostalo
            private bool AdjustWaypoint(Vector3D waypoint)
            {
                double distanceToWaypoint = (waypoint - shipController.GetPosition()).Length();
                if (distanceToWaypoint < 100) // Definisci una distanza minima per un waypoint significativo
                {
                    waypoints.Remove(waypoint);  // Rimuovi il waypoint troppo vicino
                    return true; // Il waypoint è stato rimosso, quindi il prossimo sarà trovato
                }
                return false;
            }

            // Metodo per determinare se è necessario un altro waypoint
            private bool RequiresAnotherWaypoint()
            {
                // Questo può essere definito da un controllo sulla distanza rimasta dal target
                Vector3D currentPosition = shipController.GetPosition();
                return (targetPosition - currentPosition).Length() > 2000; // Se la distanza al target è maggiore di 2000, potrebbe essere utile un nuovo waypoint
            }

            // Trova il prossimo waypoint: cerca il prossimo punto lungo il percorso
            private Vector3D FindNextWaypoint()
            {
                // Qui potresti determinare un punto in base alla navigazione corrente
                Vector3D currentPosition = shipController.GetPosition();
                Vector3D direction = (targetPosition - currentPosition);
                direction.Normalize();

                // Trova il prossimo punto da aggiungere alla traiettoria
                return currentPosition + direction * 1000; // Supponiamo che ogni "step" sia di 1000 metri
            }
        }

    }

    
}
