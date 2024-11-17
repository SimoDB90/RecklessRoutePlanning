using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        Vector3D shipPos = new Vector3D(Vector3.Zero);
        Vector3D destinationPos = new Vector3D(Vector3.Zero);
        List<IMyShipController> cockpits = new List<IMyShipController>();
        IMyShipController cockpit;
        float radius = 250000f;
        Vector3D gps = new Vector3D(0,0,0);
        List<IMyGyro> gyroList = new List<IMyGyro>();
        float gyrosTolerance = 0.02f;
        MatrixD referenceMatrix;
        List<IMyThrust> allThrusts = new List<IMyThrust>();
        float mass;
        Dictionary<string, double> thrustDictionary = new Dictionary<string, double>();
        BezierCurve bz = new BezierCurve();
        public Program()
        {
            GridTerminalSystem.GetBlocksOfType(cockpits, x => x.CustomName.Contains("asd"));
            if(cockpits.Count > 0)
            {
                cockpit = cockpits[0];
            }
            
            referenceMatrix = cockpit.WorldMatrix;
            GridTerminalSystem.GetBlocksOfType(allThrusts);
            mass = cockpit.CalculateShipMass().TotalMass;
            thrustDictionary = ThrustPerDirection();
            GetAccleration();
            
        }
        public Dictionary<string, double>ThrustPerDirection()
        {
            double forwardForce = 0;
            double backwardForce = 0;
            double leftForce = 0;
            double rightForce = 0;
            double upForce = 0;
            double downForce = 0;

            // Definisci le 6 direzioni principali (come vettori unitari)
            Vector3D forwardDirection = new Vector3D(1, 0, 0);  // Direzione "forward" (destra)
            Vector3D backwardDirection = new Vector3D(-1, 0, 0); // Direzione "backward" (sinistra)
            Vector3D leftDirection = new Vector3D(0, 0, 1);     // Direzione "left" (avanti)
            Vector3D rightDirection = new Vector3D(0, 0, -1);    // Direzione "right" (indietro)
            Vector3D upDirection = new Vector3D(0, 1, 0);        // Direzione "up" (su)
            Vector3D downDirection = new Vector3D(0, -1, 0);     // Direzione "down" (giù)

            foreach (var t in allThrusts)
            {
                Vector3D thrustDirection = t.WorldMatrix.Forward;

                // Calcola il prodotto scalare tra la direzione di spinta e le 6 direzioni principali
                double dotForward = Vector3D.Dot(thrustDirection, forwardDirection);
                double dotBackward = Vector3D.Dot(thrustDirection, backwardDirection);
                double dotLeft = Vector3D.Dot(thrustDirection, leftDirection);
                double dotRight = Vector3D.Dot(thrustDirection, rightDirection);
                double dotUp = Vector3D.Dot(thrustDirection, upDirection);
                double dotDown = Vector3D.Dot(thrustDirection, downDirection);

                // Somma la forza in base alla direzione
                if (dotForward > 0) forwardForce += t.MaxEffectiveThrust * dotForward; // Contributo positivo
                if (dotBackward > 0) backwardForce += t.MaxEffectiveThrust * dotBackward; // Contributo positivo
                if (dotLeft > 0) leftForce += t.MaxEffectiveThrust * dotLeft; // Contributo positivo
                if (dotRight > 0) rightForce += t.MaxEffectiveThrust * dotRight; // Contributo positivo
                if (dotUp > 0) upForce += t.MaxEffectiveThrust * dotUp; // Contributo positivo
                if (dotDown > 0) downForce += t.MaxEffectiveThrust * dotDown; // Contributo positivo 
            }
            Dictionary<string, double> thrustForces = new Dictionary<string, double>
                {
                    { "Forward", forwardForce },
                    { "Backward", backwardForce },
                    { "Left", leftForce },
                    { "Right", rightForce },
                    { "Up", upForce },
                    { "Down", downForce }
                };
            return thrustForces;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if ((updateSource & (UpdateType.Trigger | UpdateType.Terminal)) > 0)
            {
                if (argument.ToLower() == "start")
                {
                    //CustomData();
                    //SetupBlocks();
                    //if (setupCompleted)
                    //{
                    //    Echo($"DRONE SETUP COMPLETED!\nVersion: {droneVersion}\nNumbers of thrusters in group: {ThrustersInGroup}\nCockpit Found \nProjector Found \nFuel Tank: {tank.Count}\nTag used: {TagCustom}");
                    //    IGC.SendBroadcastMessage(BroadcastTag, $"    |DRONE SETUP COMPLETED!\n|Version: {droneVersion}\n|Numbers of active thrusters: {ThrustersInGroup} " +
                    //        $"\n|Cockpit Found \n|Projector Found\n|Fuel Tank: {tank.Count}\n|Tag used: [{TagCustom}]");
                    //}
                }
            }
        }

        public double GetDistance()
        {
            shipPos = cockpit.GetPosition();
            Vector3D destination = gps - shipPos;
            return  destination.Length();
        }
        public Vector3D GetTangentPos()
        {
            Vector3D destination = gps - shipPos;
            double distance = destination.Length();
            double alfa = radius / distance;
            return gps + alfa * destination;
        }
        public void Aligning(Vector3D desiredForward, Vector3D desiredUp)
        {
            double pitch, yaw, roll;
            GetRotationAnglesSimultaneous(desiredForward, desiredUp, referenceMatrix, 
                out yaw, out pitch, out roll);
            ApplyGyroOverride(pitch, yaw, roll, gyroList, referenceMatrix);
        }

        public Dictionary<string, double> FindBestThrustDirection(Dictionary<string, double> thrustDict)
        {
            string bestDir = "";
            double bestForce = 0;
            foreach(var d in thrustDict)
            {
                if (d.Value>bestForce)
                {
                    bestForce = d.Value;
                    bestDir = d.Key;
                }
            }
            return new Dictionary<string, double>() 
            {
                { bestDir, bestForce }
            };
        }
        
        public float GetAccleration()
        {
            Dictionary<string, double>bestDict = new Dictionary<string, double>();
            bestDict = FindBestThrustDirection(thrustDictionary);
            string direction = "";
            double force = 0;
            foreach (var kv in bestDict)
            {
                direction = kv.Key;
                force = kv.Value;
            }
            return (float)force / mass;
        }

        void GetRotationAnglesSimultaneous(Vector3D desiredForwardVector, Vector3D desiredUpVector, MatrixD worldMatrix, out double yaw, out double pitch, out double roll)
        {
            desiredForwardVector = SafeNormalize(desiredForwardVector);

            MatrixD transposedWm;
            MatrixD.Transpose(ref worldMatrix, out transposedWm);
            Vector3D.Rotate(ref desiredForwardVector, ref transposedWm, out desiredForwardVector);
            Vector3D.Rotate(ref desiredUpVector, ref transposedWm, out desiredUpVector);

            Vector3D leftVector = Vector3D.Cross(desiredUpVector, desiredForwardVector);
            Vector3D axis;
            double angle;
            if (Vector3D.IsZero(desiredUpVector) || Vector3D.IsZero(leftVector))
            {
                //Echo("vector 0");
                axis = new Vector3D(desiredForwardVector.Y, -desiredForwardVector.X, 0);
                angle = Math.Acos(MathHelper.Clamp(-desiredForwardVector.Z, -1.0, 1.0));
            }
            else
            {
                leftVector = SafeNormalize(leftVector);
                Vector3D upVector = Vector3D.Cross(desiredForwardVector, leftVector);

                // Create matrix
                MatrixD targetMatrix = MatrixD.Zero;
                targetMatrix.Forward = desiredForwardVector;
                targetMatrix.Left = leftVector;
                targetMatrix.Up = upVector;

                axis = new Vector3D(targetMatrix.M23 - targetMatrix.M32,
                                    targetMatrix.M31 - targetMatrix.M13,
                                    targetMatrix.M12 - targetMatrix.M21);

                double trace = targetMatrix.M11 + targetMatrix.M22 + targetMatrix.M33;
                angle = Math.Acos(MathHelper.Clamp((trace - 1) * 0.5, -1, 1));
            }

            if (Vector3D.IsZero(axis))
            {
                angle = desiredForwardVector.Z < 0 ? 0 : Math.PI;
                yaw = angle;
                pitch = 0;
                roll = 0;
                return;
            }

            axis = SafeNormalize(axis);
            yaw = -axis.Y * angle;
            pitch = -axis.X * angle;
            roll = -axis.Z * angle;
            //Echo($"pitch: {pitch}\nyaw: {yaw}\nz: {roll}");
        }

        void ApplyGyroOverride(double pitchSpeed, double yawSpeed, double rollSpeed, List<IMyGyro> gyroList, MatrixD worldMatrix)
        {
            //Echo("applygyro");
            GridTerminalSystem.GetBlocksOfType(gyroList);
            foreach (var gyro in gyroList) { gyro.Enabled = true; }
            var rotationVec = new Vector3D(pitchSpeed, yawSpeed, rollSpeed);
            var relativeRotationVec = Vector3D.TransformNormal(rotationVec, worldMatrix);

            foreach (var thisGyro in gyroList)
            {

                var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(thisGyro.WorldMatrix));
                thisGyro.Pitch = (float)transformedRotationVec.X * 0.5f;
                thisGyro.Yaw = (float)transformedRotationVec.Y * 0.5f;
                thisGyro.Roll = (float)transformedRotationVec.Z * 0.5f;
                thisGyro.GyroOverride = true;
                if (Math.Abs(pitchSpeed) < gyrosTolerance && Math.Abs(yawSpeed) < gyrosTolerance && Math.Abs(rollSpeed) < gyrosTolerance)
                {

                    foreach (var gyro in gyroList) { gyro.GyroOverride = false; }

                }
            }
        }
        public static Vector3D SafeNormalize(Vector3D a)
        {
            if (Vector3D.IsZero(a))
                return Vector3D.Zero;

            if (Vector3D.IsUnit(ref a))
                return a;

            return Vector3D.Normalize(a);
        }

        
    }
}
