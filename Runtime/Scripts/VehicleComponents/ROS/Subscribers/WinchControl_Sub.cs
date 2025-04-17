using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Drone;
using VehicleComponents.ROS.Core;  // for ROSBehaviour

namespace VehicleComponents.ROS.Subscribers
{
    public class WinchControl_Sub : ROSBehaviour
    {
        public Rope.Winch winchController;  // Reference to your actual Winch script

        protected override void StartROS()
        {
            rosCon = ROSConnection.GetOrCreateInstance();
            rosCon.Subscribe<WinchControlMsg>(topic, WinchControlCallback);
            Debug.Log($"Subscribed to {topic}");
        }

        void WinchControlCallback(WinchControlMsg msg)
        {
            Debug.Log($"[WinchControl_Sub] Received: target_length={msg.target_length}, speed={msg.winch_speed}");

            if (winchController != null)
            {
                winchController.TargetLength = Mathf.Clamp((float)msg.target_length, winchController.MinLength, winchController.RopeLength);
                winchController.WinchSpeed = (float)msg.winch_speed;
            }
            else
            {
                Debug.LogWarning("WinchControl_Sub: winchController not assigned.");
            }
        }
    }
}

// Setup in Unity Editor
// Add WinchControl_Sub as a new component to any GameObject.

// Set the topic field in the inspector to /winch_control.

// Drag and drop the GameObject that holds your Winch component into the winchController slot in the inspector.

// https://chatgpt.com/share/68011919-5228-8004-9393-d79c3fcc1781