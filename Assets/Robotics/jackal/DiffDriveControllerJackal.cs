using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;


public class DiffDriveControllerJackal : MonoBehaviour
{
    UnityEngine.ArticulationBody myArticulationBody;
    public ArticulationBody wheelLeftRear;
    public ArticulationBody wheelRightRear;
    public ArticulationBody wheelLeftFront;
    public ArticulationBody wheelRightFront;
    public ArticulationBody body;

    public float wheelRadius = 0.1f;
    public float wheelSeparation = 0.36f;
    
    public float wheelSeparationMultiplier = 2f;
    // public float gain = .5f;

    ROSConnection ros;
    public string topicName = "cmd_vel";

    private Vector3 linearVel = new Vector3(0, 0, 0);
    private Vector3 angularVel = new Vector3(0, 0, 0);
    // private Vector3 torque = new Vector3(0, 0, 0);

    private List<float> velocities;
    private List<float> forces;
    private List<int> startIndices;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, NewCommand);
        
        velocities = new List<float>();
        body.GetDriveTargetVelocities(velocities);
        forces = new List<float>();
        body.GetJointForces(forces);
        startIndices = new List<int>();
        body.GetDofStartIndices(startIndices);

    }

    void NewCommand(TwistMsg cmd)
    {
        angularVel.z = (float)cmd.angular.x*1f;
        angularVel.x = (float)cmd.angular.y*1f;
        angularVel.y = (float)cmd.angular.z*1f;

        linearVel.z = (float)cmd.linear.x;
        linearVel.x = (float)cmd.linear.y;
        linearVel.y = (float)cmd.linear.z;

    }

    // Update is called once per frame
    void Update()
    {
        
        // linearVel.z = 1f;
        // angularVel.y = 3f;
        
        float velLeft = (linearVel.z - angularVel.y * wheelSeparation * wheelSeparationMultiplier / 2.0f) / wheelRadius;
        float velRight = (linearVel.z + angularVel.y * wheelSeparation * wheelSeparationMultiplier / 2.0f) / wheelRadius;


        velocities[startIndices[wheelLeftRear.index]] = velLeft;
        velocities[startIndices[wheelLeftFront.index]] = velLeft;
        velocities[startIndices[wheelRightRear.index]] = velRight;
        velocities[startIndices[wheelRightFront.index]] = velRight;
        body.SetDriveTargetVelocities(velocities);
        
        

    }
}

// using System;
// using System.Collections;
// using System.Collections.Generic;
// using RosMessageTypes.Geometry;
// using RosMessageTypes.Std;
// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
//
// public class DiffDriveControllerCMD : MonoBehaviour
// {
//
//     public float linearScale = 1.0f;
//     public float angularScale = 1.0f;
//     private Rigidbody rb;
//     ROSConnection ros;
//     public string topicName = "cmd_vel";
//     
//     private bool hasReceivedMsg = false;
//     private Vector3 linearVelocity;
//     private Vector3 angularVelocity;
//
//     
//
//     // Start is called before the first frame update
//     void Start()
//     {
//         ros = ROSConnection.GetOrCreateInstance();
//         ros.Subscribe<TwistMsg>(topicName, NewCommand);
//         
//         // Get reference to rigidbody component
//         rb = GetComponent<Rigidbody>();
//         
//     }
//
//     void NewCommand(TwistMsg msg)
//     {
//         // angularVel.z = (float)cmd.angular.x;
//         // angularVel.x = (float)cmd.angular.y;
//         // angularVel.y = (float)cmd.angular.z;
//         //
//         // linearVel.z = (float)cmd.linear.x;
//         // linearVel.x = (float)cmd.linear.y;
//         // linearVel.y = (float)cmd.linear.z;
//         //
//         linearVelocity = new Vector3((float)msg.linear.x * linearScale, 0, (float)msg.linear.y * linearScale);
//         angularVelocity = new Vector3(0, (float)msg.angular.z * angularScale, 0);
//         hasReceivedMsg = true;
//
//     }
//
//     // FixedUpdate is called at a fixed interval, usually every physics step, which is
//     // typically 50 times per second. This function is used for physics-related updates,
//     // such as moving rigidbodies or applying forces to objects. 
//     void FixedUpdate()
//     {
//         if (hasReceivedMsg)
//         {
//             rb.AddRelativeForce(linearVelocity, ForceMode.VelocityChange);
//             rb.AddRelativeTorque(angularVelocity, ForceMode.VelocityChange);
//             hasReceivedMsg = false;
//         }
//     }
// }
