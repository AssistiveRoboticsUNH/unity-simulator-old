using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class DiffDriveController : MonoBehaviour
{
    public ArticulationBody wheelLeft;
    public ArticulationBody wheelRight;
    public ArticulationBody body;

    public float wheelLeftRadius = 0.1f;
    public float wheelRightRadius = 0.1f;
    public float wheelSeparation = 0.2f;
    // public float gain = .5f;

    ROSConnection ros;
    public string topicName = "cmd_vel";

    private Vector3 linearVel = new Vector3(0, 0, 0);
    private Vector3 angularVel = new Vector3(0, 0, 0);
    // private Vector3 torque = new Vector3(0, 0, 0);

    private List<float> velocities;
    private List<float> forces;
    private List<int> startIndices;

    private ROSInterface ros_interface;
    void Start()
    {
        ros_interface = FindObjectOfType<ROSInterface>();
        velocities = new List<float>();
        body.GetDriveTargetVelocities(velocities);
        forces = new List<float>();
        body.GetJointForces(forces);
        startIndices = new List<int>();
        body.GetDofStartIndices(startIndices);
    }

    void SetVelocity()
    {
        var twist = ros_interface.native_twist;
        angularVel.z = (float) twist.angular.x;
        angularVel.x = (float)twist.angular.y;
        angularVel.y = (float)twist.angular.z;

        linearVel.z = (float)twist.linear.x;
        linearVel.x = (float)twist.linear.y;
        linearVel.y = (float)twist.linear.z;

    }

    // Update is called once per frame
    void Update()
    {
        
        // linearVel.z = 1f;
        // angularVel.y = 3f;
        SetVelocity();
        
        float velLeft = (linearVel.z - angularVel.y * wheelSeparation / 2.0f) / wheelLeftRadius;
        float velRight = (linearVel.z + angularVel.y * wheelSeparation / 2.0f) / wheelRightRadius;

        velocities[startIndices[wheelRight.index]] = velRight;
        velocities[startIndices[wheelLeft.index]] = velLeft;
        body.SetDriveTargetVelocities(velocities);
        // return;
        
        // var xDiff = linearVel.x - body.velocity.x;
        // var yDiff = linearVel.y - body.velocity.y;
        // var zDiff = linearVel.z - body.velocity.z;
        // List<float> target = new List<float>(){velLeft};
        // wheelLeft.SetDriveTargetVelocities(target);

        // var gainLeft = velLeft - wheelLeft.jointVelocity[0];
        // var gainI = gain + .5f*Math.Abs(angularVel.y -body.angularVelocity.y);
        // var gainI = gain;
        // torque.x = gainI * (velLeft - wheelLeft.jointVelocity[0]);
        // forces[startIndices[wheelLeft.index]] = Math.Clamp(torque.x, -20, 20);
        // // wheelLeft.AddRelativeTorque(torque);
        //
        // torque.x = gainI * (velRight - wheelRight.jointVelocity[0]);
        // forces[startIndices[wheelRight.index]] = Math.Clamp(torque.x, -20, 20);
        // // wheelRight.AddRelativeTorque(torque);
        //
        // body.SetJointForces(forces);


        // List<float> target2 = new List<float>(){velRight};
        // wheelRight.SetDriveTargetVelocities(target2);
        // torque.x = 0;
        // torque.y = .5f*(angularVel.y - (-body.angularVelocity.y));
        // body.AddTorque(torque);
        // torque.y = 0;

        // var yAngularDiff = angularVel.y - body.angularVelocity.y;


        // if (articulationBody != null)
        // {
        //     articulationBody.AddRelativeTorque(torque);    
        // }
        // if (GetComponent<Rigidbody>() != null)
        // {
        //     GetComponent<Rigidbody>().AddRelativeTorque(torque);    
        // }
    }
}