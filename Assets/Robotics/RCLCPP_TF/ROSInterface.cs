using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using Random = UnityEngine.Random;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeQuaternion
{
    public double x;
    public double y;
    public double z;
    public double w;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeVector3
{
    public double x;
    public double y;
    public double z;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeTwist
{
    public NativeVector3 linear;
    public NativeVector3 angular;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeTransform
{
    public IntPtr frame_id;
    public IntPtr child_frame_id;
    public NativeVector3 translation;
    public NativeQuaternion rotation;
}

[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeOdom
{
    public NativeTransform pose;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] pose_covariance;

    public NativeVector3 linear_velocity;
    public NativeVector3 angular_velocity;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
    public double[] twist_covariance;
}


[StructLayout(LayoutKind.Sequential)]
[System.Serializable]
public struct NativeActivity
{
    public int person_eating;
    public int person_taking_medicine;
}


public class ROSInterface : MonoBehaviour
{
    // interface
    public List<Transform> transforms;
    public Animator animator;
    public Transform odom_transform;

    // native
    private IntPtr handle;
    private NativeTransform native_transform;
    private NativeActivity native_activity;
    private NativeOdom native_odom;
    public NativeTwist native_twist;

    // helper
    private Vector3 prev_forward;
    private Vector3 prev_position;

    void Start()
    {
        prev_forward = new Vector3();
        prev_position = new Vector3();
        prev_position = odom_transform.position;
        prev_forward = odom_transform.forward;

        handle = Init();
        native_transform = new NativeTransform();

        native_odom = new NativeOdom();
        native_odom.pose.frame_id = Marshal.StringToHGlobalAnsi("odom");
        native_odom.pose.child_frame_id = Marshal.StringToHGlobalAnsi(odom_transform.name);
        native_odom.pose_covariance = new double[36];
        native_odom.pose_covariance[0] = 0.1;
        native_odom.pose_covariance[7] = 0.1;
        native_odom.pose_covariance[14] = 1000000000000.0;
        native_odom.pose_covariance[21] = 1000000000000.0;
        native_odom.pose_covariance[28] = 1000000000000.0;
        native_odom.pose_covariance[35] = 1000000000000.0;
        native_odom.twist_covariance = new double[36];
        native_odom.twist_covariance[0] = 1000000000000.0;
        native_odom.twist_covariance[7] = 1000000000000.0;
        native_odom.twist_covariance[14] = 1000000000000.0;
        native_odom.twist_covariance[21] = 1000000000000.0;
        native_odom.twist_covariance[28] = 1000000000000.0;
        native_odom.twist_covariance[35] = 1000000000000.0;
    }

    void FixedUpdate()
    {
        var velocity = (odom_transform.position - prev_position) / Time.deltaTime;
        var tmp = Vector3.Dot(odom_transform.forward, prev_forward);
        tmp = Math.Max(tmp, 0.0f);
        tmp = Math.Min(tmp, 1.0f);
        var omega = Math.Acos(tmp) / Time.deltaTime;
        var angular_velocity = Vector3.Cross(odom_transform.forward, prev_forward);
        angular_velocity.Normalize();

        Vector3 tmp2 = new Vector3();
        tmp2.x = Vector3.Dot(odom_transform.forward, velocity);
        tmp2.y = Vector3.Dot(odom_transform.up, velocity);
        tmp2.z = Vector3.Dot(odom_transform.right, velocity);
        var flu = tmp2; //.To<FLU>();
        native_odom.linear_velocity.x = flu.x;
        native_odom.linear_velocity.y = flu.y;
        native_odom.linear_velocity.z = flu.z;

        Vector3 tmp3 = new Vector3();
        tmp3.x = (float)omega * angular_velocity.x;
        tmp3.y = (float)omega * angular_velocity.y;
        tmp3.z = (float)omega * angular_velocity.z;
        flu = tmp3; //.To<FLU>();
        native_odom.angular_velocity.x = flu.x;
        native_odom.angular_velocity.y = flu.y;
        native_odom.angular_velocity.z = flu.z;

        prev_position = odom_transform.position;
        prev_forward = odom_transform.forward;
    }


    void SetTransform(Transform trans, ref NativeTransform native)
    {
        var translation = new NativeVector3();
        var position_FLU = trans.position.To<FLU>();
        translation.x = position_FLU.x;
        translation.y = position_FLU.y;
        translation.z = position_FLU.z;
        native.translation = translation;

        var rotation = new NativeQuaternion();
        var rotation_FLU = trans.rotation.To<FLU>();
        rotation.w = rotation_FLU.w;
        rotation.x = rotation_FLU.x;
        rotation.y = rotation_FLU.y;
        rotation.z = rotation_FLU.z;
        native.rotation = rotation;
    }

    void Update()
    {
        // if (Input.GetKeyDown(KeyCode.Space)) {
        foreach (var trans in transforms)
        {
            native_transform.frame_id = Marshal.StringToHGlobalAnsi(trans.name);
            native_transform.child_frame_id = Marshal.StringToHGlobalAnsi("unity");
            SetTransform(trans, ref native_transform);
            PublishTF(handle, ref native_transform);

            Marshal.FreeHGlobal(native_transform.frame_id);
            Marshal.FreeHGlobal(native_transform.child_frame_id);
        }

        native_activity.person_eating = 0;
        native_activity.person_taking_medicine = 0;
        if (animator.GetCurrentAnimatorStateInfo(0).IsName("eating"))
        {
            native_activity.person_eating = 1;
        }

        if (animator.GetCurrentAnimatorStateInfo(0).IsName("pill"))
        {
            native_activity.person_taking_medicine = 1;
        }

        PublishActivity(handle, ref native_activity);

        SetTransform(odom_transform, ref native_odom.pose);
        PublishOdom(handle, ref native_odom);

        // if (Input.GetKeyDown(KeyCode.Space))
        ReceiveCmdVel(handle, ref native_twist);
    }

    [DllImport("libROSInterface.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr Init();

    [DllImport("libROSInterface.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)]
    private static extern void Destroy(IntPtr handle);

    [DllImport("libROSInterface.so", EntryPoint = "PublishTF", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishTF(IntPtr handle, ref NativeTransform input);

    [DllImport("libROSInterface.so", EntryPoint = "PublishActivity", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishActivity(IntPtr handle, ref NativeActivity input);

    [DllImport("libROSInterface.so", EntryPoint = "PublishOdom", CallingConvention = CallingConvention.Cdecl)]
    private static extern void PublishOdom(IntPtr handle, ref NativeOdom input);

    [DllImport("libROSInterface.so", EntryPoint = "ReceiveCmdVel", CallingConvention = CallingConvention.Cdecl)]
    private static extern void ReceiveCmdVel(IntPtr handle, ref NativeTwist output);
}