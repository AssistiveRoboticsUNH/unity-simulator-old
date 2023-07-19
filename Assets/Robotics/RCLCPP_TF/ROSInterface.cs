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
public struct NativeTransform
{
    public IntPtr frame_id;
    public NativeVector3 translation;
    public NativeQuaternion rotation;
}


public class ROSInterface : MonoBehaviour
{
    public List<Transform> transforms;

    // Used to determine how much time has elapsed since the last message was published
    private IntPtr handle;
    private NativeTransform native_transform;

    
    // Start is called before the first frame update
    void Start()
    {
        handle = Init();
        native_transform = new NativeTransform();
    }

    // Update is called once per frame
    void Update()
    {
                
                foreach (var transform in transforms)
                {
                    native_transform.frame_id = Marshal.StringToHGlobalAnsi(transform.name);
					
                    var trans = new NativeVector3();
					var position_FLU = transform.position.To<FLU>();
                    trans.x = position_FLU.x;
                    trans.y = position_FLU.y;
                    trans.z = position_FLU.z;
                    native_transform.translation = trans;

                    var rotation = new NativeQuaternion();
					var rotation_FLU = transform.rotation.To<FLU>();                    
					rotation.w = rotation_FLU.w;
                    rotation.x = rotation_FLU.x;
                    rotation.y = rotation_FLU.y;
                    rotation.z = rotation_FLU.z;
                    native_transform.rotation = rotation;
                    
                    PublishTF(handle, ref native_transform);
                    
                    Marshal.FreeHGlobal(native_transform.frame_id);
                }

    }
    
    [DllImport("libROSInterface.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)] 
    private static extern IntPtr Init();
    [DllImport("libROSInterface.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)] 
    private static extern void Destroy(IntPtr handle);
    [DllImport("libROSInterface.so", EntryPoint = "PublishTF", CallingConvention = CallingConvention.Cdecl)] 
    private static extern bool PublishTF(IntPtr handle, ref NativeTransform input);
}