using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;

using System.IO;

public class TransformData_
{
    public double posX;
    public double posY;
    public double posZ;
    public double quatX;
    public double quatY;
    public double quatZ;
    public double quatW;

    public string name;

}
public class CamLoctoJson : MonoBehaviour
{
    public List<Transform> transforms;
    
    void Start()
    {
        List<TransformData_> TransformDataList = new List<TransformData_>(); 
        foreach (var transform in transforms)
        {
            TransformData_ transformData_ = new TransformData_();
            transformData_.name = transform.name;
            transformData_.posX = transform.position.x;
            transformData_.posY = transform.position.z;
            transformData_.posZ = transform.position.y;
            transformData_.quatW = transform.rotation.w;
            transformData_.quatX = transform.rotation.x;
            transformData_.quatY = transform.rotation.z;
            transformData_.quatZ = transform.rotation.y;
            
            TransformDataList.Add(transformData_);
        }
        
        
        // Save json file with camera transforms to Desktop which will then be used to get extrinisic parameters
        
        string json = JsonConvert.SerializeObject(TransformDataList);
        
        string downloadsPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory));
        string filePath = Path.Combine(downloadsPath, "transformDataList.json");
        File.WriteAllText(filePath, json);

    }
    
    
}