using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;

using System.IO;
public class CamIntrinsic : MonoBehaviour
{
    void Start()
    {
        Camera camera = GetComponent<Camera>();
        List<float> IntDataList = new List<float>(); 
        // Field of View (in degrees)
        float fieldOfView = camera.fieldOfView;

        // Sensor Size (in millimeters)
        float sensorWidth = camera.sensorSize.x;
        float sensorHeight = camera.sensorSize.y;

        // Image Resolution
        int imageWidth = camera.pixelWidth;
        int imageHeight = camera.pixelHeight;

        // Calculate focal length approximation
        float focalLengthX = imageWidth / (2 * Mathf.Tan(fieldOfView * 0.5f * Mathf.Deg2Rad) * sensorWidth);
        float focalLengthY = imageHeight / (2 * Mathf.Tan(fieldOfView * 0.5f * Mathf.Deg2Rad) * sensorHeight);

        // Principal Point (assuming it is at the center)
        float principalPointX = imageWidth / 2f;
        float principalPointY = imageHeight / 2f;
        
        IntDataList.Add(focalLengthX);
        IntDataList.Add(focalLengthY);
        IntDataList.Add(principalPointX);
        IntDataList.Add(principalPointY);
        
        string json = JsonConvert.SerializeObject(IntDataList);
        
        string downloadsPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory));
        string filePath = Path.Combine(downloadsPath, "IntDataList.json");
        File.WriteAllText(filePath, json);
    }

}