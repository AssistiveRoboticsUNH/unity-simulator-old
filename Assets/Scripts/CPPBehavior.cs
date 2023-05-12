using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public class CPPBehavior : MonoBehaviour
{   ROSConnection ros;
    public string topicName = "unity_tf_bridge";
    private bool spaceKeyPressed = false;
    private bool collided = false;
    private int count = 0;
    private LineRenderer lineRenderer;
    private Collider robotCollider;

    [SerializeField] Vector2 planningCenter;
    [SerializeField] Vector2 planningSize;

    public float height = 0.66F;
    
    // OMPL 
    
    [SerializeField] double radius;
    [SerializeField] int pathResolution;
    [SerializeField] double planTime;
    [SerializeField] double dirScalar;
    [SerializeField] private double lambda;
    [SerializeField] private double widthScale;
    [SerializeField] private double numBasisPerMeter;

    private UnityOMPLInterface OMLInterface;
    private Vector3 goalOld;
    private Vector3 stateOld;
    
    private Vector3 goal;
    private float zAngleDegrees;
    
    private string topicName_pub = "waypoints";
    private string topicName_pose = "goal_pose";
    private HashSet<String> colliderNames;
    [SerializeField] Transform goal_;

    
    void Start()
    {
        robotCollider = GetComponent<Collider>();
        

        OMLInterface = new UnityOMPLInterface(IsStateValid, GetClosestPoint, GetState, RestoreState);
        Console.WriteLine(OMLInterface);
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = 0;
        lineRenderer.startColor = Color.red;
        lineRenderer.endColor = Color.red;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        
        Collider[] robotColliders = GetComponentsInChildren<Collider>(); 
        colliderNames = new HashSet<string>();
        foreach (var collider in robotColliders)
        {
            colliderNames.Add(collider.name);
        }
        // colliderNames.Add("Cube (4)");
        // colliderNames.Add("floor"); 
        goalOld = new Vector3();
        stateOld = new Vector3();


        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(topicName_pub);
        //ros.Subscribe<PoseMsg>(topicName_pose, NewCommand);
        goal = new Vector3(goal_.position.x, goal_.position.z, goal_.position.y);
        float zAngleDegrees = goal_.transform.eulerAngles.z;


        // colliders = Physics.OverlapSphere(gameObject.transform.position, 20.0f);
    }
    // void NewCommand(PoseMsg pose)
    // {
    //     // extract position and orientation components from pose
    //     goal = new Vector3((float)pose.position.x, (float)pose.position.z, (float)pose.position.y);
    //     Quaternion orientation = new Quaternion((float)pose.orientation.x, (float)pose.orientation.y, (float)pose.orientation.z, (float)pose.orientation.w);
    //     //goal =  pose.position.ToUnityVector3();
    //     //Quaternion orientation = pose.orientation.ToUnityQuaternion();
    //     float zAngleDegrees = orientation.eulerAngles.z;
    //
    //     // convert degrees to radians if needed
    //     //float zAngleRadians = zAngleDegrees * Mathf.Deg2Rad;
    //
    // }

    // Update is called once per frame
    void Update()
    {
        //Vector3 closest = GetComponent<Collider>().ClosestPoint(gameObject.transform.position);
        goal = new Vector3(goal_.position.x, goal_.position.z, goal_.position.y);
        float zAngleDegrees = goal_.transform.eulerAngles.z;
        
        if (!Input.GetKey(KeyCode.Space))
        {
            spaceKeyPressed = false;
        }

        bool cond = Vector3.Distance(goalOld ,goal) > 0.01 
                    || Vector3.Distance(stateOld,gameObject.transform.position) > 0.01;
        if (cond || Input.GetKey(KeyCode.Space) && !spaceKeyPressed)
        {
            goalOld = goal;
            stateOld = gameObject.transform.position;
            bool autoSimulation = Physics.autoSimulation;
            Physics.autoSimulation = false;
            spaceKeyPressed = true;
            Debug.Log("Plan Start");
            State goalState = new State();
            goalState.x = goal.x; //goal.position.x;
            goalState.y = goal.y; //goal.position.z;
            
            //goalState.theta = ((float) Math.PI)*goal.transform.eulerAngles.z/180.0f;
            goalState.theta = ((float) Math.PI)*zAngleDegrees/180.0f;
            count = 0;
            var points = OMLInterface.plan(radius , goalState, planningCenter, planningSize, planTime,pathResolution, dirScalar, lambda, widthScale, numBasisPerMeter);
            lineRenderer.positionCount = points.Count;
            int ind = 0;
            foreach (var point in points)
            {
                lineRenderer.SetPosition(ind, new Vector3(point.x, height, point.y));
                ind++;
                
                // Publish to waypoints topic 
                PointMsg msg = new PointMsg();
                msg.x = point.x;
                msg.y = point.y;
                msg.z = height;
                ros.Publish(topicName_pub, msg);
            }
            Physics.autoSimulation = true;
            Debug.Log("Done Plan, total count: " + count);
            
        }

    }
    
    void OnDrawGizmosSelected()
    {
        // Draw a semitransparent red cube at the transforms position
        Gizmos.color = new Color(0, 1, 0, 0.15f);
        Vector3 pos = new Vector3();
        pos.x = planningCenter.x;
        pos.z = planningCenter.y;
        pos.y = height;
        Vector3 size = new Vector3();
        size.x = planningSize.x;
        size.z = planningSize.y;
        Gizmos.DrawCube(pos, size);
        
        Gizmos.color = new Color(1, 0, 0, 0.4f);
        Gizmos.DrawSphere(gameObject.transform.position, (float)radius);
        
        // // Draw a semitransparent red cube at the goal position
        // Gizmos.color = new Color(1, 0, 0, 0.15f);
        // Vector3 pos_1 = new Vector3();
        // pos_1.x = goal.x;
        // pos_1.z = goal.y;
        // pos_1.y = 5;
        // Vector3 size_1 = new Vector3();
        // size_1.x = 5;
        // size_1.z = 5;
        // Gizmos.DrawCube(pos_1, size_1);
        
        // Gizmos.color = new Color(1, 0, 0, 0.4f);
        // Gizmos.DrawSphere(gameObject.transform.position, (float)radius);
    }

    public State GetState()
    {
        State stateStruct = new State();
        stateStruct.x = gameObject.transform.position.x;
        stateStruct.y = gameObject.transform.position.z;
        stateStruct.theta = ((float) Math.PI)*gameObject.transform.eulerAngles.z/180.0f;
        return stateStruct; 
    }
    public void RestoreState(State state)
    {
        Vector3 pos = gameObject.transform.position;
        pos.x = state.x;
        pos.y = height;
        pos.z = state.y;
        //pos.z = state.z;
        gameObject.transform.position = pos;
    }

    public bool IsStateValid(State state)
    {
        var pos = new Vector3();
        pos.y = height; // this is hacky
        pos.x = state.x;
        pos.z = state.y;
        
        count++;
        float robotRadius = 0.10f;
        bool valid = true;
        Collider[] colliders = Physics.OverlapSphere(pos, robotRadius);
        foreach (Collider collider in colliders) {
            if (colliderNames.Contains(collider.gameObject.name))
            {
                continue;
            }
            
            Vector3 closest = collider.ClosestPoint(pos);
            if (Vector3.Distance(closest, pos) < radius)
            {
                valid = false;
                break;
            }
        }

        return valid;
    }
    
    public bool GetClosestPoint(State state, ref State statePoint)
    {
        var pos = new Vector3();
        pos.y = height; // this is hacky
        pos.x = state.x;
        pos.z = state.y;

        Vector3 point = new Vector3();
        
        Collider[] colliders = Physics.OverlapSphere(pos, (float) radius);
        float dist = float.MaxValue;
        Vector3 ret = new Vector3();
        ret.x = dist;
        ret.y = dist;
        bool isPoint = false;
        
        foreach (Collider collider in colliders) {
            if (collider.gameObject == gameObject)
            {
                continue;
            }
            Vector3 closest = collider.ClosestPoint(pos);
            float newDist = Vector3.Distance(closest, pos);
            if (newDist < dist)
            {
                dist = newDist;
                point = closest;
                isPoint = true;
            }
        }

        statePoint.x = point.x;
        statePoint.y = point.z;
        return isPoint;
    }

}
