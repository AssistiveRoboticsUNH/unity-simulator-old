using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace Nathan
{
    public class DoorController : MonoBehaviour
    {
        Animator animator;
        ROSConnection ros;
        public string topicName = "smartthings_sensors_door";
        public bool open = false;

        void Start()
        {   ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<BoolMsg>(topicName);
            animator = GetComponent<Animator>();
        }

        // Update is called once per frame
        void Update()
        {
            BoolMsg cubePos = new BoolMsg();
            if (open != animator.GetBool("open"))
            {
                animator.SetBool("open", open);
            }

            if (animator.GetBool("open"))
            {
                cubePos.data = true;
            }
            else
            {
                cubePos.data = false;
            }
            ros.Publish(topicName, cubePos);

        }
    }
}