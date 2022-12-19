using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SingleDegreeJoint : MonoBehaviour
{
    public enum JointDegree
    {
        RotateX = 0,
        RotateY = 1,
        RotateZ = 2
    }
    public JointDegree degreeOfFreedom;
    private Vector3 _axis;
    void Start()
    {
        _axis = degreeOfFreedom switch
        {
            JointDegree.RotateX => new Vector3(1, 0, 0),
            JointDegree.RotateY => new Vector3(0, 1, 0),
            JointDegree.RotateZ => new Vector3(0, 0, 1),
            _ => _axis
        };
    }

    public void SetValue(float value)
    {
        var transform1 = transform;
        transform1.localEulerAngles = degreeOfFreedom switch
        {
            JointDegree.RotateX => new Vector3(value, 0, 0),
            JointDegree.RotateY => new Vector3(0, value, 0),
            JointDegree.RotateZ => new Vector3(0, 0, value),
            _ => transform1.localEulerAngles
        };
    }
    
    public float GetValue()
    {
        return transform.localEulerAngles[(int) degreeOfFreedom];
    }
    
}

