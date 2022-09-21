using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    [SerializeField] private Transform SensorFR;
    [SerializeField] private Transform SensorL1;
    [SerializeField] private Transform SensorL2;
    [SerializeField] private Transform SensorL3;
    [SerializeField] private Transform SensorR1;
    [SerializeField] private Transform SensorR2;
    [SerializeField] private Transform SensorR3;
    [SerializeField] private Transform SensorOR;

    [SerializeField] private float maxSteeringAngle;
    [SerializeField] private float motorForce;
    [SerializeField] private float brakeForce;

    private Rigidbody rb;

    [SerializeField] private float angle_x;
    [SerializeField] private float angle_z;
    [SerializeField] private float velocity;

    private float steerAngle;
    private bool isBreaking;
    private float s1dist =5f;
    private float s3dist = 8.4f;
    private float s2dist = 9.5f;
    Vector3 centerOfMassOffset  =new  Vector3(0, -1.0f, 0);

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        float s1x =0; float s1y = 15; float s1z =0;
        float s3x = 8.5f; float s3y =30; float s3z =0;
        float s2x =0; float s2y = 12; float s2z =0;
        AdjustSensors(SensorFR, 20, 0, 0);
        AdjustSensors(SensorL1, s1x, -s1y,s1z);
        AdjustSensors(SensorL3, s3x,-s3y,s3z);
        AdjustSensors(SensorR1, s1x,s1y,s1z);
        AdjustSensors(SensorR3, s3x,s3y,s3z);
        AdjustSensors(SensorL2,s2x, -s2y,s2z);
        AdjustSensors(SensorR2,s2x,-s2y,s2z);
        AdjustSensors(SensorOR, 50,180,0);
        rb.centerOfMass += centerOfMassOffset;
    }

    private void FixedUpdate()
    {
        StayOnRoad(); //Allows the vehicle to stay on the Road
        AvoidObstacles(); //Fuction for avoiding Obstacles
        AdjustSpeed(); //Here the speed or velocity of the vehicle is auto adjusted
        HandleMotor(); //this handles the car movement and torque and power
        UpdateWheel(); // this handles the turning of the vehicle

        angle_x = SensorOR.eulerAngles.x;
        angle_z = SensorOR.eulerAngles.z;

        velocity = rb.velocity.magnitude * 0.8f;
    }
    private void AdjustSensors(Transform sensors, float x_angle, float y_angle, float z_angle)
    {
        //Here we try to rotate the sensor to our preferred Rotation
        sensors.transform.Rotate(x_angle,y_angle,z_angle);
    }
    private void HandleMotor()
    {
        //The vehicle is been moved with motorTorque so we apply the motorTorque to all the wheel colliders
        frontLeftWheelCollider.motorTorque = motorForce;
        frontRightWheelCollider.motorTorque = motorForce;
        rearLeftWheelCollider.motorTorque = motorForce;
        rearRightWheelCollider.motorTorque = motorForce;

        brakeForce = isBreaking ? 4000f : 0f;
        frontLeftWheelCollider.brakeTorque = brakeForce;
        frontRightWheelCollider.brakeTorque = brakeForce;
        rearLeftWheelCollider.brakeTorque = brakeForce;
        rearRightWheelCollider.brakeTorque =brakeForce;
    }
    private void UpdateWheel()
    {
        //Updatingthe wheelRotation function for all collider
        UpdateWheelPos(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelPos(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelPos(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelPos(rearRightWheelCollider, rearRightWheelTransform);
    }
    private void UpdateWheelPos(WheelCollider wheelCollider, Transform trans)
    {
        //Getting the wheel to rotate when in motion
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        trans.rotation = rot;
        trans.position = pos;
    }
    private void HandleSteering(float direction)
    {
        steerAngle = maxSteeringAngle * direction;
        frontLeftWheelCollider.steerAngle = steerAngle;
        frontRightWheelCollider.steerAngle = steerAngle;
    }
    private bool sense(Transform sensor , float dist)
    {
        if(Physics.Raycast(sensor.position, sensor.TransformDirection(Vector3.forward), dist))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    private void StayOnRoad()
    {
        if(!sense(SensorL3, s3dist) || !sense(SensorR3 , s3dist))
        {
            if(!sense(SensorL3, s3dist) & !sense(SensorL2, s2dist))
            {
                HandleSteering(1);
            }
            if(!sense(SensorR3, s3dist) & !sense(SensorR2, s2dist))
            {
                HandleSteering(-1);
            }
        } else
        {
            HandleSteering(0);
        }
    }
    private void AdjustSpeed()
    {
        //We adjust the speed
        if(velocity < 5f & motorForce < 400)
        {
            motorForce = motorForce + 1.0f;
        }
        if(velocity > 6f & motorForce >0)
        {
            motorForce = motorForce -1.0f;
        }
    }
    private void AvoidObstacles()
    {
        //if we sense obstacle we take the opposite direction
        if(sense(SensorL1, s1dist))
        {
            HandleSteering(1);
        }
        if(sense(SensorR1, s1dist))
        {
            HandleSteering(-1);
        }
    }
}