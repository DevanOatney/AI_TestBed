using UnityEngine;
using System.Collections.Generic;

public class BasicVehicle : MonoBehaviour
{

    public Transform path;
    public float maxSteerAngle = 45f;
    public float turnSpeed = 5f;
    private WheelCollider wheelFL;
    private WheelCollider wheelFR;
    private WheelCollider wheelRL;
    private WheelCollider wheelRR;
    public float Acceleration = 5.0f;
    public float maxMotorTorque = 80f;
    public float maxBrakeTorque = 150f;
    private float currentSpeed;
    public float maxSpeed = 100f;
    public Vector3 centerOfMass;
    public bool isBraking = false;

    public Rigidbody CarRigidBody;

    [Header("Sensors")]
    public float SensorLengthMultiplier = 3f;
    private Vector3 frontSensorPosition = new Vector3(0f, 0.0f, 0.5f);
    private float frontSideSensorPosition = 0.0f;
    private float _sensorLengthDampener = 0.005f;

    private List<Transform> nodes;
    private int currectNode = 0;
    private bool avoiding = false;
    private float targetSteerAngle = 0;

    public int _whiskerCount = 5;
    public float MaxWhiskerLength = 10.0f;
    private List<float> _whiskerAngles = new List<float>();

    private void Start()
    {
        CarRigidBody.centerOfMass = centerOfMass;

        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }
        InitializeWheels();
        GenerateWhiskerAngles();
    }

    private void InitializeWheels()
    {
        wheelFL = this.transform.Find("Colliders/FL").GetComponent<WheelCollider>();
        wheelFR = this.transform.Find("Colliders/FR").GetComponent<WheelCollider>();
        wheelRL = this.transform.Find("Colliders/RL").GetComponent<WheelCollider>();
        wheelRR = this.transform.Find("Colliders/RR").GetComponent<WheelCollider>();
    }

    private void GenerateWhiskerAngles()
    {
        float initialAngle = 90.0f;
        float angleStep = (float)(initialAngle / (float)_whiskerCount);
        for (int i = 0; i < _whiskerCount; ++i)
        {
            _whiskerAngles.Add(initialAngle - (i * angleStep));
        }
    }

    private void FixedUpdate()
    {
        Sensors();
        ApplySteer();
        Drive();
        CheckWaypointDistance();
        Braking();
        LerpToSteerAngle();
    }

    private bool CheckWhiskerCollision(RaycastHit hit, Vector3 angle, ref float avoidMultiplier)
    {
        bool shouldAvoid = false;
        Rigidbody objRBody = hit.collider.gameObject.GetComponentInParent<Rigidbody>();
        if (objRBody != null)
        {
            Vector3 movingObjectVelocity = objRBody.velocity;
            Vector3 movingObjPos = hit.collider.transform.position;

            Vector3 myVelocity = CarRigidBody.velocity;
            Vector3 myPos = this.transform.position;

            Vector3 intersect;
            if (LinetoLineIntersection(out intersect, movingObjPos, movingObjectVelocity, myPos, myVelocity))
            {
                shouldAvoid = true;
                Debug.DrawLine(myPos, intersect, Color.blue);
            }
        }
        else if (!hit.collider.CompareTag("Terrain"))
        {
            shouldAvoid = true;
        }

        if (shouldAvoid == true)
        {
            avoiding = true;
            avoidMultiplier -= angle.x;
            return true;
        }
        return false;
    }

    public bool LinetoLineIntersection(out Vector3 intersection, Vector3 linePoint1, Vector3 lineVec1, Vector3 linePoint2, Vector3 lineVec2)
    {
        Vector3 lineVec3 = linePoint2 - linePoint1;
        Vector3 crossVec1and2 = Vector3.Cross(lineVec1, lineVec2);
        Vector3 crossVec3and2 = Vector3.Cross(lineVec3, lineVec2);

        float planarFactor = Vector3.Dot(lineVec3, crossVec1and2);

        //is coplanar, and not parrallel
        if (Mathf.Abs(planarFactor) < 0.0001f && crossVec1and2.sqrMagnitude > 0.0001f)
        {
            float s = Vector3.Dot(crossVec3and2, crossVec1and2) / crossVec1and2.sqrMagnitude;
            intersection = linePoint1 + (lineVec1 * s);
            return true;
        }
        else
        {
            intersection = Vector3.zero;
            return false;
        }
    }

    private void Sensors()
    {
        RaycastHit hit;
        Vector3 sensorStartPos = transform.position;
        sensorStartPos += transform.forward * frontSensorPosition.z;
        sensorStartPos += transform.up * frontSensorPosition.y;
        float leftAvoidMultiplier = 0;
        float rightAvoidMultiplier = 0;
        float sumAvoidMultiplier = 0;
        avoiding = false;
        targetSteerAngle = 0.0f;
        Vector3 sensorAngle = transform.forward;
        List<bool> leftWhiskers = new List<bool>();
        int leftWhiskerTriggerCount = 0;
        List<bool> rightWhiskers = new List<bool>();
        int rightWhiskerTriggerCount = 0;

        float whiskerLength = Mathf.Clamp((SensorLengthMultiplier * Acceleration * Mathf.PI * wheelFL.radius * wheelFL.rpm * 60  *0.001f) * _sensorLengthDampener, 0, MaxWhiskerLength);

        if (whiskerLength <= 0)
            return;
        //Right sensors
        sensorStartPos += transform.right * frontSideSensorPosition;
        for (int i = 0; i < _whiskerCount; ++i)
        {
            //front right whisker
            sensorAngle = Quaternion.AngleAxis(_whiskerAngles[i], transform.up) * transform.forward;
            if (Physics.Raycast(sensorStartPos, sensorAngle, out hit, whiskerLength))
            {
                bool isCollisionDetected = CheckWhiskerCollision(hit, sensorAngle, ref rightAvoidMultiplier);
                rightWhiskers.Add(isCollisionDetected);
                if (isCollisionDetected)
                {
                    rightWhiskerTriggerCount++;
                    Debug.DrawLine(sensorStartPos, hit.point, Color.red);
                }
            }
        }

        //Left sensors
        sensorStartPos -= transform.right * frontSideSensorPosition * 2;
        for (int i = 0; i < _whiskerCount; ++i)
        {
            sensorAngle = Quaternion.AngleAxis(-_whiskerAngles[i], transform.up) * transform.forward;
            if (Physics.Raycast(sensorStartPos, sensorAngle, out hit, whiskerLength))
            {
                bool isCollisionDetected = CheckWhiskerCollision(hit, sensorAngle, ref leftAvoidMultiplier);
                leftWhiskers.Add(isCollisionDetected);
                if (isCollisionDetected)
                {
                    leftWhiskerTriggerCount++;
                    Debug.DrawLine(sensorStartPos, hit.point, Color.red);
                }
            }
        }

        //front center sensor
        if (leftAvoidMultiplier == 0 && rightAvoidMultiplier == 0)
        {
            if (Physics.Raycast(sensorStartPos, transform.forward, out hit, whiskerLength))
            {
                float x = 0;
                bool isCollisionDetected = CheckWhiskerCollision(hit, transform.forward, ref x);
                {
                    Debug.DrawLine(sensorStartPos, hit.point, Color.white);
                    avoiding = true;
                    if (hit.normal.x < 0)
                    {
                        sumAvoidMultiplier = -0.5f;
                    }
                    else
                    {
                        sumAvoidMultiplier = 0.5f;
                    }
                }
            }
        }
        else
        {
            if (rightAvoidMultiplier > leftAvoidMultiplier)
                sumAvoidMultiplier = rightAvoidMultiplier;
            else
                sumAvoidMultiplier = leftAvoidMultiplier;
        }

        if (avoiding)
        {
            targetSteerAngle = maxSteerAngle * sumAvoidMultiplier;
        }
    }

    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(nodes[currectNode].position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        targetSteerAngle += newSteer;
    }

    private void Drive()
    {
        currentSpeed = Acceleration * Mathf.PI * wheelFL.radius * wheelFL.rpm * 60 * 0.001f;

        if (currentSpeed < maxSpeed && !isBraking)
        {
            wheelFL.motorTorque = maxMotorTorque;
            wheelFR.motorTorque = maxMotorTorque;
        }
        else
        {
            wheelFL.motorTorque = 0;
            wheelFR.motorTorque = 0;
        }
    }

    private void CheckWaypointDistance()
    {
        if (Vector3.Distance(transform.position, nodes[currectNode].position) < 1f)
        {
            if (currectNode == nodes.Count - 1)
            {
                currectNode = 0;
            }
            else
            {
                currectNode++;
            }
        }
    }

    private void Braking()
    {
        if (isBraking)
        {
            wheelRL.brakeTorque = maxBrakeTorque;
            wheelRR.brakeTorque = maxBrakeTorque;
        }
        else
        {
            wheelRL.brakeTorque = 0;
            wheelRR.brakeTorque = 0;
        }
    }
    private void LerpToSteerAngle()
    {
        wheelFL.steerAngle = Mathf.Lerp(wheelFL.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        wheelFR.steerAngle = Mathf.Lerp(wheelFR.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
    }
}
