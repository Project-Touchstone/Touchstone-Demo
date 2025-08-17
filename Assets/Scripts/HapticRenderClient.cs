using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class HapticRenderClient : MonoBehaviour
{

    [SerializeField] private string serverAddress = "127.0.0.1"; // Default to localhost
    [SerializeField] private int serverPort = 8080; // Default port

    // Game object that controls haptic interaction
    public GameObject nodeObject;

    // Whether force feedback is sent to server
    public bool forceFeedback = true;

    // Whether collision feedback is sent to server
    public bool collisionFeedback = true;

    // Whether inertia is enabled
    public bool inertia = false;

    // Checkbox for enabling gravity on HapticShadow
    public bool gravity = false;

    // Stiffness and damping coefficients
    public float stiffness = 1f;
    public float damping = 1f;

    //Minimum force
    public float minForce = 0.01f;

    // Whether visualization is enabled
    public bool visualization = false;

    // Force visualization scale factor
    public float forceVisualScale = 100;

    // Debug mode allows keyboard based movement of haptic node
    public bool debugMode = false;

    // Movement speed in debug mode
    public float debugMovementSpeed = 0.1f;

    private TcpClientWrapper client;

    // Mutex
    private object commLock = new object();

    public enum Headers
    {
        // NODE_DATA: 0 bytes, 28 byte response (3 float cartesian position, 4 float quaternion orientation (i, j, k, w))
        NODE_DATA = 0x1,
        // FORCE_FEEDBACK: 12 bytes (3 float force), 0 byte response
        FORCE_FEEDBACK = 0x2,
        // COLLISION_FEEDBACK: 28 bytes (3 float point, 3 float normal, 1 float time to collision seconds), 0 byte response
        COLLISION_FEEDBACK = 0x3,

        // ACK: Followed by response data
        ACK = 0x1,
        // NACK: 0 bytes
        NACK = 0x2
    }

    // Haptic node script attached to node object
    private HapticNode node;

    // Cache previous gravity state
    private bool prevGravity;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    async void Start()
    {
        //Gets haptic node component
        node = nodeObject.GetComponent<HapticNode>();
        // Sets haptic render client
        node.SetHaptics(this);

        // Set initial gravity state on HapticShadow's Rigidbody
        var shadowObj = node.shadowObject;
        var shadowRb = shadowObj.GetComponent<Rigidbody>();
        shadowRb.useGravity = gravity;
        prevGravity = gravity;

        //Handles client setup
        client = new TcpClientWrapper();
        // Sets endianness and send mode
        client.SetEndianness(TcpClientWrapper.Endianness.BigEndian);
        client.SetWriteMode(TcpClientWrapper.WriteMode.PACKET);
        // Sets read handler
        client.SetReadHandler(ReadHandler);
        await client.ConnectToServer(serverAddress, serverPort);
    }

    private void PollServer()
    {
        if (!debugMode)
        {
            lock (commLock)
            {
                // Send feedback to the server
                if (forceFeedback)
                {
                    client.writeHeader((byte)Headers.FORCE_FEEDBACK);
                    // Send the coverted force vector to the server
                    client.writeVector3(unityToHardwareForce(node.GetForceOnMirror()));
                }
                // Send a request to the server for node data
                client.writeHeader((byte)Headers.NODE_DATA);
                client.writePacket();
            }
        }
    }

    private void ReadHandler(TcpClientWrapper client, byte request) {
        switch (request)
        {
            case (byte)Headers.NODE_DATA:
                {
                    if (client.getResponseHeader() == (byte)Headers.ACK)
                    {
                        if (client.getReadBufferSize() >= 28)
                        {
                            // Read the position, velocity, and orientation data from the server
                            Vector3 position = client.readVector3();
                            Quaternion orientation = client.readQuaternion();
                            // Update the node object's position and orientation with converted data
                            node.SetMirrorPos(hardwareToUnityPos(position));
                            node.SetMirrorRot(hardwareToUnityRot(orientation));
                            //Debug.Log($"Received position: {position}, orientation: {orientation}");
                            // Clear the packet
                            client.clearPacket();
                        }
                    }
                    else
                    {
                        // If the response is not an ACK, log an error
                        Debug.LogError("Node data not acknowledged by server");
                        client.clearPacket();
                    }
                    break;
                }
            case (byte)Headers.FORCE_FEEDBACK:
                {
                    if (client.getResponseHeader() != (byte)Headers.ACK)
                    {
                        Debug.LogError("Force feedback not acknowledged by server");
                    }
                    client.clearPacket();
                    break;
                }
            case (byte)Headers.COLLISION_FEEDBACK:
                {
                    if (client.getResponseHeader() != (byte)Headers.ACK)
                    {
                        Debug.LogError("Collision feedback not acknowledged by server");
                    }
                    client.clearPacket();
                    break;
                }
        }
    }

    public void SendCollisionCandidate(HapticShadow.CollisionCandidate candidate)
    {
        if (!debugMode && collisionFeedback)
        {
            lock (commLock)
            {
                client.writeHeader((byte)Headers.COLLISION_FEEDBACK);
                // Send the contact point, collision normal, and time until collision to the server
                client.writeVector3(unityToHardwarePos(nodeObject.transform.position + candidate.getCollisionPoint()));
                client.writeVector3(unityToHardwareForce(candidate.getCollisionNormal()));
                client.writeFloat(candidate.getTimeUntilCollision());
                client.writePacket();
            }
        }
    }

    // Fixed update is called once per physics frame
    void FixedUpdate()
    {
        PollServer();
    }

    // Update is called once per frame
    void Update()
    {
        // Update gravity state if changed
        if (node != null && node.shadowObject != null)
        {
            if (gravity != prevGravity)
            {
                var shadowRb = node.shadowObject.GetComponent<Rigidbody>();
                shadowRb.useGravity = gravity;
                prevGravity = gravity;
            }
        }
    }

    private Vector3 hardwareToUnityForce(Vector3 force)
    {
        // Convert from Cartesian coordinates to Unity coordinates
        Vector3 newForce = new Vector3(force.x, force.z, force.y);
        return newForce;
    }

    private Vector3 hardwareToUnityPos(Vector3 pos)
    {
        return hardwareToUnityForce(pos);
    }

    private Quaternion hardwareToUnityRot(Quaternion quaternion)
    {
        // Convert from Cartesian coordinates to Unity coordinates
        return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
    }

    private Vector3 unityToHardwareForce(Vector3 force)
    {
        // Convert from Unity coordinates to Cartesian coordinates
        Vector3 newForce = new Vector3(force.x, force.z, force.y);
        return newForce;
    }

    private Vector3 unityToHardwarePos(Vector3 pos)
    {
        return unityToHardwareForce(pos);
    }

    private Quaternion unityToHardwareRot(Quaternion quaternion)
    {
        // Convert from Unity coordinates to Cartesian coordinates
        return new Quaternion(-quaternion.x, -quaternion.z, -quaternion.y, quaternion.w);
    }

    private void OnApplicationQuit()
    {
        // Clean up resources when the application quits
        client.Close();
    }
}