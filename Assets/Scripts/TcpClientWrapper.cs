using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class TcpClientWrapper
{
    // TcpClient for managing the connection
    private TcpClient client;
    // NetworkStream for sending/receiving data
    private NetworkStream stream;
    // Thread for receiving data asynchronously
    private Thread clientReceiveThread;

    // Endianness types for byte order
    public enum Endianness
    {
        LittleEndian,
        BigEndian
    }

    // Send mode types for controlling when data is sent
    public enum SendMode
    {
        IMMEDIATE,
        PACKET
    }

    // Current endianness (default: BigEndian)
    Endianness endianness = Endianness.BigEndian;
    // Current send mode (default: PACKET)
    SendMode sendMode = SendMode.PACKET;

    // Buffer for outgoing data
    List<byte> sendBuffer = new List<byte>();

    // Queue for request headers to track requests
    Queue<byte> requestQueue = new Queue<byte>();

    // Whether the current request has ended
    bool endFlag = true;

    // Buffer for incoming data
    List<byte> readBuffer = new List<byte>();

    // Handler to process incoming data
    Action<TcpClientWrapper, byte> readHandler;

    // Mutex for thread safety
    private object dataLock = new object();


    // Connects to server asynchronously and starts listening thread
    public async Task ConnectToServer(string serverAddress, int serverPort)
    {
        try
        {
            client = new TcpClient();
            Debug.Log($"Connecting to server at {serverAddress}:{serverPort}...");
            await client.ConnectAsync(serverAddress, serverPort);
            client.Client.NoDelay = true;
            stream = client.GetStream();
            Debug.Log("Connected to server.");
            // Start background thread to listen for incoming data
            clientReceiveThread = new Thread(new ThreadStart(ListenForData));
            clientReceiveThread.IsBackground = true;
            clientReceiveThread.Start();
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to connect to server: {ex.Message}");
        }
    }

    // Set the endianness for byte order
    public void SetEndianness(Endianness endianness)
    {
        this.endianness = endianness;
    }

    // Set the send mode (immediate or packet)
    public void SetSendMode(SendMode sendMode)
    {
        this.sendMode = sendMode;
    }

    // Set the handler for processing incoming data
    public void SetReadHandler(Action<TcpClientWrapper, byte> readHandler)
    {
        this.readHandler = readHandler;
    }

    // Thread method: listens for incoming data and processes requests
    private void ListenForData()
    {
        try
        {
            byte[] bytes = new byte[1024];
            while (true)
            {
                // Read incoming stream into byte array.
                while (stream.DataAvailable)
                {
                    int length = stream.Read(bytes, 0, bytes.Length);
                    var incomingData = new byte[length];
                    Array.Copy(bytes, 0, incomingData, 0, length);
                    lock (dataLock)
                    {
                        readBuffer.AddRange(incomingData);
                    }
                    // If a request is pending, process it using the handler
                    while (endFlag && getRequestQueueSize() > 0)
                    {
                        endFlag = false;
                        readHandler(this, requestQueue.Peek());
                    }
                }
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
        }
    }

    // Returns the size of the read buffer
    public int getReadBufferSize()
    {
        lock (dataLock)
        {
            return readBuffer.Count;
        }
    }

    // Await until the specified number of bytes are available in the read buffer
    public async Task AwaitBytes(int bytes)
    {
        while (getReadBufferSize() < bytes)
        {
            await Task.Yield();
        }
    }

    // Read a single byte from the buffer
    public byte readByte()
    {
        byte[] buffer = new byte[1];
        readBytes(buffer);
        return buffer[0];
    }

    // Read a specified number of bytes into the provided buffer
    public void readBytes(byte[] buffer)
    {
        lock (dataLock)
        {
            byte[] data = readBuffer.GetRange(0, buffer.Length).ToArray();
            Array.Copy(data, 0, buffer, 0, buffer.Length);
            readBuffer.RemoveRange(0, buffer.Length);
        }
    }

    // Read a float (4 bytes) from the buffer, handling endianness
    public float readFloat()
    {
        try
        {
            byte[] buffer = new byte[4];
            readBytes(buffer);
            Array.Reverse(buffer); // Reverse the byte order for little-endian
            float floatValue = BitConverter.ToSingle(buffer, 0);
            return floatValue;
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to read float: {ex.Message}");
            return 0f;
        }
    }

    // Read a Vector3 (3 floats) from the buffer
    public Vector3 readVector3()
    {
        Vector3 vector = new Vector3();
        vector.x = readFloat();
        vector.y = readFloat();
        vector.z = readFloat();
        return vector;
    }

    // Read a Quaternion (4 floats) from the buffer
    public Quaternion readQuaternion()
    {
        Quaternion quaternion = new Quaternion();
        quaternion.x = readFloat();
        quaternion.y = readFloat();
        quaternion.z = readFloat();
        quaternion.w = readFloat();
        return quaternion;
    }

    // Clear the read buffer
    public void flush()
    {
        lock (dataLock)
        {
            readBuffer.Clear();
        }
    }

    // Returns the size of the send buffer
    public int getSendBufferSize()
    {
        lock (dataLock)
        {
            return sendBuffer.Count;
        }
    }

    // Send the current packet (all data in send buffer)
    public void sendPacket()
    {
        if (stream != null)
        {
            try
            {
                stream.Write(sendBuffer.ToArray(), 0, sendBuffer.Count);
                sendBuffer.Clear();
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to send packet: {ex.Message}");
            }
        }
    }

    // Clear the current request from the queue and mark as ended
    public void clearPacket()
    {
        lock (dataLock)
        {
            requestQueue.Dequeue();
            endFlag = true;
        }
    }
    
    // Add a single byte to the send buffer
    public void sendByte(byte value)
    {
        sendBytes(new byte[] { value });
    }

    // Add a header byte to the send buffer and queue
    public void sendHeader(byte value)
    {
        sendByte(value);
        lock (dataLock)
        {
            requestQueue.Enqueue(value);
        }
    }

    // Returns the number of pending requests in the queue
    public int getRequestQueueSize()
    {
        lock (dataLock)
        {
            return requestQueue.Count;
        }
    }

    // Add a byte array to the send buffer (and send immediately if in IMMEDIATE mode)
    public void sendBytes(byte[] buffer)
    {
        sendBuffer.AddRange(buffer);

        // Sends packet immediately if in immediate mode
        if (sendMode == SendMode.IMMEDIATE)
        {
            sendPacket();
        }
    }

    // Add a float to the send buffer, handling endianness
    public void sendFloat(float value)
    {
            byte[] buffer = BitConverter.GetBytes(value);
            Array.Reverse(buffer); // Reverse the byte order to convert to big-endian
            sendBytes(buffer);
    }

    // Add a Vector3 (3 floats) to the send buffer
    public void sendVector3(Vector3 vector)
    {
        sendFloat(vector.x);
        sendFloat(vector.y);
        sendFloat(vector.z);
    }

    // Add a Quaternion (4 floats) to the send buffer
    public void sendQuaternion(Quaternion quaternion)
    {
        sendFloat(quaternion.x);
        sendFloat(quaternion.y);
        sendFloat(quaternion.z);
        sendFloat(quaternion.w);
    }

    // Close the connection and clean up resources
    public void Close()
    {
        // Clean up resources when the application quits
        if (stream != null)
        {
            stream.Close();
        }

        if (client != null)
        {
            client.Close();
        }

        Debug.Log("Disconnected from server.");
    }
}