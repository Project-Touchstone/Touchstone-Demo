using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;

public class TcpClientWrapper
{
    private TcpClient client;
    private NetworkStream stream;
    private Thread clientReceiveThread;

    //Endianness types
    public enum Endianness
    {
        LittleEndian,
        BigEndian
    }

    // Send mode types
    public enum SendMode
    {
        IMMEDIATE,
        PACKET
    }

    // Endianness and send mode
    Endianness endianness = Endianness.BigEndian;
    SendMode sendMode = SendMode.PACKET;

    // Send buffer
    List<byte> sendBuffer = new List<byte>();

    // Queue for request headers
    Queue<byte> requestQueue = new Queue<byte>();

    //Whether current request has ended
    bool endFlag = true;

    // Read buffer
    List<byte> readBuffer = new List<byte>();

    // Read handler
    Action<TcpClientWrapper, byte> readHandler;

    // Mutex for thread-safing
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
            clientReceiveThread = new Thread(new ThreadStart(ListenForData));
            clientReceiveThread.IsBackground = true;
            clientReceiveThread.Start();
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to connect to server: {ex.Message}");
        }
    }

    public void SetEndianness(Endianness endianness)
    {
        this.endianness = endianness;
    }

    public void SetSendMode(SendMode sendMode)
    {
        this.sendMode = sendMode;
    }

    public void SetReadHandler(Action<TcpClientWrapper, byte> readHandler)
    {
        this.readHandler = readHandler;
    }

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

    public int getReadBufferSize()
    {
        lock (dataLock)
        {
            return readBuffer.Count;
        }
    }

    public async Task AwaitBytes(int bytes)
    {
        while (getReadBufferSize() < bytes)
        {
            await Task.Yield();
        }
    }

    public byte readByte()
    {
        byte[] buffer = new byte[1];
        readBytes(buffer);
        return buffer[0];
    }

    public void readBytes(byte[] buffer)
    {
        lock (dataLock)
        {
            byte[] data = readBuffer.GetRange(0, buffer.Length).ToArray();
            Array.Copy(data, 0, buffer, 0, buffer.Length);
            readBuffer.RemoveRange(0, buffer.Length);
        }
    }

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

    public Vector3 readVector3()
    {
        Vector3 vector = new Vector3();
        vector.x = readFloat();
        vector.y = readFloat();
        vector.z = readFloat();
        return vector;
    }

    public Quaternion readQuaternion()
    {
        Quaternion quaternion = new Quaternion();
        quaternion.x = readFloat();
        quaternion.y = readFloat();
        quaternion.z = readFloat();
        quaternion.w = readFloat();
        return quaternion;
    }

    public void flush()
    {
        lock (dataLock)
        {
            readBuffer.Clear();
        }
    }

    public int getSendBufferSize()
    {
        lock (dataLock)
        {
            return sendBuffer.Count;
        }
    }

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

    public void clearPacket()
    {
        lock (dataLock)
        {
            requestQueue.Dequeue();
            endFlag = true;
        }
    }
    
    public void sendByte(byte value)
    {
        sendBytes(new byte[] { value });
    }

    public void sendHeader(byte value)
    {
        sendByte(value);
        lock (dataLock)
        {
            requestQueue.Enqueue(value);
        }
    }

    public int getRequestQueueSize()
    {
        lock (dataLock)
        {
            return requestQueue.Count;
        }
    }

    public void sendBytes(byte[] buffer)
    {
        sendBuffer.AddRange(buffer);

        // Sends packet immediately if in immediate mode
        if (sendMode == SendMode.IMMEDIATE)
        {
            sendPacket();
        }
    }

    public void sendFloat(float value)
    {
            byte[] buffer = BitConverter.GetBytes(value);
            Array.Reverse(buffer); // Reverse the byte order to convert to big-endian
            sendBytes(buffer);
    }

    public void sendVector3(Vector3 vector)
    {
        sendFloat(vector.x);
        sendFloat(vector.y);
        sendFloat(vector.z);
    }

    public void sendQuaternion(Quaternion quaternion)
    {
        sendFloat(quaternion.x);
        sendFloat(quaternion.y);
        sendFloat(quaternion.z);
        sendFloat(quaternion.w);
    }

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