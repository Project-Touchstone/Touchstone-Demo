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
    // NetworkStream for writeing/receiving data
    private NetworkStream stream;
    // Thread for receiving data asynchronously
    private Thread clientReceiveThread;

    // Endianness types for byte order
    public enum Endianness
    {
        LittleEndian,
        BigEndian
    }

    // write mode types for controlling when data is sent
    public enum WriteMode
    {
        IMMEDIATE,
        PACKET
    }

    // Current endianness (default: BigEndian)
    Endianness endianness = Endianness.BigEndian;
    // Current write mode (default: PACKET)
    WriteMode writeMode = WriteMode.PACKET;

    // Buffer for outgoing data
    List<byte> writeBuffer = new List<byte>();

    // Queue for request headers to track requests
    Queue<byte> requestQueue = new Queue<byte>();

    // Whether the current request has ended
    bool endFlag = true;

    // Buffer for incoming data
    List<byte> readBuffer = new List<byte>();

    // Byte for current response header
    byte responseHeader;

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

    // Set the write mode (immediate or packet)
    public void SetWriteMode(WriteMode writeMode)
    {
        this.writeMode = writeMode;
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
                    while (endFlag && getReadBufferSize() > 0 && getRequestQueueSize() > 0)
                    {
                        endFlag = false;
                        responseHeader = readByte();
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

    public byte getResponseHeader()
    {
        return responseHeader;
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

    // Returns the size of the write buffer
    public int getWriteBufferSize()
    {
        lock (dataLock)
        {
            return writeBuffer.Count;
        }
    }

    // write the current packet (all data in write buffer)
    public void writePacket()
    {
        if (stream != null)
        {
            try
            {
                stream.Write(writeBuffer.ToArray(), 0, writeBuffer.Count);
                writeBuffer.Clear();
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to write packet: {ex.Message}");
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
    
    // Add a single byte to the write buffer
    public void writeByte(byte value)
    {
        writeBytes(new byte[] { value });
    }

    // Add a header byte to the write buffer and queue
    public void writeHeader(byte value)
    {
        writeByte(value);
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

    // Add a byte array to the write buffer (and write immediately if in IMMEDIATE mode)
    public void writeBytes(byte[] buffer)
    {
        writeBuffer.AddRange(buffer);

        // writes packet immediately if in immediate mode
        if (writeMode == WriteMode.IMMEDIATE)
        {
            writePacket();
        }
    }

    // Add a float to the write buffer, handling endianness
    public void writeFloat(float value)
    {
            byte[] buffer = BitConverter.GetBytes(value);
            Array.Reverse(buffer); // Reverse the byte order to convert to big-endian
            writeBytes(buffer);
    }

    // Add a Vector3 (3 floats) to the write buffer
    public void writeVector3(Vector3 vector)
    {
        writeFloat(vector.x);
        writeFloat(vector.y);
        writeFloat(vector.z);
    }

    // Add a Quaternion (4 floats) to the write buffer
    public void writeQuaternion(Quaternion quaternion)
    {
        writeFloat(quaternion.x);
        writeFloat(quaternion.y);
        writeFloat(quaternion.z);
        writeFloat(quaternion.w);
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