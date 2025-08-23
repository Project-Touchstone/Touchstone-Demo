using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;
using Unity.VisualScripting;

public class MinBiTTcpClient
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
        BULK
    }

    // Request object for tracking requests
     public class Request
    {
        public enum Status
        {
            UNSENT,
            WAITING,
            FULFILLED,
            TIMEDOUT
        }

        private static long nextId = 1;
        private long id;
        private byte header;
        private byte responseHeader;
        private int responseLength;
        private Status status;
        private DateTime sentTime;

        // Mutex for thread safety
        private readonly object requestLock = new object();

        public Request(byte header)
        {
            this.header = header;
            this.responseHeader = 0;
            this.responseLength = -1;
            this.status = Status.UNSENT;
            this.id = Interlocked.Increment(ref nextId);
        }

        public void Start()
        {
            sentTime = DateTime.UtcNow;
            SetStatus(Status.WAITING);
        }

        public void SetStatus(Status newStatus)
        {
            lock (requestLock)
            {
                status = newStatus;
            }
        }

        public void SetResponseHeader(byte responseHeader)
        {
            lock (requestLock)
            {
                this.responseHeader = responseHeader;
            }
        }

        public void SetResponseLength(int responseLength)
        {
            lock (requestLock)
            {
                this.responseLength = responseLength;
            }
        }

        public Status GetStatus()
        {
            lock (requestLock)
            {
                return status;
            }
        }

        public long GetId()
        {
            return id;
        }

        public byte GetHeader()
        {
            return header;
        }

        public byte GetResponseHeader()
        {
            lock (requestLock)
            {
                return responseHeader;
            }
        }

        public int GetResponseLength()
        {
            lock (requestLock)
            {
                return responseLength;
            }
        }

        public bool IsWaiting()
        {
            lock (requestLock)
            {
                return status == Status.WAITING;
            }
        }

        public bool IsTimedOut()
        {
            lock (requestLock)
            {
                return status == Status.TIMEDOUT;
            }
        }

        public DateTime GetSentTime()
        {
            return sentTime;
        }

        // Asynchronously wait until the request is no longer in WAITING state
        public async Task<Status> WaitAsync(int pollIntervalMs = 5)
        {
            while (IsWaiting())
            {
                await Task.Delay(pollIntervalMs);
            }
            return GetStatus();
        }
    }

    // Current endianness (default: BigEndian)
    private Endianness endianness = Endianness.BigEndian;
    // Current write mode (default: PACKET)
    private WriteMode writeMode = WriteMode.BULK;

    // Buffer for outgoing data
    private List<byte> writeBuffer = new List<byte>();

    // Queue for request objects to track requests
    private Queue<Request> requestQueue = new Queue<Request>();

    // Request timeout in milliseconds (default: 500ms)
    private int requestTimeoutMs = 500;

    // Dictionary of expected response length for each request
    private Dictionary<byte, int> responseLengths = new Dictionary<byte, int>();

    // Classes for JSON unpacking
    [Serializable]
    public class ResponseLengthEntry
    {
        public byte header;
        public int length;
        // Note length of -1 allows self-declaring variable length response where the byte following the header is assumed to be length declaration
    }

    [Serializable]
    public class ResponseLengthList
    {
        public List<ResponseLengthEntry> headers;
    }

    // Whether a packet is currently being processed
    private bool packetFlag = false;

    // Buffer for incoming data
    private List<byte> readBuffer = new List<byte>();

    // Handler to process incoming data
    private Action<MinBiTTcpClient, Request> readHandler = null;

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
    public void SetReadHandler(Action<MinBiTTcpClient, Request> readHandler)
    {
        this.readHandler = readHandler;
    }

    // Set the request timeout (in milliseconds)
    public void SetRequestTimeout(int timeoutMs)
    {
        requestTimeoutMs = timeoutMs;
    }

    // Loads reponse lengths for each request
    public void LoadResponseLengthsFromJson(string jsonText)
    {
        // Uses dictionary entry classes to import from JSON
        ResponseLengthList list = JsonUtility.FromJson<ResponseLengthList>(jsonText);
        if (list != null && list.headers != null)
        {
            // Clears responseLengths list
            responseLengths.Clear();
            // Adds new entries
            foreach (var entry in list.headers)
            {
                responseLengths[entry.header] = entry.length;
            }
        }
        else
        {
            Debug.LogError("Failed to parse response lengths JSON.");
        }
    }

    // Thread method: listens for incoming data and processes server responses
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
                }

                // Timeout check: remove requests that have timed out
                lock (dataLock)
                {
                    if (requestQueue.Count > 0)
                    {
                        var req = requestQueue.Peek();
                        if ((DateTime.UtcNow - req.GetSentTime()).TotalMilliseconds > requestTimeoutMs)
                        {
                            Debug.LogWarning($"Request with header {req.GetHeader()} timed out after {requestTimeoutMs} ms.");
                            req.SetStatus(Request.Status.TIMEDOUT);
                            clearRequest();
                            flush();

                            // Calls read handler
                            if (readHandler != null)
                            {
                                readHandler(this, req);
                            }
                        }
                    }
                }

                // Process packets only when enough data is available
                while (getReadBufferSize() > 0 && getRequestQueueSize() > 0)
                {
                    // Packet is being processed
                    if (!packetFlag)
                    {
                        packetFlag = true;
                    }

                    // Gets current request
                    if (!GetCurrentRequest(out Request request))
                    {
                        Debug.LogError("Request queue cleared unexpectedly");
                        flush();
                        break;
                    }

                    // Can't process new request until old ones have been cleared
                    if (!request.IsWaiting())
                    {
                        break;
                    }

                    // Determine expected response length for this request header
                    if (!GetExpectedResponseLength(request.GetHeader(), out int expectedLength))
                    {
                        Debug.LogError($"No response length found for request header {request.GetHeader()}");
                        clearRequest();
                        flush();
                        break;
                    }

                    // Gets packet length parameters
                    if (!getPacketParameters(expectedLength, out int payloadLength, out int totalPacketLength))
                    {
                        // Waits until able to access all packet parameters
                        break;
                    }

                    // Wait until the full packet is available
                    if (getReadBufferSize() < totalPacketLength)
                    {
                        break;
                    }

                    // Now we have the full packet, so process it
                    request.SetResponseHeader(readByte()); // Sets response header

                    // If variable length, remove the length byte as well
                    if (expectedLength == -1)
                    {
                        readByte();
                    }

                    // Set payload length
                    request.SetResponseLength(payloadLength);

                    // Request has been fufilled
                    request.SetStatus(Request.Status.FULFILLED);

                    // Calls read handler if exists
                    if (readHandler != null)
                    {
                        readHandler(this, request);
                        //Clears request from queue
                        clearRequest();
                    }
                }
                // Flushes data not associated with request
                if (getReadBufferSize() > 0)
                {
                    flush();
                }
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
        }
    }
    
    // Whether a packet is currently being processed
    public bool isPacketPending()
    {
        return packetFlag;
    }

    // Gets response length for request header
    public bool GetExpectedResponseLength(byte requestHeader, out int expectedLength)
    {
        return responseLengths.TryGetValue(requestHeader, out expectedLength);
    }

    // Gets packet length parameters
    public bool getPacketParameters(int expectedLength, out int payloadLength, out int totalPacketLength)
    {
        payloadLength = 0;
        totalPacketLength = 1; // 1 byte for response header

        if (expectedLength == -1)
        {
            // Need at least 2 bytes: header + length byte
            if (getReadBufferSize() < 2)
            {
                return false;
            }
            byte lengthByte;
            lock (dataLock)
            {
                lengthByte = readBuffer[1];
            }
            // Sets expected payload length
            payloadLength = lengthByte;
            // Adds to total packet length
            totalPacketLength += 1 + payloadLength; // header + length byte + payload
        }
        else
        {
            // Updates expected payload length
            payloadLength = expectedLength;
            totalPacketLength += expectedLength;
        }
        return true;
    }

    // Returns the size of the read buffer
    public int getReadBufferSize()
    {
        lock (dataLock)
        {
            return readBuffer.Count;
        }
    }

    // Returns current request header
    public bool GetCurrentRequest(out Request request)
    {
        request = null;
        lock (dataLock)
        {
            if (requestQueue.Count > 0)
            {
                request = requestQueue.Peek();
                return true;
            }
            else
            {
                return false;
            }
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
                // Starts current request
                if (!GetCurrentRequest(out Request request))
                {
                    Debug.LogError($"Failed to write packet: no header present");
                    return;
                }
                request.Start();

                stream.Write(writeBuffer.ToArray(), 0, writeBuffer.Count);
                writeBuffer.Clear();
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to write packet: {ex.Message}");
            }
        }
    }

    // Add a single byte to the write buffer
    public void writeByte(byte value)
    {
        writeBytes(new byte[] { value });
    }

    // Add a header byte to the write buffer and queue
    public Request writeHeader(byte value)
    {
        writeByte(value);
        Request request = new Request(value);
        lock (dataLock)
        {
            requestQueue.Enqueue(request);
        }

        return request;
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

    // Clear the current request from the queue and mark as ended
    public bool clearRequest()
    {
        lock (dataLock)
        {
            packetFlag = false;
            if (requestQueue.Count > 0)
            {
                requestQueue.Dequeue();
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    // Clear the read buffer
    public void flush()
    {
        lock (dataLock)
        {
            readBuffer.Clear();
        }
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