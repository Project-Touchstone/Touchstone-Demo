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
            INCOMING,
            OUTGOING,
            COMPLETE,
            TIMEDOUT
        }

        private static long nextId = 1;
        private long id;
        private byte header;
        private byte responseHeader;
        private int payloadLength;
        private Status status;
        private DateTime sentTime;

        // Mutex for thread safety
        private readonly object requestLock = new object();

        public Request(byte header, Status status)
        {
            this.header = header;
            this.responseHeader = 0;
            this.payloadLength = -1;
            this.status = status;
            this.id = Interlocked.Increment(ref nextId);
        }

        public void Start()
        {
            sentTime = DateTime.UtcNow;
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

        public void SetPayloadLength(int responseLength)
        {
            lock (requestLock)
            {
                this.payloadLength = responseLength;
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
                return payloadLength;
            }
        }

        public bool IsIncoming()
        {
            lock (requestLock)
            {
                return status == Status.INCOMING;
            }
        }

        public bool IsOutgoing()
        {
            lock (requestLock)
            {
                return status == Status.OUTGOING;
            }
        }

        public bool IsWaiting()
        {
            return IsIncoming() || IsOutgoing();
        }

        public bool IsComplete()
        {
            lock (requestLock)
            {
                return status == Status.COMPLETE;
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
    // Current write mode (default: BULK)
    private WriteMode writeMode = WriteMode.BULK;

    // Buffer for outgoing data
    private List<byte> writeBuffer = new List<byte>();

    // Queue for unsent requests
    private Queue<Request> unsentRequests = new Queue<Request>();
    // Queue for outgoing requests
    private Queue<Request> outgoingRequests = new Queue<Request>();
    // Current request
    private Request currRequest;

    // Request timeout in milliseconds (default: 500ms)
    private int requestTimeoutMs = 500;

    // Dictionary of outgoing packet lengths by request header
    private Dictionary<byte, int> outgoingByRequest = new Dictionary<byte, int>();
    // Dictionary of outgoing packet lengths by response header
    private Dictionary<byte, int> outgoingByResponse = new Dictionary<byte, int>();
    // Dictionary of incoming packet lengths by request header
    private Dictionary<byte, int> incomingByRequest = new Dictionary<byte, int>();

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
        public List<ResponseLengthEntry> outgoingByRequest;
        public List<ResponseLengthEntry> outgoingByResponse;
        public List<ResponseLengthEntry> incomingByRequest;
    }

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
        if (list == null)
        {
            Debug.LogError("Failed to parse response lengths JSON.");
            return;
        }
        if (list.outgoingByRequest != null)
        {
            // Clears by request list
            outgoingByRequest.Clear();
            // Adds new entries
            foreach (var entry in list.outgoingByRequest)
            {
                outgoingByRequest[entry.header] = entry.length;
            }
        }
        if (list.outgoingByResponse != null)
        {
            // Clears by request list
            outgoingByResponse.Clear();
            // Adds new entries
            foreach (var entry in list.outgoingByResponse)
            {
                outgoingByResponse[entry.header] = entry.length;
            }
        }
        if (list.incomingByRequest != null) {
            // Clears by request list
            incomingByRequest.Clear();
            // Adds new entries
            foreach (var entry in list.incomingByRequest)
            {
                incomingByRequest[entry.header] = entry.length;
            }
        }
    }

    private void checkForTimeouts()
    {
        if (outgoingRequests.Count > 0)
        {
            GetOutgoingRequest(out Request req);
            if ((DateTime.UtcNow - req.GetSentTime()).TotalMilliseconds > requestTimeoutMs)
            {
                Debug.LogWarning($"Outgoing request with header {req.GetHeader()} timed out after {requestTimeoutMs} ms.");
                req.SetStatus(Request.Status.TIMEDOUT);
                if (currRequest != null && currRequest.IsOutgoing())
                {
                    clearRequest();
                }
                else
                {
                    lock (dataLock)
                    {
                        outgoingRequests.Dequeue();
                    }
                }
                flush();
                // Calls read handler
                if (readHandler != null)
                {
                    readHandler(this, req);
                }
            }
        }
    }

    private bool characterizePacket(out bool variableLength, out int payloadLength)
    {
        //Assigns default values
        variableLength = false;
        payloadLength = 0;

        // Peeks header
        byte receivedHeader = peekByte();

        // If request has not been created
        if (currRequest == null) {
            // Checks for outgoing request response header
            if (getNumOutgoingRequests() > 0) {
                if (outgoingByResponse.TryGetValue(receivedHeader, out int length)) {
                    // Assigns to current outgoing request
                    GetOutgoingRequest(out currRequest);
                    // Sets reponse header
                    currRequest.SetResponseHeader(receivedHeader);
                }
            }
            else {
                //Otherwise create new incoming request
                // Creates new incoming request
                currRequest = new Request(receivedHeader, Request.Status.INCOMING);
            }
        }

        // Only process requests that have not yet been fufilled
        if (!currRequest.IsWaiting())
        {
            return false;
        }

        // Determine expected response length for this request
        if (!GetExpectedResponseLength(currRequest, out int expectedLength))
        {
            Debug.LogError($"No response length found for request header {currRequest.GetHeader()}");
            clearRequest();
            flush();
            return false;
        }

        // Gets packet length parameters
        if (!getPacketParameters(expectedLength, out payloadLength, out int totalPacketLength))
        {
            // Waits until able to access all packet parameters
            return false;
        }

        // Wait until the full packet is available
        if (getReadBufferSize() < totalPacketLength)
        {
            return false;
        }

        // Set payload length
        currRequest.SetPayloadLength(payloadLength);
        variableLength = (expectedLength == -1);
        return true;
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

                // Process packets only when enough data is available
                while (getReadBufferSize() > 0)
                {
                    if (!characterizePacket(out bool variableLength, out int payloadLength))
                    {
                        break;
                    }

                    // Now we have the full packet, so process it
                    readByte(); // Removes response header

                    // If variable length, remove the length byte as well
                    if (variableLength)
                    {
                        readByte();
                    }

                    // Calls read handler if exists
                    if (readHandler != null)
                    {
                        readHandler(this, currRequest);
                        //Clears request from queue
                        clearRequest();
                    }

                    // Request has been fufilled
                    currRequest.SetStatus(Request.Status.COMPLETE);

                    
                }

                // Timeout check: remove requests that have timed out
                checkForTimeouts();
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
        }
    }

    // Gets response length for request header
    public bool GetExpectedResponseLength(Request request, out int expectedLength)
    {
        expectedLength = 0;
        // Checks whether request is incoming or outgoing
        if (request.IsOutgoing())
        {
            // Lengths by response header are the priority
            if (outgoingByResponse.TryGetValue(request.GetResponseHeader(), out expectedLength))
            {
                return true;
            }

            // Otherwise searches by request header
            if (outgoingByRequest.TryGetValue(request.GetHeader(), out expectedLength))
            {
                return true;
            }
        }
        else if (request.IsIncoming())
        {
            // Searches incoming requests by request header
            if (incomingByRequest.TryGetValue(request.GetHeader(), out expectedLength))
            {
                return true;
            }
        }
        // Otherwise fails
        return false;
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
    public bool GetOutgoingRequest(out Request request)
    {
        request = null;
        lock (dataLock)
        {
            if (outgoingRequests.Count > 0)
            {
                request = outgoingRequests.Peek();
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

    public byte peekByte()
    {
        lock (dataLock)
        {
            return readBuffer[0];
        }
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
    public void sendAll()
    {
        if (stream != null)
        {
            try
            {
                lock (dataLock)
                {
                    // Starts unsent requests (if there are any)
                    for (int i = 0; i < unsentRequests.Count; i++)
                    {
                        // Dequeues from unsent requests
                        Request request = unsentRequests.Dequeue();
                        // Starts request
                        request.Start();
                        // Adds to outgoing request queue
                        outgoingRequests.Enqueue(request);
                    }
                }

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
    public Request writeRequest(byte value)
    {
        Request request = new Request(value, Request.Status.OUTGOING);
        lock (dataLock)
        {
            // Adds to unsent requests
            unsentRequests.Enqueue(request);
        }

        // Writes header
        writeByte(value);

        return request;
    }

    // Returns the number of pending requests in the queue
    public int getNumOutgoingRequests()
    {
        lock (dataLock)
        {
            return outgoingRequests.Count;
        }
    }

    // Add a byte array to the write buffer (and write immediately if in IMMEDIATE mode)
    public void writeBytes(byte[] buffer)
    {
        writeBuffer.AddRange(buffer);

        // writes packet immediately if in immediate mode
        if (writeMode == WriteMode.IMMEDIATE)
        {
            sendAll();
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
            if (currRequest != null) {
                // Removes current request from queue if outgoing
                if (currRequest.IsOutgoing() && outgoingRequests.Count > 0) {
                    outgoingRequests.Dequeue();
                }
                // Clears current request
                currRequest = null;

                return true;
            }
            return false;
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