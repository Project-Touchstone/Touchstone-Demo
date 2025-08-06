using UnityEngine;

public class TestObject : MonoBehaviour
{

    [SerializeField] private float force = 0.001f; // Force applied to the object
    Rigidbody rb;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Gets rigidbody
        rb = this.GetComponent<Rigidbody>();
        rb.useGravity = false;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Adds force to local up direction
            rb.AddForce(transform.up * force, ForceMode.Impulse);
        }
    }
}
