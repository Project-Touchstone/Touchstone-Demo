using UnityEngine;

public class HapticShadow : MonoBehaviour
{
    // Haptic node script
    public HapticNode node;

    // Self rigid body
    public Rigidbody shadowRigidbody;

    // Self collider
    public Collider shadowCollider;

    // Collision candidate object
    private CollisionCandidate currCandidate = new CollisionCandidate();

    // Collision candidate class
    public class CollisionCandidate
    {
        // Collision point
        private Vector3 collisionPoint = Vector3.zero;
        // Collision normal vecotr
        private Vector3 collisionNormal = Vector3.zero;
        // Net momentum change vector
        private Vector3 momentumChange = Vector3.zero;
        // Time until collision
        private float timeUntilCollision = 0f;

        public CollisionCandidate()
        {
        }
        // Copy constructor
        public CollisionCandidate(CollisionCandidate other) : this(other.getCollisionPoint(), other.getCollisionNormal(), other.getMomentumChange(), other.getTimeUntilCollision())
        {
        }
        public CollisionCandidate(Vector3 collisionPoint, Vector3 collisionNormal, Vector3 momentumChange, float timeUntilCollision)
        {
            this.collisionPoint = collisionPoint;
            this.collisionNormal = collisionNormal;
            this.momentumChange = momentumChange;
            this.timeUntilCollision = timeUntilCollision;
        }

        public static bool fromRaycast(Collider self, Collider other, out CollisionCandidate result)
        {
            // Gets velocity of self relative to other object
            Vector3 selfVel = self.attachedRigidbody.linearVelocity;
            Vector3 otherVel = Vector3.zero;
            if (other.attachedRigidbody != null)
            {
                otherVel = other.attachedRigidbody.linearVelocity;
            }
            Vector3 relVel = selfVel - otherVel;

            if (relVel.normalized.magnitude == 0)
            {
                // No relative velocity, no collision
                result = null;
                return false;
            }

            // Raycasts from self object center against other object
            RaycastHit selfToOther;
            bool detected = other.Raycast(new Ray(self.bounds.center, relVel.normalized), out selfToOther, Mathf.Infinity);

            if (!detected)
            {
                result = null;
                return false;
            }

            // Reverse raycast from contact point against self object
            RaycastHit contactToSelf;
            detected = self.Raycast(new Ray(selfToOther.point, -relVel.normalized), out contactToSelf, Mathf.Infinity);

            if (!detected)
            {
                result = null;
                return false;
            }

            // Gets time until collision
            float timeUntilCollision = contactToSelf.distance / relVel.magnitude;
            
            // Gets object position at collision by projecting along self velocity
            Vector3 collisionPoint = selfVel * timeUntilCollision;

            // Finds global collision normal
            Vector3 collisionNormal = selfToOther.normal;

            // Gets mass
            float selfMass = self.attachedRigidbody.mass;

            Vector3 momentumChange;
            if (other.attachedRigidbody == null)
            {
                // Treats other object as immovable
                momentumChange = 2 * selfMass * Vector3.Dot(relVel, collisionNormal) * collisionNormal;
            }
            else
            {
                // Otherwise does elastic collision
                float otherMass = other.attachedRigidbody.mass;

                // Finds initial velocities relative to collision plane
                float initSelfVel = Vector3.Dot(selfVel, collisionNormal);
                float initOtherVel = Vector3.Dot(otherVel, collisionNormal);

                // Computes momentum transfer after elastic collision
                float finalSelfVel = (2 * otherMass * initOtherVel + (selfMass - otherMass) * initSelfVel) / (selfMass + otherMass);
                momentumChange = selfMass * (finalSelfVel - initSelfVel) * collisionNormal;
            }

            // Creates collision candidate
            result = new CollisionCandidate(collisionPoint, collisionNormal, momentumChange, timeUntilCollision);
            return true;
        }

        public float getTimeUntilCollision()
        {
            return timeUntilCollision;
        }

        public Vector3 getCollisionPoint()
        {
            return collisionPoint;
        }

        public Vector3 getCollisionNormal()
        {
            return collisionNormal;
        }

        public Vector3 getMomentumChange()
        {
            return momentumChange;
        }

        public bool isValid()
        {
            return momentumChange.magnitude > 0;
        }

        public int compareTo(CollisionCandidate other)
        {
            if (!isValid())
            {
                // If self is invalid, other is more urgent
                return -1;
            }
            else if (!other.isValid())
            {
                // If other is invalid, self is more urgent
                return 1;
            }

            // Compares times until collision
            float time = getTimeUntilCollision();
            float otherTime = other.getTimeUntilCollision();

            if (otherTime >= 2 * time)
            {
                // If other is at least twice as far, self is more urgent
                return 1;
            }
            else if (time >= 2 * otherTime)
            {
                // If self is at least twice as far, other is more urgent
                return -1;
            }
            else
            {
                // Otherwise they are equally urgent
                return 0;
            }
        }

        public void combineWith(CollisionCandidate other)
        {
            // Adds momentum change
            momentumChange += other.getMomentumChange();

            // Uses closest time and corresponding collision point
            if (other.getTimeUntilCollision() < getTimeUntilCollision())
            {
                timeUntilCollision = other.getTimeUntilCollision();
                collisionPoint = other.getCollisionPoint();
            }
        }
    }
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        shadowCollider = this.GetComponent<Collider>();
        shadowRigidbody = this.GetComponent<Rigidbody>();
    }

    public void SetHapticNode(HapticNode node)
    {
        this.node = node;
    }

    void FixedUpdate()
    {
        // Updates haptic node with current collision candidate after all collisions are evaluated
        node.UpdateCollisionCandidate(currCandidate);
        currCandidate = new CollisionCandidate();
    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnTriggerEnter(Collider other)
    {
        evaluateTrigger(other);
    }

    void OnTriggerStay(Collider other)
    {
        evaluateTrigger(other);
    }

    void evaluateTrigger(Collider other) {
        //Gets collision candidate from raycast
        if (other.isTrigger)
        {
            // Ignores triggers
            return;
        }
        CollisionCandidate candidate;
        if (CollisionCandidate.fromRaycast(shadowCollider, other, out candidate))
        {
            // Determines combination behavior
            int result = currCandidate.compareTo(candidate);
            if (result < 0)
            {
                // Replaces current candidate with more urgent one
                currCandidate = candidate;
            }
            else if (result == 0)
            {
                // Combines with current candidate
                currCandidate.combineWith(candidate);
            }
            //Otherwise keeps more urgent current candidate
        }
    }
}
