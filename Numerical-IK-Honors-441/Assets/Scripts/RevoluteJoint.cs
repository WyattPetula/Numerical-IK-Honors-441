using UnityEngine;

public class RevoluteJoint : MonoBehaviour
{
    [Header("Joint Configuration")]
    public Vector3 localZAxis = Vector3.forward;

    [Header("Angular Limits (Degrees)")]
    public float minAngle = -180f;
    public float maxAngle = 180f;

    [Header("Current Angle")]
    public float currentAngle = 0f;

    private Quaternion startRotQ;
    void Awake()
    {
        startRotQ = transform.localRotation;
        SetAngle(currentAngle);
    }
    public void SetAngle(float newAngle)
    {
        currentAngle = Mathf.Clamp(newAngle, minAngle, maxAngle);
        transform.localRotation = startRotQ * Quaternion.AngleAxis(currentAngle, localZAxis.normalized);
    }
}
