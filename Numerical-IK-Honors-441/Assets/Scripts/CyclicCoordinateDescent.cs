using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class CyclicCoordinateDescent : MonoBehaviour
{
    public bool solve_base_to_end = false;
    public Transform target_t;
    public Transform end_effector_t;
    [SerializeField] private List<GameObject> joint_objs;
    [SerializeField] private List<RevoluteJoint> joint_rjs;
    void Start()
    {
        // Get a list of all the joints in the scene.
        joint_objs = GameObject.FindGameObjectsWithTag("Joint").ToList();

        // Sort the joints from deepest (closest to end effector) to shallowest (closest to base).
        joint_objs = Ext.SortByDepth(joint_objs, solve_base_to_end);

        // Create a list of the joints' angle-controlling scripts for easy reference.
        foreach(GameObject joint_obj in joint_objs)
        {
            if (joint_obj.TryGetComponent(out RevoluteJoint rj))
            {
                joint_rjs.Add(rj);
            }
        }
    }

    void FixedUpdate()
    {
        foreach(RevoluteJoint joint_rj in joint_rjs)
        {
            GameObject joint_obj = joint_rj.gameObject;
            Vector3 joint_effector_v = (end_effector_t.position - joint_obj.transform.position).normalized;
            Vector3 joint_target_v = (target_t.position - joint_obj.transform.position).normalized;

            float z_angle = Vector3.SignedAngle(joint_effector_v, joint_target_v, joint_rj.localZAxis);
            joint_rj.AddAngle(z_angle);
        }
    }
}
