using UnityEngine;
using UnityEngine.Events;

public class ArmAssembler : MonoBehaviour
{
    public GameObject target_obj;
    [Min(1)] public int joint_count;
    [Min(0.01f)] public float limb_length = 3f;
    public GameObject intermediate_joint_prefab;
    public GameObject end_joint_prefab;
    public UnityEvent joints_ready;

    void Start()
    {
        int count = Mathf.Max(1, joint_count);
        float length = Mathf.Max(0.01f, limb_length);
        Transform previous_joint = null;

        for (int i = 0; i < count; i++)
        {
            bool is_end = i == count - 1;
            GameObject prefab = is_end ? end_joint_prefab : intermediate_joint_prefab;
            Transform joint = Instantiate(prefab, previous_joint).transform;

            // Keep the base at the assembler and space descendants in their parent's frame.
            // Never scale a joint: its sphere and all descendant joints keep their size.
            if (previous_joint == null)
                joint.position = transform.position;
            else
                joint.localPosition = Vector3.right * length;

            Transform limb = joint.Find("Limb");
            limb.localPosition = Vector3.right * (length * 0.5f);
            Vector3 limb_scale = limb.localScale;
            limb_scale.x = length;
            limb.localScale = limb_scale;

            if (is_end)
                joint.Find("End-Effector").localPosition = Vector3.right * length;

            previous_joint = joint;
        }

        target_obj.transform.position = new Vector3(joint_count * length * 0.5f, 0, 0);

        joints_ready.Invoke();
    }
}
