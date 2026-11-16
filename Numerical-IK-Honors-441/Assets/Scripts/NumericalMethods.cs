using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using MathNet.Numerics;
using System;
using System.Linq;
using UnityEngine.Events;

public class NumericalMethods : MonoBehaviour
{
    public enum NumericalMethod
    {
        CyclicCoordinateDescent,
        Inverse,
        Transpose,
        Pseudoinverse,
        DampenedLeastSquares
    }

    [SerializeField] private NumericalMethod numerical_method;
    [SerializeField] private List<GameObject> ee_objs;
    [SerializeField] private List<GameObject> target_objs;
    [SerializeField] private List<GameObject> joint_objs;
    [SerializeField] private List<RevoluteJoint> joint_rjs;
    
    // Matrix builder shortcuts.
    private MatrixBuilder<float> M = Matrix<float>.Build;
    private VectorBuilder<float> V = Vector<float>.Build;
    private Matrix<float> J; // Jacobian matrix
    private Matrix<float> JPS; // Jacobian Pseudoinverse
    private Matrix<float> I; // Identity
    private Vector<float> t; // Target positions
    private Vector<float> s; // End effector positions
    private Vector<float> e; // Error between targets and end effectors
    private Vector<float> da; // Delta joint angles

    public ArmAssembler arm_assembler_script;
    private bool begin_updates;

    // Scalars
    private float alpha_tr;
    private float alpha_in;
    [SerializeField] private float alpha_ps = 0.1f;
    [SerializeField] private float lambda = 0.1f;
    void Awake()
    {
        begin_updates = false;
        arm_assembler_script.joints_ready.AddListener(Setup);
    }
    void OnDestroy()
    {
        arm_assembler_script.joints_ready.RemoveListener(Setup);
    }

    void Setup()
    {
        joint_objs = GameObject.FindGameObjectsWithTag("Joint").ToList();
        ee_objs = GameObject.FindGameObjectsWithTag("End-Effector").ToList();
        target_objs = GameObject.FindGameObjectsWithTag("Target").ToList();

        // Sort the joints from deepest (closest to end effector) to shallowest (closest to base).
        joint_objs = Ext.SortByDepth(joint_objs);

        // Create a list of the joints' angle-controlling scripts for easy reference.
        joint_rjs = new List<RevoluteJoint>();
        foreach(GameObject joint_obj in joint_objs)
        {
            if (joint_obj.TryGetComponent(out RevoluteJoint rj))
            {
                joint_rjs.Add(rj);
            }
        }

        J = M.Dense(3 * ee_objs.Count, joint_objs.Count);
        I = M.DenseIdentity(J.RowCount);
        e = V.Dense(3 * ee_objs.Count);
        s = V.Dense(3 * ee_objs.Count);
        t = V.Dense(3 * ee_objs.Count);
        da = V.Dense(joint_objs.Count);

        Debug.Log($"J: {J.RowCount} x {J.ColumnCount}");
        begin_updates = true;
    }

    void FixedUpdate()
    {
        if (begin_updates)
        {
            UpdateVectors();
            UpdateJacobian();

            if(numerical_method == NumericalMethod.CyclicCoordinateDescent)
            {
                PerformCCD();
            }
            else if(numerical_method == NumericalMethod.Inverse)
            {
                CalculateAlpha();
                UpdateDeltaAnglesIN();
                ApplyDeltaAngles();
            }
            else if(numerical_method == NumericalMethod.Transpose)
            {
                CalculateAlpha();
                UpdateDeltaAnglesTR();
                ApplyDeltaAngles();
            }
            else if(numerical_method == NumericalMethod.Pseudoinverse)
            {
                CalculateConugateTranspose();
                UpdateDeltaAnglesPS();
                ApplyDeltaAngles();
            }
            else
            {
                UpdateDeltaAnglesDLS();
                ApplyDeltaAngles();
            }
        }
    }

    private void PerformCCD()
    {
        foreach(RevoluteJoint joint_rj in joint_rjs)
        {
            GameObject joint_obj = joint_rj.gameObject;
            Vector3 joint_effector_v = (ee_objs[0].transform.position - joint_obj.transform.position).normalized;
            Vector3 joint_target_v = (target_objs[0].transform.position - joint_obj.transform.position).normalized;

            float z_angle = Vector3.SignedAngle(joint_effector_v, joint_target_v, joint_rj.localZAxis);
            joint_rj.AddAngle(z_angle);
        }
    }

    /// <summary>
    /// Updates the vectors for error (i) = target position (i) - end effector position (i).
    /// </summary>
    private void UpdateVectors()
    {
        int c = 0;
        for(int ee_i = 0; ee_i < ee_objs.Count; ee_i++)
        {
            s[c] = ee_objs[ee_i].transform.position.x;
            s[c + 1] = ee_objs[ee_i].transform.position.y;
            s[c + 2] = ee_objs[ee_i].transform.position.z;

            t[c] = target_objs[ee_i].transform.position.x;
            t[c + 1] = target_objs[ee_i].transform.position.y;
            t[c + 2] = target_objs[ee_i].transform.position.z;

            e[c] = t[c] - s[c];
            e[c + 1] = t[c + 1] - s[c + 1];
            e[c + 2] = t[c + 2]- s[c + 2];

            c += 3;
        }
    }

    /// <summary>
    /// Computes each partial derivative of end effector position with respect to joint angle.
    /// </summary>
    /// <param name="ee_position"></param>
    /// <param name="unit_joint_axis"></param>
    /// <param name="joint_position"></param>
    /// <returns></returns>
    private void UpdateJacobian()
    {
        for(int r = 0; r < J.RowCount; r += 3) // Per-vector
        {
            for(int c = 0; c < J.ColumnCount; c++) // Per-angle
            {
                Vector3 ee_pos = ee_objs[r / 3].transform.position;
                Vector3 joint_pos = joint_objs[c].transform.position;
                Vector3 unit_joint_axis = joint_rjs[c].transform.forward;

                Vector3 result = Vector3.Cross(unit_joint_axis, ee_pos - joint_pos);

                J[r, c] = result.x;
                J[r + 1, c] = result.y;
                J[r + 2, c] = result.z;
            }
        }
    }

    /// <summary>
    /// Calculates the scalar approximation a.
    /// </summary>
    private void CalculateAlpha()
    {
        Vector<float> JJTe = J * J.Transpose() * e;
        float numerator = e.DotProduct(JJTe);
        float denominator = JJTe.DotProduct(JJTe);

        if(denominator.AlmostEqual(0))
        {
            alpha_tr = alpha_in = 0;
        }
        else
        {
            alpha_tr = alpha_in = numerator / denominator;
        }
    }

    private void ApplyDeltaAngles()
    {
        for(int i = 0; i < joint_rjs.Count; i++)
        {
            joint_rjs[i].AddAngle(da[i] * Mathf.Rad2Deg);
        }
    }

    // =============================================================================
    // INVERSE
    // =============================================================================
    private void UpdateDeltaAnglesIN()
    {
        if(!J.Determinant().AlmostEqual(0))
        {
            da = alpha_in * J.Inverse() * e;
        }
        else
        {
            throw new Exception($"Jacobian is not invertible! Rank is {J.Rank()}, J = {J.ToString()}.");
        }
    }

    // =============================================================================
    // TRANSPOSE
    // =============================================================================
    private void UpdateDeltaAnglesTR()
    {
        da = alpha_tr * J.Transpose() * e;
    }

    // =============================================================================
    // PSEUDOINVERSE
    // =============================================================================
    private void CalculateConugateTranspose()
    {
        JPS = J.PseudoInverse();
    }
    private void UpdateDeltaAnglesPS()
    {
        da = alpha_ps * JPS * e;
    }

    // =============================================================================
    // DAMPENED LEAST SQUARES
    // =============================================================================

    private void UpdateDeltaAnglesDLS()
    {
        Matrix<float> JT = J.Transpose(); 
        Matrix<float> A = J * JT + Mathf.Pow(lambda, 2) * I;
        Vector<float> x = A.Solve(e);
        da = JT * x;
    }
}
