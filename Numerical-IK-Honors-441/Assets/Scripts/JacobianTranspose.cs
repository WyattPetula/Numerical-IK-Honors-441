using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using MathNet.Numerics;
using System;

public class JacobianTranspose : MonoBehaviour
{
    public enum JacobianMethod
    {
        Transpose,
        Pseudoinverse,
        DampenedLeastSquares
    }

    public Transform target_t;
    public Transform end_effector_t;

    [SerializeField] private JacobianMethod jacobian_method;
    [SerializeField] private List<GameObject> ee_objs; // Set in editor
    [SerializeField] private List<GameObject> target_objs; // Set in editor
    [SerializeField] private List<GameObject> joint_objs; // Set in editor
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

    // Scalars
    private float alpha_tr;
    [SerializeField] private float alpha_ps = 0.1f;
    [SerializeField] private float lambda = 0.1f;

    void Start()
    {
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
    }

    void FixedUpdate()
    {
        UpdateVectors();
        UpdateJacobian();

        if(jacobian_method == JacobianMethod.Transpose)
        {
            CalculateAlpha();
            UpdateDeltaAnglesTR();
        }
        else if(jacobian_method == JacobianMethod.Pseudoinverse)
        {
            CalculateConugateTranspose();
            UpdateDeltaAnglesPS();
        }
        else
        {
            UpdateDeltaAnglesDLS();
        }

        ApplyDeltaAngles();
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
            alpha_tr = 0;
        else
            alpha_tr = numerator / denominator;
    }

    private void ApplyDeltaAngles()
    {
        for(int i = 0; i < joint_rjs.Count; i++)
        {
            joint_rjs[i].AddAngle(da[i] * Mathf.Rad2Deg);
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
