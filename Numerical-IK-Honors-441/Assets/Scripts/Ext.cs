using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public static class Ext
{
    // =============================================================================
    // MATRICES
    // =============================================================================

    /// <summary>
    /// Create a (k x n) matrix.
    /// </summary>
    /// <param name="k"></param>
    /// <param name="n"></param>
    /// <returns></returns>
    public static float[][] CreateMatrix(int k, int n)
    {
        float[][] matrix = new float[k][];
        for(int i = 0; i < k; i++)
        {
            matrix[i] = new float[n];
        }

        return matrix;
    }



    // =============================================================================
    // NESTING & CHILDREN
    // =============================================================================

    /// <summary>
    /// Calculate the nesting depth (e.g, Parent |_ Target is 1) of a transform.
    /// </summary>
    /// <param name="transform"></param>
    /// <returns></returns>
    public static int GetNestingDepth(this Transform transform)
    {
        int depth = 0;
        while (transform.parent != null)
        {
            depth++;
            transform = transform.parent;
        }
        return depth;
    }

    /// <summary>
    /// Sort a list of GameObjects by nesting depth.
    /// </summary>
    /// <param name="objects"></param>
    /// <param name="small_to_large"></param>
    /// <returns></returns>
    public static List<GameObject> SortByDepth(List<GameObject> objects, bool small_to_large=false)
    {
        if(small_to_large)
        {
            return objects
                .OrderBy(obj => obj.transform.GetNestingDepth())
                .ToList();
        }
        else
        {
            return objects
                .OrderByDescending(obj => obj.transform.GetNestingDepth())
                .ToList();
        }
    }
}
