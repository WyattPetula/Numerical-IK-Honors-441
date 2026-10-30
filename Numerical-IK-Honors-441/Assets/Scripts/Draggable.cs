
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Collider))]
public class Draggable : MonoBehaviour
{
    public enum DragPlane
    {
        XY,
        XZ,
        YZ
    }

    [Header("Drag Settings")]
    public DragPlane drag_plane_enum = DragPlane.XY;

    [SerializeField] private Camera mainCamera;

    private Plane drag_plane;
    private Vector3 offset;
    private bool isDragging = false;

    void Awake()
    {
        if (mainCamera == null)
            mainCamera = Camera.main;
    }

    void Update()
    {
        Mouse mouse = Mouse.current;

        if (mouse == null || mainCamera == null)
            return;

        // Switch between drag planes.
        if (Keyboard.current != null)
        {
            if (Keyboard.current.digit1Key.wasPressedThisFrame)
                drag_plane_enum = DragPlane.XY;

            if (Keyboard.current.digit2Key.wasPressedThisFrame)
                drag_plane_enum = DragPlane.XZ;

            if (Keyboard.current.digit3Key.wasPressedThisFrame)
                drag_plane_enum = DragPlane.YZ;
        }

        // Read mouse position using the new Input System.
        Vector2 mousePosition = mouse.position.ReadValue();

        Ray ray = mainCamera.ScreenPointToRay(mousePosition);

        // Begin dragging.
        if (mouse.leftButton.wasPressedThisFrame)
        {
            if (Physics.Raycast(ray, out RaycastHit hit) &&
                (hit.transform == transform ||
                 hit.transform.IsChildOf(transform)))
            {
                StartDrag(ray);
            }
        }

        // Continue dragging.
        if (isDragging && mouse.leftButton.isPressed)
        {
            Drag(ray);
        }

        // Stop dragging.
        if (mouse.leftButton.wasReleasedThisFrame)
        {
            isDragging = false;
        }
    }

    void StartDrag(Ray ray)
    {
        Vector3 planeNormal = GetPlaneNormal();

        drag_plane = new Plane(
            planeNormal,
            transform.position
        );

        if (drag_plane.Raycast(ray, out float enter))
        {
            offset = transform.position - ray.GetPoint(enter);
            isDragging = true;
        }
    }

    void Drag(Ray ray)
    {
        if (drag_plane.Raycast(ray, out float enter))
        {
            Vector3 targetPoint = ray.GetPoint(enter) + offset;

            transform.position = targetPoint;
        }
    }

    Vector3 GetPlaneNormal()
    {
        switch (drag_plane_enum)
        {
            case DragPlane.XY:
                return Vector3.forward;

            case DragPlane.XZ:
                return Vector3.up;

            case DragPlane.YZ:
                return Vector3.right;

            default:
                return Vector3.forward;
        }
    }
}
