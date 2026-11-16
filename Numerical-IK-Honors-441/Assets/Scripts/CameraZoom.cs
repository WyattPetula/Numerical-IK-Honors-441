using UnityEngine;
using UnityEngine.InputSystem;

public class CameraZoom : MonoBehaviour
{
    public GameObject target_obj;
    [Header("Input Action")]
    [SerializeField] private InputActionReference zoom_action;

    [Header("Zoom Settings")]
    [SerializeField] private float zoom_speed = 0.05f;
    [SerializeField] private float smooth_time = 0.1f;
    [SerializeField] private float min_zoom = 15f;
    [SerializeField] private float max_zoom = 60f;

    private float initial_ortho_size;
    private Vector3 initial_scale;

    private Camera _camera;
    private float target_zoom;
    private float current_velocity;

    private void Awake()
    {
        _camera = GetComponent<Camera>();
        target_zoom = _camera.orthographic ? _camera.orthographicSize : _camera.fieldOfView;
    }
    void Start()
    {
        initial_ortho_size = _camera.orthographicSize;
        initial_scale = transform.localScale;
    }

    private void OnEnable()
    {
        zoom_action.action.Enable();
    }

    private void OnDisable()
    {
        zoom_action.action.Disable();
    }

    private void Update()
    {
        float scaleFactor = _camera.orthographicSize / initial_ortho_size;
        target_obj.transform.localScale = initial_scale * scaleFactor / 2f;

        // Read the Y value from the Vector2 scroll input
        float scrollValue = zoom_action.action.ReadValue<Vector2>().y;

        if (scrollValue != 0f)
        {
            target_zoom -= scrollValue * zoom_speed;
            target_zoom = Mathf.Clamp(target_zoom, min_zoom, max_zoom);
        }

        // Smoothly interpolate to the target zoom level
        if (_camera.orthographic)
        {
            _camera.orthographicSize = Mathf.SmoothDamp(_camera.orthographicSize, target_zoom, ref current_velocity, smooth_time);
        }
        else
        {
            _camera.fieldOfView = Mathf.SmoothDamp(_camera.fieldOfView, target_zoom, ref current_velocity, smooth_time);
        }
    }
}
