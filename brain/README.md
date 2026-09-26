# brain/

Runs on the Jetson Orin Nano Super (or any laptop) and drives Jero through the [Jero link](../docs/jero-link.md).

```bash
pip install -e ../jero_link
mkdir -p ~/.config/jero && scp bdxv2@jero.local:.config/jero/link.key ~/.config/jero/
python teleop.py jero.local                      # keyboard, works over SSH
python follow.py jero.local --camera 0 --dry-run # print commands only
python follow.py jero.local --camera 0 --show    # drive
```

| File | What |
|---|---|
| `teleop.py` | Keyboard teleop, including e-stop (`x`) and pause (`p`) |
| `follow_control.py` | Follow-a-person control law (bbox → vx, wz); pure Python, unit-tested |
| `follow.py` | Camera → detector → control law → link |
| `detectors.py` | `hog` baseline (OpenCV only, slow). Add a TensorRT detector here for the event. |

On the Jetson, use JetPack's system OpenCV (it has GStreamer support for CSI cameras) rather than
`pip install opencv-python`.
