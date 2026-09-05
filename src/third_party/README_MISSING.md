# Third-party source trees not cached

The robot was offline when this snapshot was assembled. Restore the exact Jetson copies of these dependencies before building:

- `FAST_LIO`
- `livox_ros_driver2`
- `Livox-SDK2`
- `Unitree_SDK2`

Do not silently replace the SDK versions until their ABI and the paths used by `go2_control/CMakeLists.txt` have been checked.
