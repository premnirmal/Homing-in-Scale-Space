# Homing in Scale Space

### Implementation of Churchill & Vardy's Homing in Scale Space algorithm for a Pioneer 3-AT mobile robot.

The algorithm uses an omnidirectional camera to home the robot. The omnidirectional camera that we use is not V4L compatible i.e. not OpenCV compatible.
For this reason, we use mplayer to capture the image (using the system(..) command), and then OpenCV to image manipulation.

### Details on HiSS can be found in Churchill & Vardy's paper:
http://www.cs.mun.ca/~av/papers/churchill12invariant.pdf