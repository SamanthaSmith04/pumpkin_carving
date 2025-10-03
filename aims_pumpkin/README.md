# AIMS Pumpkin Carving

## Setup
```
mkdir -p ws_pumpkin/src
vcs import ws_pumpkin/src < ws_pumpkin/src/aims_pumpkin/dependencies.repos


mkdir -p ws_tesseract/src
vcs import src < src/tesseract/dependencies.repos

rosdep install --from-paths src -iry

colcon build --symlink-install --cmake-args -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF -DBUILD_STUDIO=OFF
```

## PLANNING

Client will send pose array of desired carving path to the service, which will return a trajectory to follow that path.

How to handle when jumps should happen?
  - Would it be good to do a freespace motion between carving segments?
    - Could be done by checking distance between end of one segment and start of next segment
    - If distance is above a certain threshold, add a freespace motion to trajectory

  - all waypoints that are used for carving should be straight line segments?