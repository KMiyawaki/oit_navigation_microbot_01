define lds ranger
(
  sensor(                         
    range [ 0.0  30.0 ]
    fov 270.25
    samples 1081
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define YDLIDAR_X4 ranger
(
  sensor(                         
    range [ 0.0  11.0 ]
    fov 360
    samples 720
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define robot position
(
  size [0.145 0.115 0.1265]
  # 0.145 / 2 - 0.0385
  origin [0.034 0 0 0]
  gui_nose 1
  drive "diff"
  #lds(pose [ -0.0475 0 0 0 ])
  YDLIDAR_X4(pose [ -0.0475 0 0 0 ])
)
