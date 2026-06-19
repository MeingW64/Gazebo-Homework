float32 x                           # center x
float32 y                           # center y
float32 width                       # width
float32 height                      # height

string color                          # color of cone, 'r' = red, 'b' = blue, 'y' = yellow
float32 color_confidence              # confidence of cone detect

geometry_msgs/Point position          # 3d position with (x,y,z)
float32 pose_confidence

uint8 tracking_id
string tracking_state                 # OFF, OK, SEARCHING, TERMINATE
uint8 missing_frams
