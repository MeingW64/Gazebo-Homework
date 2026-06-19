# # message of Asensing perception result.It is INS data. 
# Header header         # standard ROS message header
# uint64 frame_id_num  
# float32 acc_x
# float32 acc_y
# float32 acc_z
# float32 gyro_x
# float32 gyro_y
# float32 gyro_z
# float32 pitch
# float32 roll
# float32 heading_angle
# float32 h
# uint64 ull_timestamp
# float64 lat
# float64 lon
# float32 vn
# float32 ve
# float32 vh
# uint32 gps_flag_pos
# uint32 num_sv
# uint32 gps_flag_heading
# uint64 gps_age
# uint32 car_status
# uint32 status
# float32 std_lat
# float32 std_lon
# float32 std_local_height
# float32 std_heading
# float32 utc_year
# float32 utc_month
# float32 utc_day
# float32 utc_hour
# float32 utc_min
# float32 utc_sec
# float64 utc_msec


# message of Asensing perception result.It is INS data. 
std_msgs/Header header         # standard ROS message header
uint64 frame_id_num  
float64 lat
float64 lon
float64 h

float64 vn
float64 ve
float64 vh

float64 roll
float64 pitch
float64 heading_angle

float64 gyro_x
float64 gyro_y
float64 gyro_z

float64 acc_x
float64 acc_y
float64 acc_z

float32 std_lat
float32 std_lon
float32 std_local_height

float32 north_velocity_std
float32 east_velocity_std
float32 ground_velocity_std


float32 roll_std
float32 pitch_std
float32 std_heading 


uint32 status
uint32 gps_flag_pos

float64 gps_week_number

float64 ull_timestamp

float32 temperature

uint32 car_status



uint32 gps_flag_heading
uint32 num_sv
