uint32 c_id 
geometry_msgs/Point c_centroid
float32 c_volume
string frame_id
string recognition_label_2d
string recognition_label_3d

uint8 NONE=0
uint8 BOX=1
uint8 CYLINDER=2
uint8 SPHERE=3

int32 c_shape

uint8 c_color_average_r
uint8 c_color_average_g
uint8 c_color_average_b

uint32 c_color_average_qh
float32 c_color_average_qs
float32 c_color_average_qv

uint32 c_color_average_h
float32 c_color_average_s
float32 c_color_average_v

uint32[] c_hue_histogram
uint8 c_hue_histogram_quality

geometry_msgs/Point c_roi_origin
uint32 c_roi_width
uint32 c_roi_height

float32[] c_vfh_estimation
string c_svm_result
uint32 c_pose
