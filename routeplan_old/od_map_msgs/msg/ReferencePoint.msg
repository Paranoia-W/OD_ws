int32 index
string lane_id
string[] successor_lane_ids
string junction_id

float64 x
float64 y

float32 lane_left_boundary
float32 lane_right_boundary
float32 road_left_boundary
float32 road_right_boundary
float32 speed_limit # km/h

int32 lane_turn_type                # 0-ahead 1-turn left 2-turn right 3-U_turn
int32 merge_type                    # 0-non 1-MERGE_LEFT_IN 2-MERGE_RIGHT_IN 3-MERGE_LEFT_OUT 4-MERGE_RIGHT_OUT
int32 left_forward_lane_size
int32 right_forward_lane_size

bool left_boundary_available        # 0-实线 1-虚线
bool right_boundary_available       # 0-实线 1-虚线



