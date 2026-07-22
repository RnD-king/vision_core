#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VisionLinePoint {
  double u;
  double v;
} VisionLinePoint;

typedef struct VisionLineFeatures {
  double u_err_near;
  double u_err_lookahead;
  double u_err_ctrl;
  double slope;
  double n_visible;
  double in_recovery;
  double vx_prev;
  double wz_prev;
} VisionLineFeatures;

typedef struct VisionLineFeatureConfig {
  int max_centers;
  double image_center_u;
  double lookahead_delta_v_px;
  double lookahead_alpha_normal;
  double lookahead_alpha_recovery;
  double recover_enter_nvis;
  double recover_exit_nvis;
  double recover_enter_u;
  double recover_exit_u;
} VisionLineFeatureConfig;

typedef void *VisionLineControllerHandle;

VisionLineFeatureConfig vision_line_default_feature_config(void);
int vision_line_rectify_points(const VisionLinePoint *input, int count,
                               double fx, double fy, double cx, double cy,
                               double roll_rad, double pitch_rad,
                               VisionLinePoint *output);
VisionLineFeatures vision_line_compute_features(
    const VisionLinePoint *points, int count, int image_width, int image_height,
    int previous_in_recovery, double vx_prev, double wz_prev,
    VisionLineFeatureConfig config);
VisionLineControllerHandle vision_line_controller_create(double observation_dt);
void vision_line_controller_destroy(VisionLineControllerHandle handle);
int vision_line_controller_set_path(VisionLineControllerHandle handle,
                                    const VisionLinePoint *path_xy,
                                    const double *path_s,
                                    const double *path_heading, int count);
void vision_line_controller_observe(VisionLineControllerHandle handle,
                                    VisionLineFeatures features, double x,
                                    double y, double yaw);
void vision_line_controller_compute(VisionLineControllerHandle handle,
                                    VisionLineFeatures features, double *vx,
                                    double *wz);
void vision_line_controller_compute_tracking(VisionLineControllerHandle handle,
                                             VisionLineFeatures features,
                                             double *vx, double *wz);
void vision_line_controller_compute_search_rotation(
    VisionLineControllerHandle handle, double *vx, double *wz);
void vision_line_controller_compute_lookahead_approach(
    VisionLineControllerHandle handle, VisionLineFeatures features, double *vx,
    double *wz);
int vision_line_controller_in_recovery(VisionLineControllerHandle handle);
void vision_line_controller_begin_reacquisition(
    VisionLineControllerHandle handle);
void vision_line_controller_reset(VisionLineControllerHandle handle);

typedef struct VisionObjectDetection {
  double x;
  double y;
  double width;
  double height;
  double confidence;
  int class_id;
} VisionObjectDetection;

typedef struct VisionObjectTarget {
  int valid;
  int class_id;
  double confidence;
  double x;
  double y;
  double width;
  double height;
  double center_u;
  double center_v;
  double rectified_u;
  double rectified_v;
  int center_rectified;
} VisionObjectTarget;

typedef struct VisionObjectTargets {
  VisionObjectTarget ball;
  VisionObjectTarget goal;
  VisionObjectTarget backboard;
  VisionObjectTarget hurdle;
} VisionObjectTargets;

VisionObjectTargets vision_object_extract_targets(
    const VisionObjectDetection *detections, int count,
    int ball_class_id, int goal_class_id, int backboard_class_id,
    int hurdle_class_id, double confidence_threshold);

typedef void *VisionBallControllerHandle;

typedef struct VisionBallResult {
  int active;
  int reached_pickup_pose;
  int request_camera_down;
  int mode;
  int stable;
  int visible;
  double u_norm;
  double v_norm;
  double h_norm;
  double area_norm;
  double confidence;
  double vx;
  double vy;
  double wz;
} VisionBallResult;

VisionBallControllerHandle vision_ball_controller_create(void);
void vision_ball_controller_destroy(VisionBallControllerHandle handle);
VisionBallResult vision_ball_controller_compute(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec);
// V2 preserves VisionBallResult's ABI while adding the context required by the
// G1 adapter. camera_actual_mode: 0=FORWARD, 1=DOWN, 2=TRANSITION.
VisionBallResult vision_ball_controller_compute_v2(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int camera_actual_mode, int camera_settled);
// V3 keeps every existing struct/symbol unchanged and adds an explicit
// reference-valid bit. Set line_reference_valid=1 only for normal line
// tracking; RECOV/coast/search vx values must pass 0 even when they are > 0.
VisionBallResult vision_ball_controller_compute_v3(
    VisionBallControllerHandle handle, VisionObjectTarget ball_target,
    int image_width, int image_height, double now_sec, double line_vx,
    int line_reference_valid, int camera_actual_mode, int camera_settled);
void vision_ball_controller_reset(VisionBallControllerHandle handle);

typedef void *VisionHurdleControllerHandle;

typedef struct VisionHurdleResult {
  int active;
  int camera_request;
  int action_request;
  int mode;
  int stable;
  int visible;
  double u_norm;
  double v_norm;
  double h_norm;
  double bottom_norm;
  double confidence;
  double vx;
  double vy;
  double wz;
} VisionHurdleResult;

VisionHurdleControllerHandle vision_hurdle_controller_create(void);
void vision_hurdle_controller_destroy(VisionHurdleControllerHandle handle);
VisionHurdleResult vision_hurdle_controller_compute(
    VisionHurdleControllerHandle handle, VisionObjectTarget hurdle_target,
    int image_width, int image_height, double now_sec,
    int camera_actual_mode, int camera_settled);
void vision_hurdle_controller_reset(VisionHurdleControllerHandle handle);

typedef void *VisionGoalControllerHandle;

typedef struct VisionGoalResult {
  int active;
  int camera_request;
  int action_request;
  int mode;
  int stable;
  int visible;
  double u_norm;
  double v_norm;
  double h_norm;
  double confidence;
  double vx;
  double vy;
  double wz;
} VisionGoalResult;

VisionGoalControllerHandle vision_goal_controller_create(void);
void vision_goal_controller_destroy(VisionGoalControllerHandle handle);
void vision_goal_controller_start_after_pickup(
    VisionGoalControllerHandle handle, double now_sec);
VisionGoalResult vision_goal_controller_compute(
    VisionGoalControllerHandle handle, VisionObjectTarget goal_target,
    int image_width, int image_height, double now_sec,
    int line_reference_valid, int camera_actual_mode, int camera_settled);
void vision_goal_controller_reset(VisionGoalControllerHandle handle);

typedef struct VisionSelectedMotionCommand {
  double vx;
  double vy;
  double wz;
  int source;
  int ball_mode;
} VisionSelectedMotionCommand;

VisionSelectedMotionCommand vision_select_motion_command(
    VisionLineControllerHandle line_controller, VisionLineFeatures line_features,
    VisionBallResult ball_result);
// Use this overload when line vx/vy/wz was already computed this frame. It
// performs selection only and therefore does not advance line-controller state
// a second time.
VisionSelectedMotionCommand vision_select_motion_command_v2(
    VisionBallResult ball_result, double line_vx, double line_vy,
    double line_wz);

typedef struct VisionSelectedMissionCommand {
  double vx;
  double vy;
  double wz;
  int source;
  int active_mode;
} VisionSelectedMissionCommand;

VisionSelectedMissionCommand vision_select_mission_command(
    VisionBallResult ball_result, VisionHurdleResult hurdle_result,
    VisionGoalResult goal_result, double line_vx, double line_vy,
    double line_wz);

#ifdef __cplusplus
}
#endif
