record_cmd = "rosbag record /status /velodyne_points /scan /imu/data /jackal_velocity_controller/odom /gps/fix /gps/vel /imu/data_raw /tf /localization /move_base_simple/goal /navigation/cmd_vel /set_nav_target /set_pose"..
    " /image_raw/compressed "..
    " /joint_states"..
    " /joystick"..
    " /odom"..
    " /spot/camera/frontright/image/compressed "..
    " /spot/camera/frontleft/image/compressed "..
    " /spot/camera/right/image/compressed "..
    " /spot/camera/left/image/compressed "..
    " /spot/camera/back/image/compressed "..
    " /spot/camera/frontright/camera_info"..
    " /spot/camera/frontleft/camera_info"..
    " /spot/camera/right/camera_info"..
    " /spot/camera/left/camera_info"..
    " /spot/camera/back/camera_info"..
    " /spot/odometry/twist"..
    " /spot/status/battery_states"..
    " /spot/status/feet"..
    " /tf"..
    " /tf_static"..
    " __name:=joystick_rosbag_record&";

record_start_feedback_cmd = "paplay ~/Music/sosumi.wav &";
record_stop_feedback_cmd = "paplay ~/Music/temple.wav &";
max_linear_speed = 1.6;

Ps4Mapping = {
    manual_button = 4;
    autonomous_button = 5;
    manual_autonomous_axis = -1;

    sit_button = 3;
    stand_button = 0;
    sit_stand_axis = -1;

    x_axis = 4;
    y_axis = 3;
    r_axis = 0;
    axis_scale = -1;

    left_bumper = 4;
    record_start_button = 2;
    record_stop_button = 1;
};

SpektrumDxsMapping = {
--[[
Left Side:
    A button: button#0, also turns axis#5 from positive to negative
        axis#5 does not appear to map to anything else
    B switch: axis#4
        0 -> positive value
        1 -> zero value
        2 -> negative value
    D switch: axis#6
        0 -> positive value
        1 -> zero value
        2 -> negative value, also toggles button#1
    Left Joystick
        Left-Right: axis#0
            left -> positive
            right -> negative
        Up-Down: axis#1, only active if the H switch is set to 0
            up -> positive
            down -> negative

Right Side:
    H switch: enable/disable axis#1 joystick
        0 -> enable
        1 -> disable, arbitrary negative value reported
    F switch: reduce joystick values
        0 -> full range (-23638 to +23637 at neutral zero point)
        1 -> reduced range (about 70%)
    Right Joystick:
        Left-Right: axis #2
            left -> positive
            right -> negative
        Up-Down: axis#3
            up -> positive
            down -> negative

Four Trim Sliders: adjust zero point of joysticks
    Higher-pitch chirp at neutral zero point.
--]]
    manual_button = -1;
    autonomous_button = -1;
    manual_autonomous_axis = 6;

    sit_button = -1;
    stand_button = -1;
    sit_stand_axis = 4;

    x_axis = 3;
    y_axis = 2;
    r_axis = 0;
    axis_scale = 32768 / 23638;

    left_bumper = 0;
    record_start_button = 0;
    record_stop_button = 0;
};

Mapping = SpektrumDxsMapping;
