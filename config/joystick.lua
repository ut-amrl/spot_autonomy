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
    " /spot/camera/back/camera_info __name:=joystick_rosbag_record&";

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
-- Left button: button 0, also turns axis 5 from positive to negative
-- "A" switch: axis 4, 0->positive, 1->0, 2->negative
-- "D" switch: axis 6, 0->positive, 1->0, 2->negative and turns button 1 on
-- Left joystick LR: axis 0, left->positive, right->negative
-- Left joystick UpDn: axis 1, up->positive, down->negative; only active if "H" switch=0
-- "H" switch: 0->enables axis 1
-- "F" switch: 1->makes axes 0,2,3 magnitude smaller?
-- Right joystick LR: axis 2: left->positive, right->negative
-- Right joystick UpDn: axis 3: up->positive, down->negative

-- when the adjustment is approximately equal the sound feedback is more of a
-- high-pitch chirp than a beep

    manual_button = -1;
    autonomous_button = -1;
    manual_autonomous_axis = 6;

    sit_button = -1;
    stand_button = -1;
    sit_stand_axis = 4;
    axis_scale = 32768 / 23638;

    x_axis = 3;
    y_axis = 2;
    r_axis = 0;
    xyr_axes_inverted = false;

    left_bumper = 0;
    record_start_button = 0;
    record_stop_button = 0;
};

Mapping = SpektrumDxsMapping;
