MODULE EGMHoldWide
    ! Start this after the ROS 2 hardware interface is already running.
    ! It does not contain MoveJ/MoveL, so enabling EGM should not move the robot
    ! unless ROS 2 sends a trajectory command later.

    VAR egmident egm_id;
    VAR egm_minmax egm_condition := [-1000, 1000];

    PROC EGM_Hold_Wide()
        EGMGetId egm_id;

        EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint;

        EGMActJoint egm_id
            \J1:=egm_condition
            \J2:=egm_condition
            \J3:=egm_condition
            \J4:=egm_condition
            \J5:=egm_condition
            \J6:=egm_condition
            \MaxSpeedDeviation:=200.0;

        WHILE TRUE DO
            EGMRunJoint egm_id, EGM_STOP_HOLD,
                \J1 \J2 \J3 \J4 \J5 \J6
                \CondTime:=5
                \RampInTime:=1
                \RampOutTime:=1;
        ENDWHILE

        EGMReset egm_id;
    ENDPROC
ENDMODULE
