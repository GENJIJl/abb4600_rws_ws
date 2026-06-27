MODULE EGMTest
    ! Start this after ros2_control is running and waiting for EGM.
    ! This module does not contain MoveJ/MoveL. The robot should only move
    ! after ROS 2 sends a trajectory command.

    VAR egmident egm_id;
    VAR egm_minmax egm_condition := [-1000, 1000];

    PROC EGM_Test()
        TPWrite "Starting EGM joint mode without pre-move.";

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

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "EGM UDP communication timeout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
