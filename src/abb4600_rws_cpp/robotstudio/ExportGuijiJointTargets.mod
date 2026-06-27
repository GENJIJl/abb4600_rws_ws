MODULE ExportGuijiJointTargets
    ! Load this module together with TrainHookGrab.MOD.
    ! Run ExportGuijiJointTargets_ToFile on the FlexPendant.
    ! It writes HOME:/guiji_jointtargets.txt.
    ! The values are robot axis angles in degrees and can be copied into
    ! the MoveIt2 node as joint targets for MoveJ waypoints.

    VAR iodev out_file;

    PROC PrintJointTarget(string name, robtarget target)
        VAR jointtarget jt;

        jt := CalcJointT(target, Tooldata_HookM\WObj:=wobj_low_300_1200);
        TPWrite name + ":=[" +
            NumToStr(jt.robax.rax_1, 6) + "," +
            NumToStr(jt.robax.rax_2, 6) + "," +
            NumToStr(jt.robax.rax_3, 6) + "," +
            NumToStr(jt.robax.rax_4, 6) + "," +
            NumToStr(jt.robax.rax_5, 6) + "," +
            NumToStr(jt.robax.rax_6, 6) + "];";
    ENDPROC

    PROC WriteJointTarget(string name, robtarget target)
        VAR jointtarget jt;

        Write out_file, "CALC " + name;
        jt := CalcJointT(target, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, name;
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";
    ENDPROC

    PROC ExportGuiji1JointTargets()
        TPWrite "BEGIN guiji_1 jointtargets";
        PrintJointTarget "p10", p10;
        PrintJointTarget "p110", p110;
        PrintJointTarget "p120", p120;
        PrintJointTarget "p130", p130;
        PrintJointTarget "p140", p140;
        PrintJointTarget "p150", p150;
        PrintJointTarget "p160", p160;
        PrintJointTarget "p170", p170;
        PrintJointTarget "p180", p180;
        PrintJointTarget "p190", p190;
        PrintJointTarget "p210", p210;
        PrintJointTarget "p220", p220;
        PrintJointTarget "p230", p230;
        PrintJointTarget "p240", p240;
        PrintJointTarget "p250", p250;
        PrintJointTarget "p260", p260;
        PrintJointTarget "p270", p270;
        PrintJointTarget "p280", p280;
        PrintJointTarget "p290", p290;
        PrintJointTarget "p300", p300;
        PrintJointTarget "p310", p310;
        PrintJointTarget "p320", p320;
        PrintJointTarget "p330", p330;
        PrintJointTarget "p340", p340;
        PrintJointTarget "p350", p350;
        PrintJointTarget "p360", p360;
        PrintJointTarget "p410", p410;
        PrintJointTarget "p420", p420;
        PrintJointTarget "p430", p430;
        PrintJointTarget "p440", p440;
        PrintJointTarget "p450", p450;
        PrintJointTarget "p460", p460;
        PrintJointTarget "p470", p470;
        TPWrite "END guiji_1 jointtargets";
    ENDPROC

    PROC ExportGuiji2JointTargets()
        TPWrite "BEGIN guiji_2 jointtargets";
        PrintJointTarget "p10", p10;
        PrintJointTarget "p490", p490;
        PrintJointTarget "p710", p710;
        PrintJointTarget "p630", p630;
        PrintJointTarget "p530", p530;
        PrintJointTarget "p730", p730;
        PrintJointTarget "p650", p650;
        PrintJointTarget "p660", p660;
        PrintJointTarget "p570", p570;
        PrintJointTarget "p680", p680;
        PrintJointTarget "p720", p720;
        PrintJointTarget "p700", p700;
        PrintJointTarget "p590", p590;
        TPWrite "END guiji_2 jointtargets";
    ENDPROC

    PROC WriteGuiji1JointTargets()
        Write out_file, "BEGIN guiji_1 jointtargets";
        WriteJointTarget "p10", p10;
        WriteJointTarget "p110", p110;
        WriteJointTarget "p120", p120;
        WriteJointTarget "p130", p130;
        WriteJointTarget "p140", p140;
        WriteJointTarget "p150", p150;
        WriteJointTarget "p160", p160;
        WriteJointTarget "p170", p170;
        WriteJointTarget "p180", p180;
        WriteJointTarget "p190", p190;
        WriteJointTarget "p210", p210;
        WriteJointTarget "p220", p220;
        WriteJointTarget "p230", p230;
        WriteJointTarget "p240", p240;
        WriteJointTarget "p250", p250;
        WriteJointTarget "p260", p260;
        WriteJointTarget "p270", p270;
        WriteJointTarget "p280", p280;
        WriteJointTarget "p290", p290;
        WriteJointTarget "p300", p300;
        WriteJointTarget "p310", p310;
        WriteJointTarget "p320", p320;
        WriteJointTarget "p330", p330;
        WriteJointTarget "p340", p340;
        WriteJointTarget "p350", p350;
        WriteJointTarget "p360", p360;
        WriteJointTarget "p410", p410;
        WriteJointTarget "p420", p420;
        WriteJointTarget "p430", p430;
        WriteJointTarget "p440", p440;
        WriteJointTarget "p450", p450;
        WriteJointTarget "p460", p460;
        WriteJointTarget "p470", p470;
        Write out_file, "END guiji_1 jointtargets";
        Write out_file, "";
    ENDPROC

    PROC WriteGuiji2JointTargets()
        Write out_file, "BEGIN guiji_2 jointtargets";
        WriteJointTarget "p10", p10;
        WriteJointTarget "p490", p490;
        WriteJointTarget "p710", p710;
        WriteJointTarget "p630", p630;
        WriteJointTarget "p530", p530;
        WriteJointTarget "p730", p730;
        WriteJointTarget "p650", p650;
        WriteJointTarget "p660", p660;
        WriteJointTarget "p570", p570;
        WriteJointTarget "p680", p680;
        WriteJointTarget "p720", p720;
        WriteJointTarget "p700", p700;
        WriteJointTarget "p590", p590;
        Write out_file, "END guiji_2 jointtargets";
        Write out_file, "";
    ENDPROC

    PROC ExportGuijiJointTargets_ToFile()
        Open "HOME:" \File:="guiji_jointtargets.txt", out_file \Write;
        Write out_file, "TrainHookGrab guiji_1/guiji_2 jointtargets";
        Write out_file, "Unit: degree";
        Write out_file, "";
        Write out_file, "START WriteGuiji1JointTargets";
        WriteGuiji1JointTargets;
        Write out_file, "START WriteGuiji2JointTargets";
        WriteGuiji2JointTargets;
        Close out_file;
        TPWrite "Exported HOME:/guiji_jointtargets.txt";
    ENDPROC

    PROC ExportMoveJJointTargets_ToFile()
        VAR jointtarget jt;

        Open "HOME:" \File:="guiji_movej_jointtargets.txt", out_file \Write;
        Write out_file, "TrainHookGrab guiji_1/guiji_2 MoveJ jointtargets";
        Write out_file, "Unit: degree";
        Write out_file, "";

        Write out_file, "CALC p10";
        jt := CalcJointT(p10, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, "p10";
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";

        Write out_file, "CALC p110";
        jt := CalcJointT(p110, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, "p110";
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";

        Write out_file, "CALC p120";
        jt := CalcJointT(p120, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, "p120";
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";

        Write out_file, "CALC p470";
        jt := CalcJointT(p470, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, "p470";
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";

        Write out_file, "CALC p490";
        jt := CalcJointT(p490, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, "p490";
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";

        Write out_file, "CALC p710";
        jt := CalcJointT(p710, Tooldata_HookM\WObj:=wobj_low_300_1200);
        Write out_file, "p710";
        Write out_file, "j1=" + NumToStr(jt.robax.rax_1, 6);
        Write out_file, "j2=" + NumToStr(jt.robax.rax_2, 6);
        Write out_file, "j3=" + NumToStr(jt.robax.rax_3, 6);
        Write out_file, "j4=" + NumToStr(jt.robax.rax_4, 6);
        Write out_file, "j5=" + NumToStr(jt.robax.rax_5, 6);
        Write out_file, "j6=" + NumToStr(jt.robax.rax_6, 6);
        Write out_file, "";

        Close out_file;
        TPWrite "Exported HOME:/guiji_movej_jointtargets.txt";
    ENDPROC

    PROC ExportGuijiJointTargets_Main()
        ExportGuiji1JointTargets;
        ExportGuiji2JointTargets;
    ENDPROC
ENDMODULE
