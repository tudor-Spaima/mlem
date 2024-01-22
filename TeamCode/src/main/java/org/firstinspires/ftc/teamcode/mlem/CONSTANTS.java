package org.firstinspires.ftc.teamcode.mlem;

public class CONSTANTS {


    public static final double CLAW_L_OPEN = 0.2;
    public static final double CLAW_L_CLOSED = 0.1;
    public static final double CLAW_R_CLOSED = 0.9;
    public static final double CLAW_R_OPEN = 0.8;





    public static final double BRAT_INTAKE = 0.95;
    public static final double BRAT_SAFE = 0.97;
    public static final double BRAT_SCORE = 0.54;
    public static final double BRAT_STACK_1 = 0.87;
    public static final double BRAT_STACK_2 = 0.8;


    public static final double PIVOT_INTAKE = 0.12;
    public static final double PIVOT_SCORE = 0.31;
    public static final double PIVOT_SAFE = 0.7;

    public static final int SLIDE_UP = -1600;




    public enum intake_states {extended, retracted}

    public enum claw_states {open, closed}

    public enum brat_states {intake, score, safe, stack_1, stack_2}

    public enum claws{claw_l,claw_r}

    public enum swing_direction{up,down}









}
