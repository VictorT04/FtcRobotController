package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class SlidyFrog extends BlocksOpModeCompanion {

    public static double StablePowerMax = 0.45, StablePowerMin = 0.18, DistanceMax = 3900;


    @ExportToBlocks
            (
        parameterLabels = {"Angle","Arm Pos"}
    )
    public static double GravityCounter (double angle, double ArmPos)
    {
        if (ArmPos < 0.0)
        {
            ArmPos = 0.0;
        }
        return ((ArmPos / DistanceMax) * (StablePowerMax - StablePowerMin) + StablePowerMin)* Math.cos(Math.toRadians(angle));
    };
}
