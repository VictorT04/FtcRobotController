package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import java.util.ArrayList;


@Config
public class Dashboard extends BlocksOpModeCompanion {
    static FtcDashboard dashboard;

    public static ArrayList<Double> RobotTrajectory = new ArrayList<>();

    public static ArrayList<Double> PerfectTrajectory = new ArrayList<>(), TargetRobot = new ArrayList<>();

    public static float a = 0, b = 0,c = 0, d = 0, e = 0, f = 0, g = 0, h = 0, i = 0, j = 0;

    public static double startedX, startedY;

    public static boolean RobotImage = true;

    static int count = 0;


    @ExportToBlocks(
            parameterLabels = {}
    )
    public static void InitDashboard() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotTrajectory.clear();
        PerfectTrajectory.clear();
        //TelemetryPacket packet = new TelemetryPacket();
    }

    @ExportToBlocks(
            parameterLabels = {"key", "text"}
    )
    public static void Telemetry_with_Text(String key, String text) {
        telemetry.addData(key, text);
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static void TelemetryUpdate() {
        telemetry.addData("size", RobotTrajectory.size());
        telemetry.update();
    }

    @ExportToBlocks(
            parameterLabels = {"key", "number"}
    )
    public static void Telemetry_with_number(String key, double number) {
        telemetry.addData(key, number);
    }


    @ExportToBlocks(
            parameterLabels = {"X", "Y", "orientation"}
    )
    public static void Trajectory(double X, double Y, double orientation) {
        count++;

        if (count >= 10) {
            TelemetryPacket packetPos = new TelemetryPacket();

            RobotTrajectory.add(ToDashboardCoordinate(Y, 'Y'));
            RobotTrajectory.add(ToDashboardCoordinate(X, 'X'));

            for (int count = 0; count < RobotTrajectory.size(); count += 2) {
                packetPos.fieldOverlay()
                        .setFill("blue")
                        .fillCircle(RobotTrajectory.get(count), RobotTrajectory.get(count + 1), 0.5);
            }

            for (int count = 0; count < PerfectTrajectory.size(); count += 2) {
                packetPos.fieldOverlay()
                        .setFill("green")
                        .fillCircle(PerfectTrajectory.get(count), PerfectTrajectory.get(count + 1), 0.5);
            }

            if (RobotImage) {
                packetPos.fieldOverlay()
                        .drawImage("/TargetRobot.png", TargetRobot.get(0), TargetRobot.get(1), 18, 18, Math.toRadians(TargetRobot.get(2)), 9, 9, true);

                packetPos.fieldOverlay()
                        .drawImage("/Robot.png", X / 2.54 + 9 + startedX, -Y / 2.54 + 135 - startedY, 18, 18, Math.toRadians(orientation + 90), 9, 9, true);
            }

            dashboard.sendTelemetryPacket(packetPos);
        }
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float A() {
        return a;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float B() {
        return b;
    }


    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float C() {
        return c;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float D() {
        return d;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float E() {
        return e;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float F() {
        return f;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float G() {
        return g;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float H() {
        return h;
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static float I() {
        return j;
    }

    @ExportToBlocks(
            parameterLabels = {"X at begining", "Y at begening"}
    )
    public static void InitTrajectory (float XtoAdd, float YtoAdd){
        startedX = XtoAdd/2.54;
        startedY = YtoAdd/2.54;

        TargetRobot.add(0.0);
        TargetRobot.add(0.0);
        TargetRobot.add(0.0);
    }

    @ExportToBlocks(
            parameterLabels = {"Xtarget","Ytarget", "Orientation Target", "X","Y", "Spline"}
    )
    public static void Target_Trajectory (double Xtarget, double Ytarget, double OrientationTarget, double X, double Y, char Spline)
    {
        TargetRobot.clear();
        TargetRobot.add(Xtarget / 2.54 + 9 + startedX);
        TargetRobot.add(-Ytarget/ 2.54 + 135 - startedY);
        TargetRobot.add(OrientationTarget + 90);

        PerfectTrajectory.clear();
        PerfectTrajectory.add(ToDashboardCoordinate(Y,'Y'));
        PerfectTrajectory.add(ToDashboardCoordinate(X,'X'));

        double Xdistance = Xtarget - X;
        double Ydistance = Ytarget - Y;

        double NbrOfPoints = 0, Xstep = 0, Ystep = 0;

        if (Math.abs(Xdistance) > Math.abs(Ydistance))
        {
            NbrOfPoints = Math.abs(Xdistance/3);
            if (Xdistance > 0)
                Xstep = 3;
            else if (Xdistance < 0)
                Xstep = -3;
            Ystep = Ydistance / NbrOfPoints;
        }
        else
        {
            NbrOfPoints = Ydistance/3;
            if (Ydistance > 0)
                Ystep = 3;
            else if (Ydistance < 0)
                Ystep = -3;
            Xstep = Xdistance / NbrOfPoints;
        }

        if (Spline == 'X')
        {
            double coefstep = 2/NbrOfPoints;

            for (int i = 0; i < NbrOfPoints; i++)
            {
                double coefX = 2 - i*coefstep;
                double coefY = i*coefstep;

                X += Xstep*coefX;
                Y += Ystep*coefY;

                PerfectTrajectory.add(ToDashboardCoordinate(Y,'Y'));
                PerfectTrajectory.add(ToDashboardCoordinate(X,'X'));
            }
        }
        else if (Spline == 'Y')
        {
            double coefstep = 2/NbrOfPoints;

            for (int i = 0; i < NbrOfPoints; i++)
            {
                double coefY = 2 - i * coefstep;
                double coefX = i * coefstep;

                X += Xstep * coefX;
                Y += Ystep * coefY;

                PerfectTrajectory.add(ToDashboardCoordinate(Y, 'Y'));
                PerfectTrajectory.add(ToDashboardCoordinate(X, 'X'));
            }
        }
        else

            for (int i = 0; i < NbrOfPoints; i++)
            {
                X += Xstep;
                Y += Ystep;

                PerfectTrajectory.add(ToDashboardCoordinate(Y,'Y'));
                PerfectTrajectory.add(ToDashboardCoordinate(X,'X'));
            }

    }

    public static double ToDashboardCoordinate (double coord, char axe)
    {
        double result = 0;
        if (axe == 'Y')
        {
            result = coord/2.54 - 63 + startedY;
        }
        else if (axe == 'X')
        {
            result = -coord/2.54 + 63 - startedX;
        }

        return result;
    }

}



