package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class RedParams {
    public static double heading = 1.4;
    @Config
    public static class LeftSpikeMark{
        public static double x = 26;
        public static double y = -4;

        public static Vector2d position = new Vector2d(x, y);
    }

    @Config
    public static class CenterSpikeMark{
        public static double x = 36;
        public static double y = -14;

        public static Vector2d position = new Vector2d(x, y);
    }

    @Config
    public static class RightSpikeMark{
        public static double x = 26;
        public static double y = -24;

        public static Vector2d position = new Vector2d(x, y);
    }
}
