package org.firstinspires.ftc.teamcode.pedro.visualizer_playground;

import com.acmerobotics.dashboard.config.Config;
@Config
public class CoordinatesOfValue {
    /**
     * Use this space as storage for named poses of value to use for Pedro
     * Keep the naming convention the same to allow easy implementation in auto creation
     */
    // StartA Pose (x, y in inches, heading in degrees) near hive centered and facing
    public static double STARTA_X = 58;
    public static double STARTA_Y = 8;
    public static double STARTA_HEADING = 90.0;

    // ShootA Pose
    public static double SHOOTA_X = 58;
    public static double SHOOTA_Y = 16.5;
    public static double SHOOTA_HEADING = 90.0;

    // AlignFlowerA
    public static double ALIGNFLOWERA_X = 22.5;
    public static double ALIGNFLOWERA_Y = 47;
    public static double ALIGNFLOWERA_HEADING = -180;

    // PickFlowerA
    public static double PICKFLOWERA_X = 17;// TODO - This is a high value until tuned for safe flowerPicker extension and will need tuned

    public static double PICKFLOWERA_Y = 47;
    public static double PICKFLOWERA_HEADING = 180;

    // ShootA Pose
    public static double SHOOTB_X = 58;
    public static double SHOOTB_Y = 127;
    public static double SHOOTB_HEADING = 90.0;

    // AlignFlowerA
    public static double ALIGNFLOWERB_X = 47;
    public static double ALIGNFLOWERB_Y = 132;   // TODO - This is a low value until tuned for safe flowerPicker extension and will need tuned
    public static double ALIGNFLOWERB_HEADING = 90;

}
