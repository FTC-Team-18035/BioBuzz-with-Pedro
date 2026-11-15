package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Use this procedure to store passable information from autonomous to teleOp *
 *
 * // In Autonomous
 * StoredData.myVariable = getAnythingYouNormallyRead();
 *
 * // In TeleOp
 * double neededInfo = StoredData.myVariable;
 */
public class StoredData {
    // add variables for information you want to save
    public static double lastAutoTargety = 52;  // default to near target y is always 58
    public static Pose lastAutoPose = new Pose(0,0,0);  // assign a value to avoid null possibility

}


