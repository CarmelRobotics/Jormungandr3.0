package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class vision extends SubsystemBase {
    @Override
    public void periodic() {
        SmartDashboard.putNumber("targets", LimelightHelpers.getTargetCount("limelight-lltwo"));
        if(LimelightHelpers.getTargetCount("limelight-lltwo") > 0){
            LimelightHelpers.setLEDMode_ForceBlink("limelight-lltwo");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight-lltwo");
        }
    }
}
