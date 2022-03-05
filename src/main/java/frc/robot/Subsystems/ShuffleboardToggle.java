package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import frc.robot.Subsystems.TurretSubsystem;

public class ShuffleboardToggle extends SubsystemBase{
    
    public ShuffleboardToggle(){

    }



    public void shuffleboardToggle(){
        if (Configrun.get(false, "extraShuffleBoardToggle")){
        }
    } 

}
