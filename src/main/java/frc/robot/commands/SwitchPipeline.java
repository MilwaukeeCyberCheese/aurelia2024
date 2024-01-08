package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import java.util.function.IntSupplier;

public class SwitchPipeline extends Command {
    private CameraSubsystem m_cameraSubsystem;
    private IntSupplier m_pipelineIndex;
    private IntSupplier m_cameraNum;

    /**
     * Switch pipeline of a camera
     * 
     * @param cameraSubsystem subsystem containing the cameras
     *                        {@link frc.robot.subsystems.CameraSubsystem link}
     * @param pipelineIndex   the number of the pipeline to switch to
     * @param cameraNum       the number of the camera that you want to switch
     */
    public SwitchPipeline(CameraSubsystem cameraSubsystem, IntSupplier pipelineIndex, IntSupplier cameraNum) {
        m_cameraSubsystem = cameraSubsystem;
        m_pipelineIndex = pipelineIndex;
        m_cameraNum = cameraNum;
        addRequirements(m_cameraSubsystem);
    }

    @Override
    public void initialize() {
        CameraSubsystem.switchIndex(m_pipelineIndex.getAsInt(), m_cameraNum.getAsInt());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
