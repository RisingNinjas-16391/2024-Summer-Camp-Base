package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem;

public class ExtensionCommand extends CommandBase {
    private final SlidesSubsystem slidesSubsystem;
    private final double extension;

    public ExtensionCommand(final SlidesSubsystem slides, final double extension) {
        slidesSubsystem = slides;
        this.extension = extension;

        addRequirements(slidesSubsystem);
    }

    @Override
    public void execute() {
        slidesSubsystem.setExtension(extension);
        System.out.println("Pivot Position Execute");
    }


}
