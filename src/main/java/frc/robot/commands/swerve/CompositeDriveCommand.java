package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.swerve.requests.RotationDirective;
import frc.robot.util.swerve.requests.RotationRequest;
import frc.robot.util.swerve.requests.TranslationDirective;
import frc.robot.util.swerve.requests.TranslationRequest;

public class CompositeDriveCommand extends Command {

    private final DriveSubsystem drive;

    private TranslationDirective translationDirective;
    private RotationDirective rotationDirective;

    private TranslationDirective defaultTranslationDirective;
    private RotationDirective defaultRotationDirective;

    public CompositeDriveCommand(
            DriveSubsystem drive,
            TranslationDirective defaultTranslation,
            RotationDirective defaultRotation,
            TranslationDirective starterTranslation,
            RotationDirective starterRotation
    ) {
        this.drive = drive;

        // Enforce non-null defaults: stopped directives if nothing provided
        this.defaultTranslationDirective = (defaultTranslation != null) ? defaultTranslation : new TranslationDirective() {
            @Override
            public TranslationRequest getRequest() {
                return new TranslationRequest.Stop();
            }
        };

        this.defaultRotationDirective = (defaultRotation != null) ? defaultRotation : new RotationDirective() {
            @Override
            public RotationRequest getRequest() {
                return new RotationRequest.Stop();
            }
        };

        // Active directives start with starter or default
        this.translationDirective = (starterTranslation != null) ? starterTranslation : this.defaultTranslationDirective;
        this.rotationDirective = (starterRotation != null) ? starterRotation : this.defaultRotationDirective;

        addRequirements(drive);
    }

    /* ---------------- Directive management ---------------- */

    public void setTranslationDirective(TranslationDirective directive) {
        if (translationDirective != null) translationDirective.end(true);
        translationDirective = (directive != null) ? directive : defaultTranslationDirective;
        translationDirective.init();
    }

    public void setRotationDirective(RotationDirective directive) {
        if (rotationDirective != null) rotationDirective.end(true);
        rotationDirective = (directive != null) ? directive : defaultRotationDirective;
        rotationDirective.init();
    }

    public void cancelTranslationDirective() {
        if (translationDirective != null) translationDirective.end(true);
        translationDirective = defaultTranslationDirective;
        translationDirective.init();
    }

    public void cancelTranslationDirective(TranslationDirective directive) {
        if (translationDirective == directive) cancelTranslationDirective();
    }

    public void cancelRotationDirective() {
        if (rotationDirective != null) rotationDirective.end(true);
        rotationDirective = defaultRotationDirective;
        rotationDirective.init();
    }

    public void cancelRotationDirective(RotationDirective directive) {
        if (rotationDirective == directive) cancelRotationDirective();
    }

    /* ---------------- Command lifecycle ---------------- */

    @Override
    public void initialize() {
        translationDirective.init();
        rotationDirective.init();
    }

    @Override
    public void execute() {
        drive.driveWithCompositeRequests(
                translationDirective.getRequest(),
                rotationDirective.getRequest()
        );

        if (translationDirective.isFinished()) {
            translationDirective.end(false);
            translationDirective = defaultTranslationDirective;
            translationDirective.init();
        }

        if (rotationDirective.isFinished()) {
            rotationDirective.end(false);
            rotationDirective = defaultRotationDirective;
            rotationDirective.init();
        }
    }

    @Override
    public void end(boolean interrupted) {
        translationDirective.end(true);
        rotationDirective.end(true);
    }

}