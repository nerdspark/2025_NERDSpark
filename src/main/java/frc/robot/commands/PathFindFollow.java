// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.SmartController;
// import frc.robot.SmartController.DriveModeType;
// import frc.robot.subsystems.linebreak.LineBreak;

// /** A command that runs pathfindThenFollowPath based on the current drive mode. */
// public class PathFindFollow extends Command {
//   private Command scoreCommand;
//   private Command pathRun;
//   private LineBreak lineBreak;
//   boolean lastLineBreak = false;
//   private DriveModeType lastDriveMode = DriveModeType.AMP;

//   /**
//    * Creates a new PathFinderAndFollow command.
//    *
//    * @param driveModeSupplier a supplier for the drive mode type
//    */
//   public PathFindFollow(LineBreak lineBreak) {
//     this.lineBreak = lineBreak;
//   }

//   @Override
//   public void initialize() {
//     DriveModeType currentDriveMode = SmartController.getInstance().getDriveModeType();
//     if (lineBreak.hasNoGamePiece()) {
//       scoreCommand = getIntakeAutonPathCommand();

//     } else if (DriveModeType.AMP == currentDriveMode) {
//       scoreCommand = getAmpAutonPathCommand();
//     } else if (DriveModeType.SPEAKER == currentDriveMode) {
//       scoreCommand =
//           Commands.startEnd(
//               () -> SmartController.getInstance().enableSmartControl(),
//               () -> SmartController.getInstance().disableSmartControl());
//     }
//     scoreCommand.schedule();
//     lastLineBreak = lineBreak.hasNoGamePiece();
//     lastDriveMode = SmartController.getInstance().getDriveModeType();
//   }

//   @Override
//   public void execute() {
//     DriveModeType currentDriveMode = SmartController.getInstance().getDriveModeType();
//     boolean hasChangedLineBreak = (lineBreak.hasNoGamePiece() != lastLineBreak);

//     if (lineBreak.hasNoGamePiece() && hasChangedLineBreak) {
//       scoreCommand.cancel();
//       scoreCommand = getIntakeAutonPathCommand();
//       scoreCommand.schedule();
//     } else if (DriveModeType.AMP == currentDriveMode
//         && ((lastDriveMode != currentDriveMode) || hasChangedLineBreak)) {
//       scoreCommand.cancel();
//       scoreCommand = getAmpAutonPathCommand();
//       scoreCommand.schedule();
//     } else if (DriveModeType.SPEAKER == currentDriveMode
//         && ((lastDriveMode != currentDriveMode) || hasChangedLineBreak)) {
//       scoreCommand.cancel();
//       scoreCommand =
//           Commands.startEnd(
//               () -> SmartController.getInstance().enableSmartControl(),
//               () -> SmartController.getInstance().disableSmartControl());
//       scoreCommand.schedule();
//     }

//     lastLineBreak = lineBreak.hasNoGamePiece();
//     lastDriveMode = SmartController.getInstance().getDriveModeType();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     super.end(interrupted);
//     scoreCommand.cancel();
//   }

//   @Override
//   public boolean isFinished() {
//     return pathRun.isFinished();
//   }

//   /** Runs a new autonomous path based on the current drive mode. */
//   public Command getAmpAutonPathCommand() {
//     PathPlannerPath ampPath = PathPlannerPath.fromPathFile("Amp Placement Path");
//     PathConstraints constraints =
//         new PathConstraints(
//             drivetrainConfig.maxLinearVelocity(),
//             drivetrainConfig.maxLinearAcceleration(),
//             drivetrainConfig.maxAngularVelocity(),
//             drivetrainConfig.maxAngularAcceleration());
//     pathRun = AutoBuilder.pathfindThenFollowPath(ampPath, constraints, 0.0);
//     return Commands.sequence(pathRun);
//   }

//   public Command getIntakeAutonPathCommand() {
//     PathPlannerPath intakePath = PathPlannerPath.fromPathFile("Intake Path");
//     PathConstraints constraints =
//         new PathConstraints(
//             drivetrainConfig.maxLinearVelocity(),
//             drivetrainConfig.maxLinearAcceleration(),
//             drivetrainConfig.maxAngularVelocity(),
//             drivetrainConfig.maxAngularAcceleration());
//     pathRun = AutoBuilder.pathfindThenFollowPath(intakePath, constraints, 0.0);
//     return Commands.sequence(pathRun);
//   }
// }