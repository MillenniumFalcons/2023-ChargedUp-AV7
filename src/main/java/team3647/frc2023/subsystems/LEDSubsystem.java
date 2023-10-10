// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import team3647.frc2023.constants.LEDConstants;
import team3647.lib.PeriodicSubsystem;

public class LEDSubsystem implements PeriodicSubsystem {
    /** Creates a new LEDSubsystem. */
    private boolean target = false;

    private Piece currentPiece = Piece.none;
    private Piece storedPiece = Piece.none;

    private Piece wantedPiece = Piece.cone;
    private VisionState visionState = VisionState.none;
    private boolean wantedCone = false;
    private boolean pieceIn = false;
    private boolean pieceInBottom = false;

    private Animation cubeColor = LEDConstants.SOLID_PURPLE;
    private Animation coneColor = LEDConstants.SOLID_YELLOW;
    private Animation bottomCubeInColor = LEDConstants.FLASH_PURPLE;

    public static enum LEDStates {
        TARGET("TARGET"),
        RAINBOW("RAINBOW"),
        IDLE("IDLE");

        private String name;

        LEDStates(String name) {
            this.name = name;
        }

        public String getLEDState() {
            return name;
        }
    }

    public enum Piece {
        none,
        cone,
        cube,
        groundcube
    }

    public enum VisionState {
        none,
        sees,
        aligned
    }

    private LEDStates currentState;
    private LEDStates wantedState = LEDStates.IDLE;

    private CANdle m_candle = LEDConstants.m_candle;

    public LEDSubsystem() {
        setAnimation(LEDConstants.BREATHE_RED);
    }

    private void setAnimation(Animation animation) {
        m_candle.animate(animation);
    }

    public void setLEDState(LEDStates state) {
        wantedState = state;
    }

    public void setTarget(boolean bool) {
        this.target = bool;
    }

    public Command setPiece(Piece piece) {
        return Commands.sequence(setPieceInTrue(), runOnce(() -> this.currentPiece = piece));
    }

    public Command setPieceInTrue() {
        return Commands.runOnce(() -> setPieceIn(true));
    }

    public Command storePiece() {
        return Commands.run(() -> this.storedPiece = this.wantedPiece);
    }

    public Command setWantedPiece(Piece piece) {
        return Commands.runOnce(() -> this.wantedPiece = piece);
    }

    public String getStoredPiece() {
        return storedPiece.toString();
    }

    public Command unStore() {
        return Commands.runOnce(() -> setWantedPiece(this.storedPiece));
    }

    public Command cubeOrCone(BooleanSupplier cone) {
        return setWantedPiece(cone.getAsBoolean() ? Piece.cone : Piece.cube);
    }

    public void setPieceIn(boolean in) {
        this.pieceIn = in;
    }

    public void setBottomCubeIn(boolean in) {
        this.pieceInBottom = in;
    }

    public void setVisionState(VisionState state) {
        this.visionState = state;
    }

    public boolean getPieceIn() {
        return pieceIn;
    }

    public String getWantedPiece() {
        return this.wantedPiece.toString();
    }

    public String getCurrentPiece() {
        return this.currentPiece.toString();
    }

    public String getLEDState() {
        return wantedState.getLEDState();
    }

    @Override
    public void periodic() {
        switch (wantedState) {
            case TARGET:
                if (currentPiece == Piece.cone) {
                    switch (this.visionState) {
                        case none:
                            this.setAnimation(LEDConstants.SOLID_RED);
                            break;
                        case sees:
                            this.setAnimation(LEDConstants.BREATHE_GREEN);
                            break;
                        case aligned:
                            this.setAnimation(LEDConstants.SOLID_BLUE);
                            break;
                    }
                } // if leds turn off when scoring cube put else statement here
                break;
            case RAINBOW:
                this.setAnimation(LEDConstants.RAINBOWCONTROLLER);
                break;
            case IDLE:
                if (pieceIn) {
                    switch (this.currentPiece) {
                        case none:
                            this.setAnimation(LEDConstants.SOLID_RED);
                            break;
                        case cone:
                            this.setAnimation(LEDConstants.FLASH_YELLOW);
                            break;
                        case cube:
                            this.setAnimation(LEDConstants.FLASH_PURPLE);
                            break;
                        case groundcube:
                            this.setAnimation(LEDConstants.FLASH_BROWN);
                            break;
                    }
                } else {
                    switch (this.wantedPiece) {
                        case none:
                            this.setAnimation(LEDConstants.SOLID_RED);
                            break;
                        case cone:
                            this.setAnimation(LEDConstants.SOLID_YELLOW);
                            break;
                        case cube:
                            this.setAnimation(LEDConstants.SOLID_PURPLE);
                            break;
                        case groundcube:
                            this.setAnimation(LEDConstants.SOLID_BROWN);
                            break;
                    }
                }
                // if (pieceIn) {
                //     if (pieceInBottom) {
                //         this.setAnimation(bottomCubeInColor);
                //     } else {
                //         if (currentCone) {
                //             this.setAnimation(coneColor);
                //         } else {
                //             this.setAnimation(cubeColor);
                //         }
                //     }
                // } else {
                //     if (wantedCone) {
                //         this.setAnimation(coneColor);
                //     } else {
                //         this.setAnimation(cubeColor);
                //     }
                // }
                break;
        }
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "LED Subystem";
    }
}
