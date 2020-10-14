package virtual_robot.config;

import javafx.scene.image.Image;

/**
 * Class for configuring field (width and image), and gamepad (virtual vs. real)
 */
public class Config {

    /**
     *  Width of the field, in pixels
     */
    public static final double FIELD_WIDTH = 648;

    /**
     * Whether to use "Virtual Gamepad" (true -> Virtual gamepad, false -> Real gamepad)
     */
    public static final boolean USE_VIRTUAL_GAMEPAD = true;

    /**
     * The image object for the field.
     */
    public static final Image BACKGROUND = new Image("/virtual_robot/assets/ultimategoal648Circle.png");
}
