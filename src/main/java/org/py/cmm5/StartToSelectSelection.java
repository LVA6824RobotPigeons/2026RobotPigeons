package org.py;

import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWGamepadState;

import java.awt.*;

import java.util.ArrayList;

public class StartToSelectSelection {

    public static Font calSans;

    public static void start() {

        Polyware.mode = 3;
        Polyware.boot();

    }
    private static int glfwInitTimer = 0;
    private static final ArrayList<Integer> selectedJIDs = new ArrayList<>();
    private static ArrayList<Integer> presentJIDs = new ArrayList<>();
    private static ArrayList<Integer> p_presentJIDs = new ArrayList<>();
    private static final boolean[] startPressed = new boolean[16];

    public static void process() {

        // GLFW Polling
        glfwInitTimer--;
        if(glfwInitTimer < 0) {
            glfwInitTimer = 100;
            GLFW.glfwPollEvents();
        }

        // Controller checking
        p_presentJIDs = presentJIDs;
        presentJIDs = new ArrayList<>();
        for(int jid = GLFW.GLFW_JOYSTICK_1; jid < GLFW.GLFW_JOYSTICK_16; jid++) {
            if(GLFW.glfwJoystickPresent(jid)) {
                presentJIDs.add(jid);
                GLFWGamepadState state = GLFWGamepadState.create();
                if(GLFW.glfwGetGamepadState(jid, state)) {
                    if(state.buttons(GLFW.GLFW_GAMEPAD_BUTTON_START) == 1) {
                        if(!startPressed[jid]) { // Just pressed
                            if(selectedJIDs.contains(jid)) selectedJIDs.remove((Integer) jid);
                            else selectedJIDs.add(jid);
                        }
                        startPressed[jid] = true;
                    } else startPressed[jid] = false;
                    if(Polyware.jf != null && selectedJIDs.indexOf(jid) == 0 && state.buttons(GLFW.GLFW_GAMEPAD_BUTTON_BACK) == 1) {
                        finish();
                    }
                } else startPressed[jid] = false;
            }
        }


        // JFrame repainting
        if(Polyware.jf != null) Polyware.jf.repaint();

    }

    private static final double[][] moduleYOff = new double[16][2]; // T, C

    public static void draw(Graphics g) {

        // Main GUI
        g.setFont(calSans.deriveFont(40f));
        g.setColor(new Color(0, 0, 0));
        g.drawString("Controller Selection", 20, 30 + 20);

        g.setFont(calSans.deriveFont(20f));
        g.drawString("Press START on your controller to be selected.",20, 60+20);
        if(!selectedJIDs.isEmpty()) {
            g.setColor(new Color(154, 0, 255));
            g.drawString("Select BACK on controller 1 to finish.",20,90+20);
        }

        int xOffset = 0;

        // Controller checking
        if(presentJIDs.isEmpty()) {

            g.setColor(new Color(200,200,200));
            g.setFont(calSans.deriveFont(24f));
            g.drawString("No controllers here.",Polyware.jf.getWidth()/2-g.getFontMetrics().stringWidth("No controllers here.")/2,Polyware.jf.getHeight()/2);

        }

        for(int jid = 0; jid < 16; jid++) {

            if(!presentJIDs.contains(jid)) {
                moduleYOff[jid][0] = -1;
            }

            if(presentJIDs.contains(jid) || moduleYOff[jid][1] > -1) {

                // Body rendering

                // JID Number
                if(selectedJIDs.contains(jid)) {

                    g.setColor(new Color(0,0,0));
                    g.setFont(calSans.deriveFont(24f));
                    g.drawString(
                            selectedJIDs.indexOf(jid)+1+"",
                            xOffset+70-g.getFontMetrics().stringWidth(selectedJIDs.indexOf(jid)+1+"")/2,
                            Polyware.jf.getHeight()-200
                    );

                }

                // Y calculating
                if(selectedJIDs.contains(jid)) {
                    moduleYOff[jid][0] = 160.0;
                } else if(presentJIDs.contains(jid)) {
                    moduleYOff[jid][0] = 90.0;
                }
                if(!p_presentJIDs.contains(jid) && presentJIDs.contains(jid)) {
                    moduleYOff[jid][0] = 0;
                    moduleYOff[jid][1] = 0;
                }
                moduleYOff[jid][1] += (moduleYOff[jid][0]-moduleYOff[jid][1])/5.0;

                g.setColor(new Color(200,200,200));
                g.fillRoundRect(
                        xOffset+20,
                        Polyware.jf.getHeight()-(int)Math.round(moduleYOff[jid][1]),
                        100,
                        100,
                        10,
                        10
                );

                g.setColor(new Color(180,180,180));
                int[] oval = new int[] {
                        xOffset+20 + (int) Math.round(100.0 * 0.20), // x
                        (int) Math.round(Polyware.jf.getHeight()-moduleYOff[jid][1] + 100.0 * 0.20), // y
                        (int) Math.round(100 * 0.60), // w
                        (int) Math.round(100 * 0.60) // h
                };
                g.fillOval(oval[0],oval[1],oval[2],oval[3]);

                g.setColor(new Color(150, 150, 150));
                g.setFont(new Font("Roboto",Font.PLAIN,30));
                // 2+, that way it doesn't look wierd
                g.drawString("▶", 2+(int) Math.round(oval[0] + oval[2] / 2.0 - g.getFontMetrics().stringWidth("▶") / 2.0), (int) Math.round(oval[1] + oval[3] / 2.0 + 100 * 0.09));

                xOffset += 120;

            }
        }
    }

    private static void finish() {
        Polyware.exit();
        Polyware.jf = null;
        Polyware.jp = null;
        ArrayList<Controller> controllers = new ArrayList<>();
        for(int i = 0; i < selectedJIDs.size(); i++) controllers.add(new Controller());
        Controls.setControllers(controllers);
        Main.selectedJIDs = selectedJIDs;
        System.out.println("[ControlsManager:Polyware] Updated controllers: JIDs: " + selectedJIDs.toString());
        Sandbox.ready = true;
        Sandbox.sandboxStart();
    }

}
