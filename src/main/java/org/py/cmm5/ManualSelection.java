package org.py.cmm5;

import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWGamepadState;

import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;

public class ManualSelection {

    public static Font calSans = new Font("Arial",Font.PLAIN,12);

    private static boolean focused = false;
    private static final double[] focusIndicator = new double[2]; // T, C
    private static final double[][] modulePositions = new double[16][4]; // TX, CX, TY, CY
    private static final double[][] moduleDimensions = new double[16][4]; // TW, CW, TH, CH
    private static final boolean[] presentJIDs = new boolean[16];
    private static final boolean[] previousPresentJIDs = new boolean[16];
    private static final boolean[] justAddedJIDs = new boolean[16];
    private static final double[][] aJumps = new double[16][2]; // T, C
    private static final double[] selectorPos = new double[2]; // T, C
    private static final double[] selectorDim = new double[2]; // T, C

    private static int glfwInitTimer = 0;
    private static int selector = 0;
    private static int availableControllers = 0;
    private static final ArrayList<Integer> selectedJIDs = new ArrayList<>();
    public static void start() {

        Polyware.mode = 2;
        Polyware.boot();
        focused = Polyware.jf.isFocused();

        // Event Listeners for manual mode
        Polyware.jf.addWindowFocusListener(new WindowFocusListener() {
            @Override
            public void windowGainedFocus(WindowEvent e) {
                focused = true;
            }

            @Override
            public void windowLostFocus(WindowEvent e) {
                focused = false;
            }
        });
        Polyware.jf.addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                if(availableControllers > 0) if(e.getKeyCode() == 37) {
                    if(selector != 0) selector--;
                } else if(e.getKeyCode() == 39) {
                    if(selector != availableControllers - 1) selector++;
                } else if(e.getKeyCode() == 10 && availableControllers > 0) {
                    int index = -1;
                    for(int i = 0; i < 15; i++) {
                        if(GLFW.glfwJoystickPresent(i)) index++;
                        if(selector == index) {
                            if(selectedJIDs.contains(i)) selectedJIDs.remove((Integer) i);
                            else selectedJIDs.add(i);
                            break;
                        }
                    }
                } else if(e.getKeyCode() == 32) {
                    finish();
                }
            }
        });
        Polyware.jf.addWindowListener(new WindowListener() {
            @Override
            public void windowOpened(WindowEvent e) {}

            @Override
            public void windowClosing(WindowEvent e) {
                finish();
            }

            @Override
            public void windowClosed(WindowEvent e) {}

            @Override
            public void windowIconified(WindowEvent e) {}

            @Override
            public void windowDeiconified(WindowEvent e) {}

            @Override
            public void windowActivated(WindowEvent e) {

            }

            @Override
            public void windowDeactivated(WindowEvent e) {

            }
        });

    }
    public static void process() {

        if(selector >= availableControllers && selector != 0) {
            selector = availableControllers - 1;
        }
        System.arraycopy(presentJIDs, GLFW.GLFW_JOYSTICK_1, previousPresentJIDs, GLFW.GLFW_JOYSTICK_1, GLFW.GLFW_JOYSTICK_16);
        glfwInitTimer--;
        if(glfwInitTimer <= 0) {
            glfwInitTimer = 100;
            availableControllers = 0;
            GLFW.glfwPollEvents();
            for(int i = GLFW.GLFW_JOYSTICK_1; i < GLFW.GLFW_JOYSTICK_16; i++) {
                presentJIDs[i] = GLFW.glfwJoystickPresent(i);
                if(GLFW.glfwJoystickPresent(i)) availableControllers++;
            }
        }
        for(int jid = 0; jid < 15; jid++) {
            justAddedJIDs[jid] = presentJIDs[jid] != previousPresentJIDs[jid] && presentJIDs[jid];
            if(previousPresentJIDs[jid] != presentJIDs[jid] && !presentJIDs[jid] && selectedJIDs.contains(jid)) selectedJIDs.remove((Integer)jid);
        }
        if(Polyware.jf != null) Polyware.jf.repaint();
    }
    public static void draw(Graphics g) {

        g.setFont(calSans.deriveFont(40f));
        g.setColor(new Color(0, 0, 0));
        g.drawString("Controller Selection", 20, 30 + 20);

        // Focus indicator
        focusIndicator[0] = focused ? 10.0 : 0.0;
        focusIndicator[1] += ( focusIndicator[0] - focusIndicator[1] ) / 5.0;
        int f = (int) Math.round(focusIndicator[1]);
        g.setColor(new Color(0, 139, 255));
        g.fillRect(0, f, f, Polyware.jp.getHeight());
        g.fillRect(0, Polyware.jp.getHeight() - f, Polyware.jp.getWidth(), f);
        g.fillRect(Polyware.jp.getWidth() - f, 0, f, Polyware.jp.getHeight());
        g.fillRect(0, 0, Polyware.jp.getWidth(), f);

        g.setColor(new Color(0, 0, 0));
        g.setFont(calSans.deriveFont(19f));
        g.drawString("Press the left and right arrow keys to move between", 20, Polyware.jp.getHeight() - 80);
        g.drawString("controllers. To select, press enter. When you are", 20, Polyware.jp.getHeight() - 55);
        g.drawString("finished, press space or close this window.", 20, Polyware.jp.getHeight() - 30);


        Point mousePos = Polyware.jp.getMousePosition();
        if(mousePos == null) mousePos = new Point(0, 0);


        selectorPos[0] = selector * ( 100 + 20 );
        selectorPos[1] += ( selectorPos[0] - selectorPos[1] ) / 5.0;
        selectorDim[1] += ( selectorDim[0] - selectorDim[1] ) / 3.0;
        if(availableControllers == 0) selectorDim[0] = 0;
        else selectorDim[0] = 110;
        g.setColor(new Color(148, 209, 247));
        double stretch = ( selectorPos[0] - selectorPos[1] ) / 2;
        int stretchXAdd = ( stretch / ( stretch == 0 ? 1 : Math.abs(stretch) ) == -1 ) ? (int) Math.round(stretch) : 0;
        g.fillRoundRect(
                20 + 50 + (int) Math.round(selectorPos[1] - selectorDim[1] / 2.0 + stretchXAdd),
                50 + 50 + 20 - (int) Math.round(selectorDim[1] / 2.0),
                (int) Math.round(Math.abs(stretch)) + (int) Math.round(selectorDim[1]),
                (int) Math.round(selectorDim[1]),
                20,
                20);

        int x = 0;

//            System.out.println((int)Math.round(selectorPos[1]+(selectorDim[1]/-2.0)+(stretch/(Math.abs(stretch))==0?1:Math.abs(stretch))==-1?stretch:0));

        // Controller modules
        for(int jid = GLFW.GLFW_JOYSTICK_1; jid <= GLFW.GLFW_JOYSTICK_16; jid++) {
            if(presentJIDs[jid]) {
                boolean aPressed = false;
                if(GLFW.glfwJoystickPresent(jid)) {
                    GLFWGamepadState s = GLFWGamepadState.create();
                    GLFW.glfwGetGamepadState(jid, s);
                    aPressed = s.buttons(GLFW.GLFW_GAMEPAD_BUTTON_A) == 1;
                }

                int dimAmount = 0;
                if(!GLFW.glfwJoystickPresent(jid)) dimAmount = 50;

                moduleDimensions[jid][1] += ( moduleDimensions[jid][0] - moduleDimensions[jid][1] ) / 5.0;
                moduleDimensions[jid][3] += ( moduleDimensions[jid][2] - moduleDimensions[jid][3] ) / 5.0;
                modulePositions[jid][1] += ( modulePositions[jid][0] - modulePositions[jid][1] ) / 5.0;
                modulePositions[jid][3] += ( modulePositions[jid][2] - modulePositions[jid][3] ) / 5.0;

                DPoint d_dim = new DPoint(
                        100 + moduleDimensions[jid][1],
                        100 + moduleDimensions[jid][3]
                );
                Dimension dim = new Dimension(
                        (int) Math.round(d_dim.x),
                        (int) Math.round(d_dim.y)
                );
                Point pos = new Point(
                        x + 20 + (int) Math.round(modulePositions[jid][1] - (int) Math.round(moduleDimensions[jid][1]) / 2.0),
                        50 + 20 + (int) Math.round(modulePositions[jid][3] - (int) Math.round(moduleDimensions[jid][3] / 2.0))
                );


                if(mousePos.x > pos.x && mousePos.x < pos.x + dim.width &&
                        mousePos.y > pos.y && mousePos.y < pos.y + dim.height) {
                    moduleDimensions[jid][0] = -10.0;
                    moduleDimensions[jid][2] = -10.0;
                } else {
                    moduleDimensions[jid][0] = 0.0;
                    moduleDimensions[jid][2] = 0.0;
                }
                if(justAddedJIDs[jid]) {
                    moduleDimensions[jid][1] = -dim.width;
                    moduleDimensions[jid][3] = -dim.height;
                }
                if(aPressed) {
                    aJumps[jid][0] = 10;
                } else {
                    aJumps[jid][0] = 0;
                }
                aJumps[jid][1] += ( aJumps[jid][0] - aJumps[jid][1] ) / 4;
                int jumpAdd = -(int) aJumps[jid][1];
                int jumpColorAdd = (int) aJumps[jid][1] * 5;

                Color tc1 = new Color(180 - dimAmount, 180 - dimAmount + jumpColorAdd, 180 - dimAmount);
                g.setColor(tc1);
                g.fillRoundRect(pos.x, pos.y, dim.width, dim.height, 20, 20);

                if(selectedJIDs.contains(jid)) {
                    g.setColor(new Color(0, 0, 0));
                    g.setFont(calSans.deriveFont(30f));
                    g.drawString(
                            Integer.toString(selectedJIDs.indexOf(jid) + 1),
                            pos.x + dim.width / 2,
                            pos.y + 180
                    );
                }

                pos.y += jumpAdd;
                g.setColor(new Color(200 - dimAmount, 200 - dimAmount + jumpColorAdd, 200 - dimAmount));
                g.fillRoundRect(pos.x, pos.y, dim.width, dim.height, 20, 20);

                Color tc2 = new Color(150 - dimAmount, 150 - dimAmount + jumpColorAdd, 150 - dimAmount);
                g.setColor(tc2);
                g.setFont(calSans.deriveFont(12f + (float) ( moduleDimensions[jid][3] / 2 )));
                g.drawString(Integer.toString(jid), pos.x + 8, pos.y + 8 + (int) Math.round(12 + moduleDimensions[jid][3] / 2));


                g.setColor(tc1);
                int[] oval = new int[] {
                        pos.x + (int) Math.round(dim.width * 0.20), // x
                        pos.y + (int) Math.round(dim.height * 0.20), // y
                        (int) Math.round(dim.width * 0.60), // w
                        (int) Math.round(dim.height * 0.60) // h
                };
                g.fillOval(oval[0], oval[1], oval[2], oval[3]);

                g.setColor(tc2);
                g.setFont(calSans.deriveFont(30f + (float) ( moduleDimensions[jid][3] ) / 2));
                g.drawString("A", (int) Math.round(oval[0] + oval[2] / 2.0 - g.getFontMetrics().stringWidth("A") / 2.0), (int) Math.round(oval[1] + oval[3] / 2.0 + dim.height * 0.09));
                x += 100 + 20;

            }
        }
    }

    private static void finish() {
        Polyware.exit();
        Polyware.jf = null;
        Polyware.jp = null;
        ArrayList<Controller> controllers = new ArrayList<>();
        for(int i = 0; i < selectedJIDs.size(); i++) controllers.add(new Controller(selectedJIDs.get(i)));
        Controls.setControllers(controllers);
        Main.selectedJIDs = selectedJIDs;
        System.out.println("[ControlsManager:Polyware] Updated controllers: JIDs: " + selectedJIDs.toString());
        AllControls.ready = true;
//        frc.robot.Main.robot.m_robotContainer.refreshDriver();
        AllControls.sandboxStart();
    }

}
