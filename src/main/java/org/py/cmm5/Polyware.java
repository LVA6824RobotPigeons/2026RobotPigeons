package org.py.cmm5;

import javax.swing.*;
import java.awt.*;
import java.io.File;

// so freaking tuff
public class Polyware {

    public static String assetsDirectory = System.getProperty("user.dir")+"/src/main/java/org/py/assets";

    public static int mode = 0;
    // 0: inactive
    // 1: diagnostic
    // 2: manual controller select
    // 3: startToSelect
    public static JFrame jf;
    public static JPanel jp;

    public static void boot() {

        System.out.println("[ControlsManager:Polyware] Opening new Polyware...");

        jf = new JFrame("ControlsManager Polyware");
        jp = new JPanel() {
            @Override
            public void paintComponent(Graphics g) {
                draw(g);
            }
        };

        jf.setIconImage(Toolkit.getDefaultToolkit().getImage(assetsDirectory+"/cmpolyware.png"));

        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        int padding = 20;
        Dimension windowDimensions = new Dimension(500,500);
        jf.setBounds(
                screenSize.width-windowDimensions.width-padding,
                screenSize.height-windowDimensions.height-padding-48,
                windowDimensions.width,
                windowDimensions.height
        );
        jf.add(jp);
        jf.toFront();
        jf.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        jf.setVisible(true);

    }
    public static void exit() {
        System.out.println("[ControlsManager:Polyware] Closing Polyware instance...");
        if(jf != null) jf.dispose();
    }
    public static void process() {
        if(mode == 2) ManualSelection.process();
        if(mode == 3) StartToSelectSelection.process();
    }

    private static Font calSans;
    public static void draw(Graphics g) {
        if(calSans == null) try {
            calSans = Font.createFont(
                    Font.TRUETYPE_FONT,new File(assetsDirectory+"/CalSans.ttf")
            );
        } catch(Exception ignored) {
            calSans = new Font("Arial",Font.PLAIN,12);
        }
        if(mode == 2) {
            ManualSelection.calSans = calSans;
            ManualSelection.draw(g);
        }
        if(mode == 3) {
            StartToSelectSelection.calSans = calSans;
            StartToSelectSelection.draw(g);
        }


    }



    public static void controllerSelection(String mode) {
        switch(mode) {
            case "manual":
                ManualSelection.start(); break;
            case "startToSelect":
            case "start-to-select":
                StartToSelectSelection.start(); break;
            default:
                if(Controls.errorLoggerOn) System.out.println("[ControlsManager:Polyware] Unknown controller selection mode: " + mode); break;
        }
    }









}
