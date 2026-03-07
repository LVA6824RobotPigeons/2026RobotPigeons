package org.py;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

public class txtSon {

    public static HashMap<String,String> getData(String filePath) {
        return getData(new File(filePath));
    }
    public static HashMap<String,String> getData(File file) {
        ArrayList<String> contents = new ArrayList<>();
        try {
            Scanner sc = new Scanner(file);
            while(true) {
                String next = null;
                try { next = sc.nextLine(); } catch(Exception ignored) {}
                if(next == null) break;
                contents.add(next);
            }
            sc.close();
        } catch(FileNotFoundException e) {
            if(Controls.errorLoggerOn) System.out.println("[ControlsManager:ErrorLogger] File not found: " + file.getAbsolutePath());
        }
        return getData(contents.toArray(new String[0]));
    }
    public static HashMap<String,String> getData(String[] fileContents) {
        HashMap<String,String> data = new HashMap<>();
        for(String line : fileContents) {
            line = line.replaceAll(" ","").replaceAll("&&_"," ");
            String[] split = line.split(":");
            if(split.length == 1) {
                if(Controls.errorLoggerOn) System.out.println("[ControlsManager:ErrorLogger] A line in a txtSon file is missing a colon. Line: \"" + line + "\"");
            } else {
                data.put(split[0],split[1]);
            }
        }
        return data;
    }

}
