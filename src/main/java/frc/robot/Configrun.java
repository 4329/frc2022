package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Configrun
{
    private static HashMap<String, String> keys = new HashMap<String, String>();
    private static NetworkTableEntry theRobot = Shuffleboard.getTab("TestValues").add("ConfigFile", "NoConfig")
            .getEntry();

    static
    {
        // System.err.println("in config");
        // System.err.println("in config");
        System.err.println("in config");
        loadconfig();

    }

    public static int get(int defaultvalue, String key)
    {
        if (keys.get(key) != null)
        {
            // System.out.println("Run");
            return Integer.valueOf(keys.get(key));

            // return(keys.get(key));
        } else
        {
            System.err.println("No such value/type1");
            return defaultvalue;
        }
    }

    public static double get(double defaultvalue, String key)
    {
        if (keys.get(key) != null)
        {
            return Double.valueOf(keys.get(key));
            // return(keys.get(key));
        } else
        {
            System.err.println("No such value/type2");
            return defaultvalue;
        }

    }

    public static boolean get(boolean defaultvalue, String key)
    {
        if (keys.get(key) != null)
        {
            return Boolean.valueOf(keys.get(key));
            // return(keys.get(key));
        } else
        {
            System.err.println("No such value/type");
            return defaultvalue;
        }

    }

    public static void loadconfig()
    {
        BufferedReader reader;

        try
        {
            // int linenumber=0;
            // Files.exists(Paths.get("/home/lvuser/proto")
            if (new File("/home/lvuser/proto").exists())
            {
                reader = new BufferedReader(new FileReader("/home/lvuser/deploy/protoConfig.txt"));
                theRobot.setString("Proto");
            } else if (new File("/home/lvuser/dev").exists())
            {
                reader = new BufferedReader(new FileReader("/home/lvuser/deploy/devConfig.txt"));
                theRobot.setString("Dev");
            } else
            {
                reader = new BufferedReader(new FileReader("/home/lvuser/deploy/compConfig.txt"));
                theRobot.setString("Comp");
            }
            String line;
            // System.out.println("in load config");
            // reader.readLine();
            // System.out.println(line);
            // for (int run2 = 0; run2 <= line.length() - 1;) {
            // int run2=0;
            // System.out.println(line);
            // if (line.charAt(run2) == '#'){
            while ((line = reader.readLine()) != null)
            {
                // System.out.println("trim config");
                int linelength = line.trim().length();

                // line = reader.readLine();
                // System.out.println(linelength);
                if (linelength > 0)
                {
                    if (!line.startsWith("#"))
                    {
                        // if (!line.startsWith(" ")) {

                        // System.out.println(line);
                        String[] array = line.split("=");
                        // System.out.println(array[0]);
                        // System.out.println(array[1]);
                        // System.out.println("hashmap config");

                        keys.put(array[0].trim(), array[1].trim());
                    }
                }

                /*
                 * else { line = reader.readLine(); //}
                 */

            }
            reader.close();
        } catch (IOException e)
        {
            e.printStackTrace();
            System.err.println("in config");
            System.err.println("in config");
            System.err.println("in config");
        }
    }
}
