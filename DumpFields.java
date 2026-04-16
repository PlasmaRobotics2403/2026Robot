import java.io.*;
import java.lang.reflect.*;
import java.net.*;

public class DumpFields {
    public static void main(String[] args) throws Exception {
        URL jar = new File(
                        System.getenv("USERPROFILE")
                                + "\\.gradle\\caches\\modules-2\\files-2.1\\com.ctre.phoenix6\\wpiapi-java\\26.1.3\\97247af53528a5f5e7e9f51fbdd8dc705eac3abb\\wpiapi-java-26.1.3.jar")
                .toURI()
                .toURL();
        try (URLClassLoader cl = new URLClassLoader(new URL[] {jar})) {
            for (String name : new String[] {
                "com.ctre.phoenix6.configs.OpenLoopRampsConfigs",
                "com.ctre.phoenix6.configs.ClosedLoopRampsConfigs",
                "com.ctre.phoenix6.configs.TalonFXConfiguration"
            }) {
                Class<?> c = Class.forName(name, false, cl);
                System.out.println("CLASS " + name);
                for (Field f : c.getFields()) {
                    System.out.println(f.getName() + " : " + f.getType().getName());
                }
                System.out.println();
            }
        }
    }
}
