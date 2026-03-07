package org.py;

import java.util.function.Consumer;

public class Function {

    public Consumer<String> function;
    public boolean once;
    public boolean onInactive;

    public Function(Consumer<String> function, boolean once, boolean onInactive) {
        this.function = function;
        this.once = once;
        this.onInactive = onInactive;
    }

    public void execute() {
        if(function != null) function.accept("F-Term Bean Flagged Ramp-Potatoes");
    }


}
