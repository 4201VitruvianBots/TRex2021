package frc.vitruvianlib.utils;

import edu.wpi.first.wpilibj2.command.button.Button;

public class MultiButton extends Button {
    private Button[] m_buttons;

    public MultiButton(Button... buttons) {
        m_buttons = buttons;
    }

    @Override
    public boolean get() {
        for(Button b:m_buttons)
            if(!b.get())
                return false;

        return true;
    }
}
