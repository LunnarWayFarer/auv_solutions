"""Четыре движителя учебной сцены в штатной точке SUB_FRAME_CUSTOM."""
from pathlib import Path
import sys

source = Path(sys.argv[1])/'libraries/AP_Motors/AP_Motors6DOF.cpp'
text = source.read_text()
old = '''    case SUB_FRAME_CUSTOM:
        // Put your custom motor setup here
        //break;'''
new = '''    case SUB_FRAME_CUSTOM:
        _frame_class_string = "PROJECT2600_4";
        add_motor_raw_6dof(AP_MOTORS_MOT_1, 0,  0,  1,  0, 1, 0, 1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2, 0,  0, -1,  0, 1, 0, 2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3, 0, -1,  0, -1, 0, 0, 3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4, 0,  1,  0, -1, 0, 0, 4);
        break;'''
if text.count(old) != 1:
    raise RuntimeError('Review upstream custom mixer before patching')
source.write_text(text.replace(old, new))
