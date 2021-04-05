#! /usr/bin/env python
from homodeus_bhvr_dialog import DialogBehavior

dialogbehavior = DialogBehavior()
print("My daughter had good grades in school, this week")
test = dialogbehavior._dialog_callback("My daughter had good grades in school, this week")
print("the weather shure is nice today")
test = dialogbehavior._dialog_callback("the weather shure is nice today")
print("You are a fat robot")
test = dialogbehavior._dialog_callback("You are a fat robot")
print('end')