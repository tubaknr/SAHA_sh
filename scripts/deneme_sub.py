#!/usr/bin/env python3

import rospy 
from msg_pub_sub.msg import xyz
from enum import Enum


def add_quotes(var_value):
    """
    Adds double quotes to beginning and ending of given value.
    """
    quoted = '"' + var_value + '"'
    return quoted

def double_quoted(var_value):
    """
    Adds double and single quotes together to beginning 
    and ending of given value.
    Used for coordinate data type, which is described in xyz.msg file.
    """
    double_quoted = add_quotes("'" + (var_value) + "'")
    return double_quoted

def single_quotes(var_value):
    """
    Adds single quotes to beginning and ending of given value.
    """
    quoted = "'" + var_value + "'"
    return quoted


def data_type(msg):
    """
    Determines data type of the variable's value. 
    Executes the necessary operations related to that type. 
    For example, adding corresponding quotes, etc.
    """
    if msg.var_type.lower() == "str" or msg.var_type.lower() == "string":
        msg.var_value = single_quotes(msg.var_value)
        msg.var_type = xyz.STRING 
        print("value:", msg.var_value)
        print("type:", msg.var_type)

    elif msg.var_type.lower() == "int" or msg.var_type.lower() == "integer":
        msg.var_value = int(msg.var_value)
        msg.var_type = xyz.INT
        print("value:", msg.var_value)
        print("type:", msg.var_type)

    elif msg.var_type.lower() == "float" or msg.var_type.lower() == "double":
        msg.var_value = single_quotes(str(float(msg.var_value)))
        msg.var_type = xyz.FLOAT
        print("value:", msg.var_value)
        print("type:", msg.var_type)

    elif msg.var_type.lower() == "bool" or msg.var_type.lower() == "boolean":
        msg.var_value = single_quotes(str(msg.var_value).lower())
        msg.var_type = xyz.BOOL
        print("value:", msg.var_value)
        print("type:", msg.var_type)

    elif msg.var_type.lower() == "coord" or msg.var_type.lower() == "coordinate" or msg.var_type.lower() == "koordinat":
        msg.var_type = xyz.COORDINATE
        msg.var_value = double_quoted(msg.var_value)
        print("value:", msg.var_value)
        print("type:", msg.var_type)

    return [msg.var_type, msg.var_value]


def save_to_sh(msg):
    """
    Saves the vairable to vars.sh file. 
    Saving format: export var_name = var_value.
    Variable's value is taken from data_type() function which 
    justifies the format of the value of that variable.
    """
    path = "vars.sh"
    old_var = False
    var_name1 = msg.var_name

    with open(path,"r") as f:
        lines = f.readlines()
    
    #check whether it has already declared
    for line in lines:
        if var_name1 in line:
            old_var=True
            print("OLD VARIABLE!")
            break

    if old_var: #OLD VAR
        for index,line in enumerate(lines, start=1):
            if var_name1 in line:
                #get the number of the line which should be updated
                line_num = index
                
        lines[line_num-1] = "export " + var_name1 + " = " + str(data_type(msg)[1]) + "\n"

        with open(path, "w") as f:
            f.writelines(lines)

    else: #NEW VAR
        with open(path, "a") as f:
            var_value = str(data_type(msg)[1])
            f.write(f'export {var_name1} = {var_value}\n')



def callback(msg):
    """
    ROS callback function.
    Also prints the entered variable on terminal.
    """
    rospy.loginfo(f"entered -> name: {msg.var_name}, value: {msg.var_value}, type: {msg.var_type}")
    save_to_sh(msg)
    print("saved!")


#Subscriber
def sub():
    rospy.init_node("sub01", anonymous=True)
    rospy.Subscriber("topic01", xyz, callback)
    rospy.spin()



if __name__ == "__main__":
    sub()



