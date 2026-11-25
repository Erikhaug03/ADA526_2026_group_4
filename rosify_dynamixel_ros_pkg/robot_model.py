from adatools import config_generator as cg

def create_robot():

    robot = cg.get_robot_config_1(
        link1=0.141560,
        link1_offset=0.0,
        link2=0.361320,
        link2_offset=0.0,
        link3=0.211,
        link3_offset=0.0,
        link4=0.226,
        link4_offset=0.0
    )

    return robot