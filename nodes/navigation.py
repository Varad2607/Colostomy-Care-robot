"""
Navigation Node
    + Based on the user's singnal, start navigation of robot to the desired location

    + navigate_to_patient
        - send /initialpose to navigation for localization
        - send /amcl_pose as the goal location & orientation
    
    + navigate_to_bin
        - send /amcl_pose as the goal location & orientation
        - send singal to saved-poses to perform thowing away
    
    + navigate_to_initial
        - return to the inital position
"""