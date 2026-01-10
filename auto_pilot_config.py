GRASP_SEQUENCE = [

    # Step 0: (Move to top of target pos)
    {
        "name": "Descend", 
        "offset": [0.0, 0.0, 0.1], 
        "tol": 0.11, 
        "gripper": 1, 
        "wait_frames": 20
    },
    # Step 1: (Move to target pos)
    {
        "name": "Descend", 
        "offset": [0.0, 0.0, 0.00], 
        "tol": 0.11, 
        "gripper": 1, 
        "wait_frames": 20
    },
    # Step 2: (Close gripper)
    {
        "name": "Grasp", 
        "offset": [0.0, 0.0, 0.00], 
        "tol": 0.11, 
        "gripper": -1, 
        "wait_frames": 60
    },
    # Step 3: (Lift up)
    {
        "name": "Lift", 
        "offset": [0.0, 0.0, 0.15], 
        "tol": 0.15, 
        "gripper": -1, 
        "wait_frames": 120
    }
]