GRASP_SEQUENCE = [

    # Step 0: (Move to top of target pos)
    {
        "name": "Descend",
        "target": "cube", 
        "offset": [0.0, 0.0, 0.1], 
        "tol": 0.11, 
        "gripper": 1, 
        "wait_frames": 120
    },
    # Step 1: (Move to target pos)
    {
        "name": "Descend",
        "target": "cube", 
        "offset": [0.0, 0.0, 0.00], 
        "tol": 0.11, 
        "gripper": 1, 
        "wait_frames": 120
    },
    # Step 2: (Close gripper)
    {
        "name": "Grasp",
        "target": "cube", 
        "offset": [0.0, 0.0, 0.00], 
        "tol": 0.11, 
        "gripper": -1, 
        "wait_frames": 60
    },
    # Step 3: (Lift up)
    {
        "name": "Lift",
        "target": "cube", 
        "offset": [0.0, 0.0, 0.15], 
        "tol": 0.15, 
        "gripper": -1, 
        "wait_frames": 120
    },
    # Step 4: (Top of container)
    {
        "name": "MoveToContainer", 
        "target": "container",  
        "offset": [0.0, 0.0, 0.20], 
        "tol": 0.15, 
        "gripper": -1, 
        "wait_frames": 40
    },
    # Step 5: (Descend)
    {
        "name": "DescendToDrop", 
        "target": "container",
        "offset": [0.0, 0.0, 0.1], 
        "tol": 0.15, 
        "gripper": -1, 
        "wait_frames": 30
    },
    # Step 6: (Release)
    {
        "name": "Release", 
        "target": "container",
        "offset": [0.0, 0.0, 0.1], 
        "tol": 0.15, 
        "gripper": 1, 
        "wait_frames": 60
    },
    # Step 7: (Lift)
    {
        "name": "PostReleaseLift", 
        "target": "container",
        "offset": [0.0, 0.0, 0.25], 
        "tol": 0.15, 
        "gripper": 1, 
        "wait_frames": 120
    }
]