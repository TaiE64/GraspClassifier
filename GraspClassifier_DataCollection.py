import pybullet as p
import pandas as pd
import numpy as np
import time
import random
import os
import pybullet_data
# # -------------------------------PATH---------------------------------------
# address = 'example/share/004_sugar_box/textured.obj'
# path = "data/experiments"
# ----------------------------------Saving Data------------------------------------
data = {'Ori_x': [],  # gripper orientation(base)
        'Ori_y': [],
        'Ori_z': [],
        'Ori_w': [],
        'Pos_x': [],  # gripper position(base)
        'Pos_y': [],
        'Pos_z': [],
        'Obj_Pos_x': [],  # object position
        'Obj_Pos_y': [],
        'Obj_Pos_z': [],
        'Label': []  # Good or Bad grasp (1 / 0)
        }
# ----------------------------------Define functions------------------------------------


def random_position(object_id, where):
    # Generate random position around object to grasp
    aabb = p.getAABB(object_id)
    min_bound, max_bound = aabb
    print(f'Object\'s Min Bound: {min_bound}')
    print(f'Object\'s Max Bound: {max_bound}')

    if where == 0:
        # noise for above(0)
        noise_1 = random.gauss(mu=0, sigma=0.3)*0.1  # x noise
        noise_2 = random.gauss(mu=0, sigma=0.3)*0.1  # y noise
        noise_3 = random.gauss(mu=0.25, sigma=0.1)  # z noise
        # Generate random position above the object (0)
        x_safe = (max_bound[0]+min_bound[0])/2+noise_1
        y_safe = (max_bound[1]+min_bound[1])/2+noise_2
        z_safe = max_bound[2]+noise_3

    elif where == 1:
        # noise for side(1)
        noise_1 = random.gauss(mu=0.2, sigma=0.1)  # x noise
        noise_2 = random.gauss(mu=0, sigma=0.4)*0.1  # y noiseA
        noise_3 = random.gauss(mu=0.25, sigma=0.1)*0.1  # z noise
        # Generate random position from the side(1)
        x_safe = min_bound[0]-noise_1
        y_safe = (max_bound[1]+min_bound[1])/2+noise_2
        z_safe = (max_bound[2]+min_bound[2])/2+noise_3

    elif where == 2:
        # noise for side(2)
        noise_1 = random.gauss(mu=0, sigma=0.3)*0.1  # x noise
        noise_2 = random.gauss(mu=0.2, sigma=0.1)  # y noiseA
        # z noise  (if gripper in contact with the table,it will deform ,so I set Z-noise positive)
        noise_3 = random.gauss(mu=0.2, sigma=0.1)*0.1
        # Generate random position from the side(2)
        x_safe = (max_bound[0]+min_bound[0])/2+noise_1
        y_safe = min_bound[1]-noise_2
        z_safe = (max_bound[2]+min_bound[2])/2+noise_3

    elif where == 3:
        # noise for side(3)
        noise_1 = random.gauss(mu=0, sigma=0.3)*0.1  # x noise
        noise_2 = random.gauss(mu=0.2, sigma=0.1)  # y noiseA
        noise_3 = random.gauss(mu=0.2, sigma=0.1)*0.1  # z noise
        # Generate random position from the side(3)
        x_safe = (max_bound[0]+min_bound[0])/2+noise_1
        y_safe = max_bound[1]+noise_2
        z_safe = (max_bound[2]+min_bound[2])/2+noise_3

    elif where == 4:
        # noise for side(4)
        noise_1 = random.gauss(mu=0.2, sigma=0.1)  # x noise
        noise_2 = random.gauss(mu=0, sigma=0.4)*0.1  # y noiseA
        noise_3 = random.gauss(mu=0.25, sigma=0.1)*0.1  # z noise
        # Generate random position from the side(4)
        x_safe = max_bound[0]+noise_1
        y_safe = (max_bound[1]+min_bound[1])/2+noise_2
        z_safe = (max_bound[2]+min_bound[2])/2+noise_3

    else:
        print("Please input number 0 ~ 4 to select the direction to generate random grasp")

    return [x_safe, y_safe, z_safe]


def update_gripper_direction(safe_position, gripperCID, where):
    # SELECT CORRESPONDING ORIENTATION OF GRASPING HERE (0~4) WHICH MATCHES POSITION IN FUNCTION random_position():

    # # -----Motion 1 : Flash-----
    # #Downward Pose
    # print(f"Random safe position: {safe_position}")
    # if where == 0:
    #   hand_ori = p.getQuaternionFromEuler([0,np.pi/2, 0])
    # elif where == 1:
    #   hand_ori = p.getQuaternionFromEuler([np.pi,0, 0])
    # elif where == 2:
    #   hand_ori = p.getQuaternionFromEuler([np.pi,0, np.pi/2])
    # elif where == 3:
    #   hand_ori = p.getQuaternionFromEuler([np.pi,0, -np.pi/2])
    # elif where == 4:
    #   hand_ori = p.getQuaternionFromEuler([0,0,np.pi])
    # p.changeConstraint(gripperCID, safe_position,jointChildFrameOrientation= hand_ori)
    # time.sleep(2)

    # ------Motion2 : Slowly move over (prevent collision with object)-----
    # Current gripper pos
    current_position = [0, 0, 3]
    if where == 0:
        hand_ori = p.getQuaternionFromEuler(
            [0, np.pi/2, 0])  # #above the object  (0)
    elif where == 1:
        hand_ori = p.getQuaternionFromEuler(
            [0, 0, 0])  # at side of the object (1)
    elif where == 2:
        hand_ori = p.getQuaternionFromEuler(
            [np.pi, 0, np.pi/2])  # at side of the object  (2)
    elif where == 3:
        hand_ori = p.getQuaternionFromEuler(
            [np.pi, 0, -np.pi/2])  # at side of the object (3)
    elif where == 4:
        hand_ori = p.getQuaternionFromEuler(
            [0, 0, np.pi])  # at side of the object  (4)
    steps = 80
    for step in range(steps):
        t = step / steps
        interpolated_position = [
            current_position[i] + (safe_position[i] - current_position[i]) * t for i in range(3)]
        # Update Constraint: Position and Direction
        p.changeConstraint(gripperCID, interpolated_position,
                           jointChildFrameOrientation=hand_ori)
        p.stepSimulation()
        time.sleep(0.01)
    print("Gripper position updated gradually.")
    time.sleep(0.4)


def grasp(gripperID):
    # close the gripper's fingers
    p.setJointMotorControl2(gripperID, 0, p.POSITION_CONTROL,
                            targetPosition=0, maxVelocity=2000, force=2000)
    p.setJointMotorControl2(gripperID, 2, p.POSITION_CONTROL,
                            targetPosition=0, maxVelocity=2000, force=2000)
    time.sleep(2)


def lifting(safe_position, gripperCID):  # CID = Constraint ID
    # Lift object up slowly
    print("Lifting....")
    for step in range(100):
        target_z = safe_position[2] + step * 0.005
        p.changeConstraint(
            gripperCID, [safe_position[0], safe_position[1], target_z])
        p.stepSimulation()
        time.sleep(0.01)
    print()


def reshape(gripperID):
    # reshape gripper
    p.setJointMotorControl2(gripperID, 0, p.POSITION_CONTROL,
                            targetPosition=0.05, maxVelocity=1, force=1)
    p.setJointMotorControl2(gripperID, 2, p.POSITION_CONTROL,
                            targetPosition=0.05, maxVelocity=1, force=1)


def getdata(object_id, gripperID):
    # collect data : position and orientation of gripper after grasp the object
    obj_pos, _ = p.getBasePositionAndOrientation(object_id)
    print(obj_pos)
    print()
    print('_______data_______')
    position, orientation = p.getBasePositionAndOrientation(gripperID)
    print(f'Position of gripper:{position}')
    print(f'Orientation of gripper:{orientation}')
    print(f'Position of object:{obj_pos}')
    print('_______data_______')
    print()
    return position, orientation, obj_pos


def is_successful(object_id, gripperID):
    # Label of the grasping : 0 or 1
    # if grasp can hold for 3 seconds
    Label = None
    print('Evaluating grasp quality....')
    time.sleep(3)
    contact_points = p.getContactPoints(bodyA=gripperID, bodyB=object_id)
    if len(contact_points) == 0:
        Label = 0
        print('Bad grasp :<')
    elif len(contact_points) >= 2:
        Label = 1
        print('Success! :>')
    else:
        print("Something went wrong...")
    print()
    return Label


def SaveData(position, orientation, label, obj_pos):
    global data
    data['Ori_x'].append(round(orientation[0], 6))
    data['Ori_y'].append(round(orientation[1], 6))
    data['Ori_z'].append(round(orientation[2], 6))
    data['Ori_w'].append(round(orientation[3], 6))
    data['Pos_x'].append(round(position[0], 6))
    data['Pos_y'].append(round(position[1], 6))
    data['Pos_z'].append(round(position[2], 6))
    data['Obj_Pos_x'].append(round(obj_pos[0], 6))
    data['Obj_Pos_y'].append(round(obj_pos[1], 6))
    data['Obj_Pos_z'].append(round(obj_pos[2], 6))
    data['Label'].append(label)
    file_path = "data.csv"
    df = pd.DataFrame(data)
    if os.path.exists(file_path):
        df.to_csv(file_path, mode="a", header=False, index=False)
        print("Data appended to existing file")
    else:
        df.to_csv(file_path, index=False)  # Create and write header
        print("File created and data saved")
    data = {key: [] for key in data.keys()}


def random_grasp_object(object_id, gripperID, gripperCID, where):
    # procedure of grasping and collect data

    # Generate random position around the object
    random_Pos = random_position(object_id, where)

    # update gripper Postion to random_Pos and predefined Orientation
    update_gripper_direction(random_Pos, gripperCID, where)

    # Reshape pos, get ready for grasping
    reshape(gripperID)

    # Grasp object at random position
    grasp(gripperID)

    # Get base link's(wrist of the gripper,link 0) Position and Orientation
    position, orientation, obj_pos = getdata(object_id, gripperID)

    # Lifting object
    lifting(random_Pos, gripperCID)

    label = is_successful(object_id, gripperID)

    SaveData(position, orientation, label, obj_pos)

    # print(data) #check data


# set_camera
def set_camera():
    camera_distance = 2
    camera_yaw = 45
    camera_pitch = -30
    target_position = [0.0, -0.4, 0.5]

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=target_position
    )


# Run with GUI
def run_GUI_Once(where):
    # --------------------Initialization Start--------------------
    cid = p.connect(p.SHARED_MEMORY)
    if (cid < 0):
        p.connect(p.GUI)
    p.resetSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    initial_position = [0, 0, 3]
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
    objects = [p.loadURDF("urdf/Gripper/pr2_gripper.urdf",
                          initial_position, initial_orientation)]
    pr2_gripper = objects[0]  # Gripper ID
    # Loading Table's urdf
    table = p.loadURDF("urdf/Table/table.urdf",
                       [0.0, -0.4, 0], [0, 0, 0, 1], 0)
    # Loading object
    objID = p.loadURDF("urdf/Object/cube_small.urdf",
                       [0.0, -0.4, 0.62], globalScaling=2)
    jointPositions = [3, 0.000000, 3, 0.000000]
    for jointIndex in range(p.getNumJoints(pr2_gripper)):
        p.resetJointState(pr2_gripper, jointIndex, jointPositions[jointIndex])
    # Create constraint of gripper Gripper CID
    pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                 [0, 0, 0])
    # --------------------Initialization End--------------------

    # --------------------Start Simulation--------------------
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -10)
    os.system('cls')  # Clear terminal
    print("------------------------------Start------------------------------")
    random_grasp_object(objID, pr2_gripper, pr2_cid, where)
    p.stepSimulation()
    print("------------------------------End------------------------------")
    time.sleep(2)
    input("Enter to quit")
    p.disconnect()


# No GUI DataCollection
def NoGUI_DataCollection(where, batch_size=100):
    # --------------------Initialization Start--------------------
    # Shut down GUI
    for i in range(batch_size):
        cid = p.connect(p.DIRECT)
        if cid < 0:
            raise RuntimeError("Failed to connect to PyBullet in DIRECT mode.")
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        initial_position = [0, 0, 3]
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
        objects = [p.loadURDF("urdf/Gripper/pr2_gripper.urdf",
                              initial_position, initial_orientation)]
        pr2_gripper = objects[0]  # Gripper ID
        # Loading Table's urdf
        table = p.loadURDF("urdf/Table/table.urdf",
                           [0.0, -0.4, 0], [0, 0, 0, 1], 0)
        # Loading object
        objID = p.loadURDF("urdf/Object/cube_small.urdf",
                           [0.0, -0.4, 0.62], globalScaling=2)
        jointPositions = [3, 0.000000, 3, 0.000000]
        for jointIndex in range(p.getNumJoints(pr2_gripper)):
            p.resetJointState(pr2_gripper, jointIndex,
                              jointPositions[jointIndex])
        # Create constraint of gripper
        pr2_cid = p.createConstraint(
            pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        # --------------------Initialization End--------------------

        # --------------------Start Data Colletion--------------------

        os.system('cls')  # Clear terminal
        print(f"--- Collecting Cample {i + 1} / {batch_size} ---")
        random_grasp_object(objID, pr2_gripper, pr2_cid, where)
        p.stepSimulation()
        print("--- Data Collection Completed ---")
        time.sleep(2)
        p.disconnect()


# GUI DataCollection
def GUI_DataCollection(where, batch_size=100):  # Recommend!
    # --------------------Initialization Start--------------------
    for i in range(batch_size):
        cid = p.connect(p.SHARED_MEMORY)
        if (cid < 0):
            p.connect(p.GUI)
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        initial_position = [0, 0, 3]
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
        objects = [p.loadURDF("urdf/Gripper/pr2_gripper.urdf",
                              initial_position, initial_orientation)]
        pr2_gripper = objects[0]  # Gripper ID
        # Loading Table's urdf
        table = p.loadURDF("urdf/Table/table.urdf",
                           [0.0, -0.4, 0], [0, 0, 0, 1], 0)
        # Loading object
        objID = p.loadURDF("urdf/Object/cube_small.urdf",
                           [0.0, -0.4, 0.62], globalScaling=2)
        jointPositions = [3, 0.000000, 3, 0.000000]
        for jointIndex in range(p.getNumJoints(pr2_gripper)):
            p.resetJointState(pr2_gripper, jointIndex,
                              jointPositions[jointIndex])
        # Create constraint of gripper Gripper CID
        pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                     [0, 0, 0])
        set_camera()
        # --------------------Initialization End--------------------

        # --------------------Start Simulation--------------------
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.setRealTimeSimulation(1)
        p.setGravity(0, 0, -10)
        os.system('cls')  # Clear terminal
        print("------------------------------Start------------------------------")
        print(f"--- Collecting Cample {i + 1} / {batch_size} ---")
        random_grasp_object(objID, pr2_gripper, pr2_cid, where)
        p.stepSimulation()
        print("------------------------------End------------------------------")
        time.sleep(1)
        p.disconnect()


# ----------------------------------Run------------------------------------
# run_GUI_Once(2)
NoGUI_DataCollection(1, batch_size=100)
# GUI_DataCollection(0,50)
# ----------------------------------------------------------------------
