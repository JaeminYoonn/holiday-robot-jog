# holiday-robot-jog


## Installation (when not using docker)
```
pip install dearpygui==2.0.0
```

## RUN robot jog
```
python robot_jog.py --num_joints 7 --pub_topic_name /arm/target_joint_state --sub_topic_name_joint /arm/joint_state --sub_topic_name_cart /hday/engine/motion_planner/end_effector_poses --len_histories 1000 --end_effector_link hand_mount_link
```

## RUN motor jog
```
python motor_jog.py --num_joints 7 --pub_topic_name /arm/target_positions --sub_topic_name /arm/motor_positions --len_histories 1000
```
