# holiday-robot-jog


## Installation (when not using docker)
```
pip install dearpygui==2.0.0
```

## RUN robot jog
```
python robot_jog.py --num_joints 7 --pub_topic_name /arm/target_joint_state --sub_topic_name /arm/joint_state --len_histories 1000
```

## RUN motor jog
```
python motor_jog.py --num_joints 7 --pub_topic_name /arm/target_positions --sub_topic_name /arm/motor_positions --len_histories 1000
```
