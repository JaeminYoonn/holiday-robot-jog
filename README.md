# holiday-robot-jog


## Installation (when not using docker)
```
pip install dearpygui==2.0.0
```

## RUN robot jog
```
python joint_jog.py --num_joints 7 --pub_topic_name /arm/target_joint_state --sub_topic_name /arm/joint_state --len_histories 1000
```
