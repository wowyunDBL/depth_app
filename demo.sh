roslaunch depth_app hector_mower.launch
python3 /home/ncslaber/realSense/catkin_ws/src/plot_utils/scripts/find_trunk.py
python3 /home/ncslaber/cpp/path_planning.py
python3 /home/ncslaber/cpp/plot_pickle.py
