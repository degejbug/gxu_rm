colcon build --symlink-install
cmds=( 
	"ros2 launch hik_camera hik_camera.launch.py"
	)
for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
