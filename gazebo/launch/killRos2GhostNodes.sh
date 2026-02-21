# finds and pkills all ros2 nodes by name, because `ros2 daemon stop` does not always work
ros2 daemon stop
pkill python3
ros2nodes=($(ros2 node list))
IFS='/'
for node in "${ros2nodes[@]}"; do
    read -ra nodeNameParts <<< "$node" # split by /
    nodeLastName="${nodeNameParts[-1]}"
    nodeProcName="${node:1:15}";
    echo "Found node $node, killing $nodeProcName";
    pkill nodeProcName;
done
pkill sim_state_trans; # for some reason roscopter_truth has a unique name
