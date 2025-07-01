# Identifier of the robot that publishes the message.
int16 emitter

# Identifier of the robot that has to process the message.
int16 receiver

# Robot to camera transformation.
geometry_msgs/Pose robot2camera

# Keyframe timestamps.
float64[] keyframe_stamps

# Local cluster poses.
geometry_msgs/Pose[] local_cluster_poses

# Cluster-keyframe relation.
int32[] cluster_keyframe_relations

# Graph (all poses and transformations are relative to the camera axis).
Link[] links
Node[] nodes

# Inter-loop closures.
Link[] inter_loop_closures