URDF examples

Link

<link name="my_link">
    <inertial>
        <mass value="${mass}" />
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="X" ixy="0.0" ixz="0.0" iyy="Y" iyz="0.0" izz="Z"/>
    </inertial>
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <cylinder length="A" radius="B"/>
            <!-- <box size="1 1 1" /> -->
        </geometry>
        <material name="White"/>
    </visual>
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <cylinder length="A" radius="B"/>
            <!-- <box size="1 1 1" /> -->
        </geometry>
    </collision>
</link>

Available joint types fixed, revolute, continuous, floating 
<joint name="my_joint" type="floating">
    <origin xyz="0 0 1" rpy="0 0 3.1416"/>
    <parent link="link1"/>
    <child link="link2"/>

    <calibration rising="0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
    <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
    <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
 </joint>
<joint name="${model_prefix}base_link_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="${model_prefix}base_footprint"/>
    <child link="${model_prefix}base_link"/>
</joint>

<joint name="${model_prefix}base_link_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="${model_prefix}base_footprint"/>
    <child link="${model_prefix}base_link"/>
</joint>

