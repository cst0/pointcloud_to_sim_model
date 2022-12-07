SDF_HEADER = """<?xml version='1.0'?>
<sdf version='1.4'>
    <model name="$MODEL_NAME">
    <!--
        This model was automatically generated.
        Don't attempt to edit it where you found it (it'll just get overwritten).
        If you want to keep/modify this file, it's safe to copy it to a different location instead.
        (If you're using the STL mode, though, you'll need to copy the STL file too. It's in a tmp directory.)
    -->
        <static>false</static>
"""

SDF_STL_LINK = """
        <link name='$NAME'>
            <pose>0 0 0 0 0 0</pose>
            <mass value="10"/>
            <inertial>
                ixx="1"
                ixy="1"
                ixz="1"
                iyy="1"
                iyz="1"
                izz="1"/>
            </inertial>
            <collision name='collision'>
                <geometry>
                    <mesh><uri>file://$FILE</uri></mesh>
                </geometry>
            </collision>
            <visual name='visual'>
                <material>
                    <ambient>0.5 0.5 0.5 1</ambient>
                    <diffuse>0.5 0.5 0.8 1</diffuse>
                    <specular>0.5 0.5 0.5 1</specular>
                    <emissive>0 0 0 1</emissive>
                </material>
                <geometry>
                    <mesh><uri>file://$FILE</uri></mesh>
                </geometry>
            </visual>
        </link>

"""

SDF_BOX_LINK = """
        <link name='$LINKNAME'>
            <pose>$X_POSE $Y_POSE $Z_POSE $ROLL_POSE $PITCH_POSE $YAW_POSE</pose>
            <mass value="10"/>
            <inertial>
                ixx="1"
                ixy="1"
                ixz="1"
                iyy="1"
                iyz="1"
                izz="1"/>
            </inertial>
            <collision name='collision'>
                <geometry>
                    <box>
                        <size>$X_SIZE $Y_SIZE $Z_SIZE</size>
                    </box>
                </geometry>
            </collision>
            <visual name='visual'>
                <geometry>
                    <box>
                        <size>$X_SIZE $Y_SIZE $Z_SIZE</size>
                    </box>
                </geometry>
            </visual>
        </link>
"""

SDF_BOX_JOINT = """
        <joint type="fixed" name="$CHILD_$PARENT">
            <pose>0 0 0 0 0 0</pose>
            <child>$CHILD</child>
            <parent>$PARENT</parent>
            <axis>
                <xyz>0 1 0</xyz>
            </axis>
        </joint>
"""

SDF_FOOTER = """
    </model>
</sdf>
"""

URDF_HEADER = """<?xml version='1.0'?>
<robot name="$MODEL_NAME">
"""

URDF_FOOTER = """
</robot>
"""
