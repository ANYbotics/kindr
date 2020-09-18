# Kindr 1.0.0 - Notes

# Changes from Kindr 0.0.1

* C_BI.boxPlus(dt * B_w_IB) ==> C_BI.boxPlus(dt * C_IB * B_w_IB)
* C_BI.boxMinus(dt *  B_w_IB) ==>  -C_BI.boxMinus(dt * B_w_IB)
* setFromVectors(v1, v2) ==> setFromVectors(v2, v1) = setFromVectors(v1, v2).invert()
* EulerAnglesZyx(z, y, x) ==> EulerAnglesZyx(-z, -y,-x)
* EulerAnglesXyz(x, y, z) ==> EulerAnglesXyz(-x, -y,-z)

# Kindr 1.0.0
* Convention: Hamilton, passive

# Gazebo

* Convention: Hamilton, passive
* Convention tests can be found in the unit tests of package kindr_gazebo in repo any_gazebo.
* Gazebos's gazebo::math::Quaternion has the same convention as kindr.
* Gazebo's output:
    - (I_r, C_IB) = link->GetWorldPose(); 	(position expressed in inertial frame, rotation from base to interial frame)
    - B_v = link->GetRelativeLinearVel(); 	(linear velocity in base frame
    - B_w_IB = link->GetRelativeAngularVel(); 	(local angular velocity) 

# ROS TF, Rviz

* Convention: Hamilton, passive, RViz wants TF message with rotation C_IB
* Convention tests can be found in the unit tests of package kindr_ros in repo kindr_ros.
* tf::Quaternion has the same convention as kindr.
* tf::Matrix3x3 (rotation matrix) has the same convention as kindr
* tf::Transform behaves the same as kindr::HomogeneousTransformationPosition3RotationQuaternionD
    
    
# RBDL

* Convention: not consistent (mostly Hamiltonian)
* Conversion tests can be found in the unit tests of package rbdl in the monorepo.
* RBDL's RigidBodyDynamics::Math::Quaternion has the same convention as kindr, but the resulting rotation matrix from toMatrix() is inverted!
* RBDL's rotation of RigidBodyDynamics::Math::SpatialTransform has the same convention as kindr
* Attention: RBDL inverts the quaternion in the generalized coordinates!

## Calculations in RBDL

### Transformation X_base
std::vector<Math::SpatialTransform> X_base
Comments in code: "Transformation from the base to bodies reference frame."

According to *CalcBodyToBaseCoordinates* (see below), the following position and orientation is stored:

X_base.r = W_r_WB
X_base.E = C_BW


### CalcBodyToBaseCoordinates
Base Coordinates = World Coordinates
Comments in code: 
* "Returns the base coordinates of a point given in body coordinates.
* "3-D vector with coordinates of the point in base coordinates"

The calc should be: W_r_WP = W_r_WB + C_WB*B_r_BP

The code says:

```
  Matrix3d body_rotation = model.X_base[body_id].E.transpose();
  Vector3d body_position = model.X_base[body_id].r; 
  return body_position + body_rotation * point_body_coordinates;
```

X_base.E should correspond to C_BW


### CalcBaseToBodyCoordinates 

* "Returns the body coordinates of a point given in base coordinates."
* "Returns a 3-D vector with coordinates of the point in body coordinates"

B_r_BP = C_BW * (W_r_WP - W_r_WB)

```
 Matrix3d body_rotation = model.X_base[body_id].E;
 Vector3d body_position = model.X_base[body_id].r;
 return body_rotation * (point_base_coordinates - body_position);
```





  

