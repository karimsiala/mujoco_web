// These are Model related enums taken from mjmodel.h MuJoCo codebase.

/** Disable default feature bitflags */
export enum mjtDisableBit {
    mjDSBL_CONSTRAINT = 1 << 0, // entire constraint solver
    mjDSBL_EQUALITY = 1 << 1, // equality constraints
    mjDSBL_FRICTIONLOSS = 1 << 2, // joint and tendon frictionloss constraints
    mjDSBL_LIMIT = 1 << 3, // joint and tendon limit constraints
    mjDSBL_CONTACT = 1 << 4, // contact constraints
    mjDSBL_PASSIVE = 1 << 5, // passive forces
    mjDSBL_GRAVITY = 1 << 6, // gravitational forces
    mjDSBL_CLAMPCTRL = 1 << 7, // clamp control to specified range
    mjDSBL_WARMSTART = 1 << 8, // warmstart constraint solver
    mjDSBL_FILTERPARENT = 1 << 9, // remove collisions with parent body
    mjDSBL_ACTUATION = 1 << 10, // apply actuation forces
    mjDSBL_REFSAFE = 1 << 11, // integrator safety: make ref[0]>=2*timestep
    mjDSBL_SENSOR = 1 << 12, // sensors
    mjDSBL_MIDPHASE = 1 << 13, // mid-phase collision filtering
    mjDSBL_EULERDAMP = 1 << 14, // implicit integration of joint damping in Euler integrator
    mjDSBL_AUTORESET = 1 << 15, // automatic reset when numerical issues are detected
    mjNDISABLE = 16 // number of disable flags
}

/** Enable optional feature bitflags */
export enum mjtEnableBit {
    mjENBL_OVERRIDE = 1 << 0, // override contact parameters
    mjENBL_ENERGY = 1 << 1, // energy computation
    mjENBL_FWDINV = 1 << 2, // record solver statistics
    mjENBL_INVDISCRETE = 1 << 3, // discrete-time inverse dynamics
    // experimental features:
    mjENBL_MULTICCD = 1 << 4, // multi-point convex collision detection
    mjENBL_ISLAND = 1 << 5, // constraint island discovery
    mjENBL_NATIVECCD = 1 << 6, // native convex collision detection
    mjNENABLE = 7 // number of enable flags
}

/** Type of degree of freedom */
export enum mjtJoint {
    mjJNT_FREE = 0, // global position and orientation (quat)       (7)
    mjJNT_BALL, // orientation (quat) relative to parent        (4)
    mjJNT_SLIDE, // sliding distance along body-fixed axis       (1)
    mjJNT_HINGE // rotation angle (rad) around body-fixed axis  (1)
}

/** Type of geometric shape */
export enum mjtGeom {
    // regular geom types
    mjGEOM_PLANE = 0, // plane
    mjGEOM_HFIELD, // height field
    mjGEOM_SPHERE, // sphere
    mjGEOM_CAPSULE, // capsule
    mjGEOM_ELLIPSOID, // ellipsoid
    mjGEOM_CYLINDER, // cylinder
    mjGEOM_BOX, // box
    mjGEOM_MESH, // mesh
    mjGEOM_SDF, // signed distance field
    mjNGEOMTYPES, // number of regular geom types
    // rendering-only geom types: not used in mjModel, not counted in mjNGEOMTYPES
    mjGEOM_ARROW = 100, // arrow
    mjGEOM_ARROW1, // arrow without wedges
    mjGEOM_ARROW2, // arrow in both directions
    mjGEOM_LINE, // line
    mjGEOM_LINEBOX, // box with line edges
    mjGEOM_FLEX, // flex
    mjGEOM_SKIN, // skin
    mjGEOM_LABEL, // text label
    mjGEOM_TRIANGLE, // triangle
    mjGEOM_NONE = 1001 // missing geom type
}

/** Tracking mode for camera and light */
export enum mjtCamLight {
    mjCAMLIGHT_FIXED = 0, // pos and rot fixed in body
    mjCAMLIGHT_TRACK, // pos tracks body, rot fixed in global
    mjCAMLIGHT_TRACKCOM, // pos tracks subtree com, rot fixed in body
    mjCAMLIGHT_TARGETBODY, // pos fixed in body, rot tracks target body
    mjCAMLIGHT_TARGETBODYCOM // pos fixed in body, rot tracks target subtree com
}

/** Type of texture */
export enum mjtTexture {
    mjTEXTURE_2D = 0, // 2d texture, suitable for planes and hfields
    mjTEXTURE_CUBE, // cube texture, suitable for all other geom types
    mjTEXTURE_SKYBOX // cube texture used as skybox
}

/** Role of texture map in rendering */
export enum mjtTextureRole {
    mjTEXROLE_USER = 0, // unspecified
    mjTEXROLE_RGB, // base color (albedo)
    mjTEXROLE_OCCLUSION, // ambient occlusion
    mjTEXROLE_ROUGHNESS, // roughness
    mjTEXROLE_METALLIC, // metallic
    mjTEXROLE_NORMAL, // normal (bump) map
    mjTEXROLE_OPACITY, // transperancy
    mjTEXROLE_EMISSIVE, // light emission
    mjTEXROLE_RGBA, // base color, opacity
    mjTEXROLE_ORM, // occlusion, roughness, metallic
    mjNTEXROLE
}

/** Integrator mode */
export enum mjtIntegrator {
    mjINT_EULER = 0, // semi-implicit Euler
    mjINT_RK4, // 4th-order Runge Kutta
    mjINT_IMPLICIT, // implicit in velocity
    mjINT_IMPLICITFAST // implicit in velocity, no rne derivative
}

/** Type of friction cone */
export enum mjtCone {
    mjCONE_PYRAMIDAL = 0, // pyramidal
    mjCONE_ELLIPTIC // elliptic
}

/** Type of constraint Jacobian */
export enum mjtJacobian {
    mjJAC_DENSE = 0, // dense
    mjJAC_SPARSE, // sparse
    mjJAC_AUTO // dense if nv<60, sparse otherwise
}

/** Constraint solver algorithm */
export enum mjtSolver {
    mjSOL_PGS = 0, // PGS (dual)
    mjSOL_CG, // CG (primal)
    mjSOL_NEWTON // Newton (primal)
}

/** Type of equality constraint */
export enum mjtEq {
    mjEQ_CONNECT = 0, // connect two bodies at a point (ball joint)
    mjEQ_WELD, // fix relative position and orientation of two bodies
    mjEQ_JOINT, // couple the values of two scalar joints with cubic
    mjEQ_TENDON, // couple the lengths of two tendons with cubic
    mjEQ_FLEX, // fix all edge lengths of a flex
    mjEQ_DISTANCE // unsupported, will cause an error if used
}

/** Type of tendon wrap object */
export enum mjtWrap {
    mjWRAP_NONE = 0, // null object
    mjWRAP_JOINT, // constant moment arm
    mjWRAP_PULLEY, // pulley used to split tendon
    mjWRAP_SITE, // pass through site
    mjWRAP_SPHERE, // wrap around sphere
    mjWRAP_CYLINDER // wrap around (infinite) cylinder
}

/** Type of actuator transmission */
export enum mjtTrn {
    mjTRN_JOINT = 0, // force on joint
    mjTRN_JOINTINPARENT, // force on joint, expressed in parent frame
    mjTRN_SLIDERCRANK, // force via slider-crank linkage
    mjTRN_TENDON, // force on tendon
    mjTRN_SITE, // force on site
    mjTRN_BODY, // adhesion force on a body's geoms
    mjTRN_UNDEFINED = 1000 // undefined transmission type
}

/** Type of actuator dynamics */
export enum mjtDyn {
    mjDYN_NONE = 0, // no internal dynamics; ctrl specifies force
    mjDYN_INTEGRATOR, // integrator: da/dt = u
    mjDYN_FILTER, // linear filter: da/dt = (u-a) / tau
    mjDYN_FILTEREXACT, // linear filter: da/dt = (u-a) / tau, with exact integration
    mjDYN_MUSCLE, // piece-wise linear filter with two time constants
    mjDYN_USER // user-defined dynamics type
}

/** Type of actuator gain                   */
export enum mjtGain {
    mjGAIN_FIXED = 0, // fixed gain
    mjGAIN_AFFINE, // const + kp*length + kv*velocity
    mjGAIN_MUSCLE, // muscle FLV curve computed by mju_muscleGain()
    mjGAIN_USER // user-defined gain type
}

/** Type of actuator bias */
export enum mjtBias {
    mjBIAS_NONE = 0, // no bias
    mjBIAS_AFFINE, // const + kp*length + kv*velocity
    mjBIAS_MUSCLE, // muscle passive force computed by mju_muscleBias()
    mjBIAS_USER // user-defined bias type
}

/** Type of MujoCo object */
export enum mjtObj {
    mjOBJ_UNKNOWN = 0, // unknown object type
    mjOBJ_BODY, // body
    mjOBJ_XBODY, // body, used to access regular frame instead of i-frame
    mjOBJ_JOINT, // joint
    mjOBJ_DOF, // dof
    mjOBJ_GEOM, // geom
    mjOBJ_SITE, // site
    mjOBJ_CAMERA, // camera
    mjOBJ_LIGHT, // light
    mjOBJ_FLEX, // flex
    mjOBJ_MESH, // mesh
    mjOBJ_SKIN, // skin
    mjOBJ_HFIELD, // heightfield
    mjOBJ_TEXTURE, // texture
    mjOBJ_MATERIAL, // material for rendering
    mjOBJ_PAIR, // geom pair to include
    mjOBJ_EXCLUDE, // body pair to exclude
    mjOBJ_EQUALITY, // equality constraint
    mjOBJ_TENDON, // tendon
    mjOBJ_ACTUATOR, // actuator
    mjOBJ_SENSOR, // sensor
    mjOBJ_NUMERIC, // numeric
    mjOBJ_TEXT, // text
    mjOBJ_TUPLE, // tuple
    mjOBJ_KEY, // keyframe
    mjOBJ_PLUGIN, // plugin instance
    mjNOBJECT, // number of object types
    // meta elements, do not appear in mjModel
    mjOBJ_FRAME = 100 // frame
}

/** Type of constraint */
export enum mjtConstraint {
    mjCNSTR_EQUALITY = 0, // equality constraint
    mjCNSTR_FRICTION_DOF, // dof friction
    mjCNSTR_FRICTION_TENDON, // tendon friction
    mjCNSTR_LIMIT_JOINT, // joint limit
    mjCNSTR_LIMIT_TENDON, // tendon limit
    mjCNSTR_CONTACT_FRICTIONLESS, // frictionless contact
    mjCNSTR_CONTACT_PYRAMIDAL, // frictional contact, pyramidal friction cone
    mjCNSTR_CONTACT_ELLIPTIC // frictional contact, elliptic friction cone
}

/** Constraint state                        */
export enum mjtConstraintState {
    mjCNSTRSTATE_SATISFIED = 0, // constraint satisfied, zero cost (limit, contact)
    mjCNSTRSTATE_QUADRATIC, // quadratic cost (equality, friction, limit, contact)
    mjCNSTRSTATE_LINEARNEG, // linear cost, negative side (friction)
    mjCNSTRSTATE_LINEARPOS, // linear cost, positive side (friction)
    mjCNSTRSTATE_CONE // squared distance to cone cost (elliptic contact)
}

/** Type of sensor                          */
export enum mjtSensor {
    // common robotic sensors, attached to a site
    mjSENS_TOUCH = 0, // scalar contact normal forces summed over sensor zone
    mjSENS_ACCELEROMETER, // 3D linear acceleration, in local frame
    mjSENS_VELOCIMETER, // 3D linear velocity, in local frame
    mjSENS_GYRO, // 3D angular velocity, in local frame
    mjSENS_FORCE, // 3D force between site's body and its parent body
    mjSENS_TORQUE, // 3D torque between site's body and its parent body
    mjSENS_MAGNETOMETER, // 3D magnetometer
    mjSENS_RANGEFINDER, // scalar distance to nearest geom or site along z-axis
    mjSENS_CAMPROJECTION, // pixel coordinates of a site in the camera image
    // sensors related to scalar joints, tendons, actuators
    mjSENS_JOINTPOS, // scalar joint position (hinge and slide only)
    mjSENS_JOINTVEL, // scalar joint velocity (hinge and slide only)
    mjSENS_TENDONPOS, // scalar tendon position
    mjSENS_TENDONVEL, // scalar tendon velocity
    mjSENS_ACTUATORPOS, // scalar actuator position
    mjSENS_ACTUATORVEL, // scalar actuator velocity
    mjSENS_ACTUATORFRC, // scalar actuator force
    mjSENS_JOINTACTFRC, // scalar actuator force, measured at the joint
    // sensors related to ball joints
    mjSENS_BALLQUAT, // 4D ball joint quaternion
    mjSENS_BALLANGVEL, // 3D ball joint angular velocity
    // joint and tendon limit sensors, in constraint space
    mjSENS_JOINTLIMITPOS, // joint limit distance-margin
    mjSENS_JOINTLIMITVEL, // joint limit velocity
    mjSENS_JOINTLIMITFRC, // joint limit force
    mjSENS_TENDONLIMITPOS, // tendon limit distance-margin
    mjSENS_TENDONLIMITVEL, // tendon limit velocity
    mjSENS_TENDONLIMITFRC, // tendon limit force
    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    mjSENS_FRAMEPOS, // 3D position
    mjSENS_FRAMEQUAT, // 4D unit quaternion orientation
    mjSENS_FRAMEXAXIS, // 3D unit vector: x-axis of object's frame
    mjSENS_FRAMEYAXIS, // 3D unit vector: y-axis of object's frame
    mjSENS_FRAMEZAXIS, // 3D unit vector: z-axis of object's frame
    mjSENS_FRAMELINVEL, // 3D linear velocity
    mjSENS_FRAMEANGVEL, // 3D angular velocity
    mjSENS_FRAMELINACC, // 3D linear acceleration
    mjSENS_FRAMEANGACC, // 3D angular acceleration
    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    mjSENS_SUBTREECOM, // 3D center of mass of subtree
    mjSENS_SUBTREELINVEL, // 3D linear velocity of subtree
    mjSENS_SUBTREEANGMOM, // 3D angular momentum of subtree
    // sensors for geometric distance; attached to geoms or bodies
    mjSENS_GEOMDIST, // signed distance between two geoms
    mjSENS_GEOMNORMAL, // normal direction between two geoms
    mjSENS_GEOMFROMTO, // segment between two geoms
    // global sensors
    mjSENS_CLOCK, // simulation time
    // plugin-controlled sensors
    mjSENS_PLUGIN, // plugin-controlled
    // user-defined sensor
    mjSENS_USER // sensor data provided by mjcb_sensor callback
}

/** Computation stage */
export enum mjtStage {
    mjSTAGE_NONE = 0, // no computations
    mjSTAGE_POS, // position-dependent computations
    mjSTAGE_VEL, // velocity-dependent computations
    mjSTAGE_ACC // acceleration/force-dependent computations
}

/** Data type for sensors  */
export enum mjtDataType {
    mjDATATYPE_REAL = 0, // real values, no constraints
    mjDATATYPE_POSITIVE, // positive values; 0 or negative: inactive
    mjDATATYPE_AXIS, // 3D unit vector
    mjDATATYPE_QUATERNION // unit quaternion
}

export enum mjtSameFrame { // frame alignment of bodies with their children
    mjSAMEFRAME_NONE = 0, // no alignment
    mjSAMEFRAME_BODY, // frame is same as body frame
    mjSAMEFRAME_INERTIA, // frame is same as inertial frame
    mjSAMEFRAME_BODYROT, // frame orientation is same as body orientation
    mjSAMEFRAME_INERTIAROT // frame orientation is same as inertia orientation
}

/** Mode for actuator length range computation */
export enum mjtLRMode {
    mjLRMODE_NONE = 0, // do not process any actuators
    mjLRMODE_MUSCLE, // process muscle actuators
    mjLRMODE_MUSCLEUSER, // process muscle and user actuators
    mjLRMODE_ALL // process all actuators
}

export enum mjtFlexSelf_ { // mode for flex selfcollide
    mjFLEXSELF_NONE = 0, // no self-collisions
    mjFLEXSELF_NARROW, // skip midphase, go directly to narrowphase
    mjFLEXSELF_BVH, // use BVH in midphase (if midphase enabled)
    mjFLEXSELF_SAP, // use SAP in midphase
    mjFLEXSELF_AUTO // choose between BVH and SAP automatically
}
