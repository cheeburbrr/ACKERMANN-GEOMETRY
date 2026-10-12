# ACKERMANN-GEOMETRY
Building a rover and turning it with the help of ackermann geometry and using steering actuators


function sysCall_init()

sim = require('sim')

    --------------------------------------------------
    -- ROBOT OBJECTS
    --------------------------------------------------

base = sim.getObject('/base_link_respondable')

flSteer = sim.getObject('/front_left_steering_joint')
frSteer = sim.getObject('/front_right_steering_joint')

  flWheel = sim.getObject('/front_left_wheel_joint')
  frWheel = sim.getObject('/front_right_wheel_joint')
  rlWheel = sim.getObject('/rear_left_wheel_joint')
   rrWheel = sim.getObject('/rear_right_wheel_joint')

    --------------------------------------------------
    -- SETTINGS
    --------------------------------------------------

  wheelRadius = 0.06
    speed = 0.20
    maxSteer = math.rad(25)

  steerSign = 1
    driveSign = 1

    --------------------------------------------------
    -- TARGET IN INITIAL ROBOT COORDINATES
    --------------------------------------------------

   xTarget = 2.0
    yTarget = 0.0

    --------------------------------------------------
    -- INITIAL POSITION AND HEADING
    --------------------------------------------------

  startPos = sim.getObjectPosition(base, -1)

  local o = sim.getObjectOrientation(base, -1)
    startHeading = o[3]

    --------------------------------------------------
    -- WHEEL GEOMETRY
    -- Assumes local +X is forward and +Y is left.
    --------------------------------------------------

  local function pos(h)
        return sim.getObjectPosition(h, base)
    end

   local pFL = pos(flSteer)
    local pFR = pos(frSteer)
    local pRL = pos(rlWheel)
    local pRR = pos(rrWheel)

   L = math.abs(
        (pFL[1] + pFR[1]) / 2
        - (pRL[1] + pRR[1]) / 2
    )

   W = math.abs(pFL[2] - pFR[2])

   if L < 0.001 or W < 0.001 then
        error('Invalid wheelbase or track width')
    end
    
   print(string.format(
        'Wheelbase: %.3f m | Track width: %.3f m',
        L, W
    ))

   targetReached = false

end


function sysCall_actuation()

   if targetReached then
        stopRobot()
        setSteering(0, 0)
        return
    end

    --------------------------------------------------
    -- TARGET POSITION IN WORLD COORDINATES
    --------------------------------------------------

  local targetWorldX =
        startPos[1]
        + xTarget * math.cos(startHeading)
        - yTarget * math.sin(startHeading)

  local targetWorldY =
        startPos[2]
        + xTarget * math.sin(startHeading)
        + yTarget * math.cos(startHeading)

  local p = sim.getObjectPosition(base, -1)

  local ex = targetWorldX - p[1]
    local ey = targetWorldY - p[2]

  local distance = math.sqrt(ex * ex + ey * ey)

    --------------------------------------------------
    -- STOP AT THE TARGET
    --------------------------------------------------

  if distance < 0.10 then
        targetReached = true
        stopRobot()
        setSteering(0, 0)
        print('TARGET REACHED')
        return
    end

    --------------------------------------------------
    -- TARGET IN CURRENT ROBOT FRAME
    --------------------------------------------------

   local o = sim.getObjectOrientation(base, -1)
    local heading = o[3]

  local xt =
        ex * math.cos(heading)
        + ey * math.sin(heading)

   local yt =
        -ex * math.sin(heading)
        + ey * math.cos(heading)

    --------------------------------------------------
    -- STRAIGHT-LINE MODE
    --------------------------------------------------

  if math.abs(yTarget) < 0.001 then

  setSteering(0, 0)

  local dx = p[1] - startPos[1]
        local dy = p[2] - startPos[2]

   local forwardDistance =
            dx * math.cos(startHeading)
            + dy * math.sin(startHeading)

  if forwardDistance >= xTarget then
            targetReached = true
            stopRobot()
            setSteering(0, 0)
            print('TARGET REACHED')
            return
        end

 setSpeed(speed)
        return
    end

    --------------------------------------------------
    -- PURE-PURSUIT STEERING
    --------------------------------------------------

    -- Do not steer toward a target behind the rover.
    -- Stop rather than rotate indefinitely in place.
  if xt <= 0 then
        stopRobot()
        setSteering(0, 0)
        print('Target is behind the rover.')
        return
    end

  local theta = math.atan(yt, xt)

  local delta = math.atan(
        2 * L * math.sin(theta),
        distance
    )

    -- Clamp the equivalent steering angle.
  delta = math.max(
        -maxSteer,
        math.min(maxSteer, delta)
    )

    --------------------------------------------------
    -- ACKERMANN STEERING ANGLES
    --------------------------------------------------

   local leftAngle = 0
    local rightAngle = 0

  if math.abs(delta) > 0.001 then

  local R = L / math.tan(delta)

  leftAngle = math.atan(
            L / (R - W / 2)
        )

   rightAngle = math.atan(
            L / (R + W / 2)
        )

       -- Safety limit for each steering joint
  leftAngle = math.max(
            -maxSteer,
            math.min(maxSteer, leftAngle)
        )

  rightAngle = math.max(
            -maxSteer,
            math.min(maxSteer, rightAngle)
        )
 end

  setSteering(leftAngle, rightAngle)
    setSpeed(speed)

end
  

function setSteering(left, right)

  sim.setJointTargetPosition(
        flSteer, steerSign * left
    )

   sim.setJointTargetPosition(
        frSteer, steerSign * right
    )

end


--------------------------------------------------
-- WHEEL SPEED CONTROL
--------------------------------------------------

function setSpeed(v)

   local w = driveSign * v / wheelRadius

   sim.setJointTargetVelocity(flWheel, w)
    sim.setJointTargetVelocity(frWheel, w)
    sim.setJointTargetVelocity(rlWheel, w)
    sim.setJointTargetVelocity(rrWheel, w)

end


--------------------------------------------------
-- STOP ALL WHEELS
--------------------------------------------------

function stopRobot()

   sim.setJointTargetVelocity(flWheel, 0)
    sim.setJointTargetVelocity(frWheel, 0)
    sim.setJointTargetVelocity(rlWheel, 0)
    sim.setJointTargetVelocity(rrWheel, 0)

end
