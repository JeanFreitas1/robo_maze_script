-- ROBOT MODES

--   MODE          ID

--  idle            0
--  foward          9
--  backward        1
--  turning right   2
--  turning left    6
--  stop            5
--  turn back       8
--
--
--
--
--
---------------------------------------------------------------

function sysCall_init()

    clock=os.clock -- define a funcao de tempo
    t=clock() -- faz a leitura do tempo inicial



    robot =sim.getObjectHandle("Pioneer_p3dx") -- Define o robo na variavel robot
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor") -- Motor esquerda
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")-- Motor direita

    timeinteger=0
    velocity=1
    vLeft=velocity  
    vRight=velocity 
    robot_mode=0
    last_move=5
    last_move2=5
    lastError=0
    last_turn=2
    i=0
    n=0
    robot_position0=sim.getObjectPosition
    robot_positionx=robot_position0[0]
    robot_positiony=robot_position0[1]
    moves={}
    path={}

    printf("Time Counter (seconds):")

end
------------------------------------------------------xxxxxxxxxxx
function sysCall_cleanup() 

-- TIME ELAPSED
    printf("Total Time Elapsed:")
    printf(t1-t)
 
end 
------------------------------------------------------xxxxxxxxxxx
function sysCall_actuation()
    
-- TIME COUNTER DISPLAY
    clock=os.clock
    t1=clock()
    telapse=math.floor(t1-t)
    if telapse==timeinteger then
    printf(telapse) printf(" seconds")
    timeinteger=timeinteger+1
    end
---------------------------------------------------------------
-- ROBOT MODES

--   MODE          ID

--  idle            0
--  foward          9
--  backward        1
--  turning right   2
--  turning left    6
--  stop            5
--  turn back       8
--  choosing path   10
---------------------------------------------------------------

--READING BASIC NAVIGATION SENSORS - DATA VECTOR STORAGE

    data_l1=0
    data_l2=0
    data_l3=0
    data_c1=0
    data_c2=0

    data_r1=0
    data_r2=0
    data_r3=0

    resultl1,data_l1=sim.readVisionSensor(sim.getObjectHandle("vs_left1"))
    resultr1,data_r1=sim.readVisionSensor(sim.getObjectHandle("vs_right1"))
    resultl3,data_l3=sim.readVisionSensor(sim.getObjectHandle("vs_left3"))
    resultr3,data_r3=sim.readVisionSensor(sim.getObjectHandle("vs_right3"))

    
    resultl2,data_l2=sim.readVisionSensor(sim.getObjectHandle("vs_left2"))
    resultr2,data_r2=sim.readVisionSensor(sim.getObjectHandle("vs_right2"))
    resultc1,data_c1=sim.readVisionSensor(sim.getObjectHandle("vs_center1"))
    resultc2,data_c2=sim.readVisionSensor(sim.getObjectHandle("vs_center2"))




    --  PID Equation

    KP= 0.7
    KD= 0.2
    KI= 0.05
    error= (0 -(data_r2[11]-data_l2[11]))
    PID_eq = ((KP * error) + (KD*(error -lastError)) + (KI)) 

    lastError=error

-- MODES MOTOR CONTROL 

        --IDLE MODE - STRAIGHT ONLY
    if robot_mode==0 then
          sim.setJointTargetVelocity(motorLeft,vLeft)
          sim.setJointTargetVelocity(motorRight,vRight)
            --SET TO FOWARD IF ANY SENSOR SEES BLACK
            if (data_r1[11]<0.2) or (data_l1[11]<0.2) or (data_r2[11]<0.2) or (data_l2[11]<0.2) then
            robot_mode=9
            end
    end
 ---------------------------------------------------------------
--FOWARD MODE

    if robot_mode==9 then

            --PID Correction
              sim.setJointTargetVelocity(motorLeft,(vLeft + PID_eq))
              sim.setJointTargetVelocity(motorRight,(vRight - PID_eq))

         --DUAL PATH    
            if ((data_r3[11]<0.4) and (data_l3[11]<0.4)) then 
                robot_mode=10
            end

        --  CORNER CHECK
                if (data_l1[11]<0.2) then
                    robot_mode=6
                    last_move=9     
                end
                if (data_r1[11]<0.2) then
                    robot_mode=2
                    last_move=9
                end

        --TURN BACK IF PATH IS OVER
                if (data_r1[11]>0.6) and (data_l1[11]>0.6) and (data_c1[11]>0.6) and (data_l2[11]>0.6) and (data_r2[11]>0.6) and (data_l3[11]>0.6) and (data_r3[11]>0.6) and (data_c2[11]>0.6)  then
                    robot_mode=8
                end

    end
---------------------------------------------------------------
--BACKWARD MODE
    if robot_mode==1 then
        sim.setJointTargetVelocity(motorLeft,-vLeft)
        sim.setJointTargetVelocity(motorRight,-vRight)
    end
---------------------------------------------------------------
-- TURN LEFT MODE
    if robot_mode==6 then 
        sim.setJointTargetVelocity(motorLeft, -0.20)
        sim.setJointTargetVelocity(motorRight,vRight)

        if (data_c1[11]<0.2) then
                robot_mode=9
                last_move=6
        end

    end
---------------------------------------------------------------
--  TURN RIGHT MODE
    if robot_mode==2 then
        sim.setJointTargetVelocity(motorLeft,vLeft)
        sim.setJointTargetVelocity(motorRight,-0.20)
        if (data_c1[11]<0.2) then
                robot_mode=9
                last_move=2
        end

    end
---------------------------------------------------------------
--  STOP MODE
    if robot_mode==5 then
        sim.setJointTargetVelocity(motorLeft,0)
        sim.setJointTargetVelocity(motorRight,0)
    end
---------------------------------------------------------------
-- TURN BACK MODE - ID 8
    if robot_mode==8 then
        sim.setJointTargetVelocity(motorLeft,-vLeft/2)
        sim.setJointTargetVelocity(motorRight,vRight/2)
            if (data_c1[11]<0.2) then
                robot_mode=9
                last_move=8
            end
    end
---------------------------------------------------------------
-- CHOOSING PATH MODE - ID 10

    if robot_mode==10 then
       sim.setJointTargetVelocity(motorLeft,0)
       sim.setJointTargetVelocity(motorRight,0)
         x=math.random(0,1)
       if x==0 then
       robot_mode=2
        print("DIREITA.")
        print(x)
       else robot_mode=6
        print("ESQUERDA.")
        print(x)
       end

    end
---------------------------------------------------xxxxxxxxxxxx
---------------------------------------------------------------
-- ROBOT MODES

--   MODE          ID

--  idle            0
--  foward          9
--  backward        1
--  turning right   2
--  turning left    6
--  stop            5
--  turn back       8
--  choosing path   10
---------------------------------------------------------------
end