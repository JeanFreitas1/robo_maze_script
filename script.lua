function sysCall_init()
    sensoresNome = {
        "Vision_sensor_S1",
        "Vision_sensor_S2",
        "Vision_sensor_S3",
        "Vision_sensor_SE",
        "Vision_sensor_SD",
        "Vision_sensor_RE",
        "Vision_sensor_RD",
        "Vision_sensor_RR"
    }
    -- Inicializando as variaveis
    field=sim.getObjectHandle("ResizableFloor_5_25") -- campo
    robot=sim.getObjectHandle("Pioneer_p3dx") -- Define o robo na variavel robot
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor") -- Motor esquerda
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")-- Motor direita
    sensores={-1,-1,-1,-1,-1,-1,-1,-1} -- cria um vetor para os sensores de visao
    
    for i = 1, 8, 1 do -- loop no array com os nomes dos sensores para associa-los ao array de sensores
        sensores[i]=sim.getObjectHandle(sensoresNome[i])
    end

    robot_pos0=sim.getObjectPosition(robot,field)
    
    clock=os.clock -- define a funcao de tempo
    t=clock() -- faz a leitura do tempo inicial
    t0=t -- tempo inicial recebe t
    sp=20.00 -- set point
    vp=20.0 -- valor inicial da vari?vel de processo
    e0=0.0 -- erro inicial
    ci=0.0 -- acumulado do integrativo
    v=3.0 -- velocidade base
    y=0
    op={0.0,0.0,0.0,0.0} -- zero o vetor
    buf_esc={1,1,1} -- {esquerda, frente, direita}
    
    index=1
    map={}
    voltando=false

end

function saveAction(action)
    map[index] = action
    index=index+1
    -- print('acao salva')
end

function invertMap()
    -- print(map)
    local i
    local len = table.getn(map) -- 13
    local invertedMap = {}

    for i=1,len do 
        invertedMap[i] = (map[(len+1)-i])*-1
    end
    map = invertedMap
    -- print(map)
    
end


function leitura()

    
    local num=0
    local den=0
    robot_pos_t=sim.getObjectPosition(robot,field)

    S={0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0} -- cria vetor de 8 posicoes -- o ultimo sensor come?a 1
    for i=1,7,1 do
        result,data=sim.readVisionSensor(sensores[i]) --data[11] e a intensidade do sensor
        if (result>=0) then
            if (data[11]<0.5) then -- compara a intensidade do sensor
                S[i]=1.0 -- sensor na linha recebe 1, e fora da linha recebe 0 na declara??o
            end
        end
    end

    -- sensor traseiro
    result8,data8=sim.readVisionSensor(sensores[8])
    if (result8 >=0) then
        if (data8[11]>0.8) then -- compara a intensidade do sensor
            S[8]=0.0 -- sensor na linha recebe 1, e fora da linha recebe 0 na declara??o
        end
    end

    --corrigi ao passar pela intersecao
    if ((S[4]==1) and (S[1]==1)) then S[1]=0 end
    if ((S[5]==1) and (S[3]==1)) then S[3]=0 end
    den=S[1]+S[2]+S[3]
    if den>0 then
        vp=(S[1]*10.0+S[2]*20.0+S[3]*30.0)/den -- calcula o valor da vari?vel de processo

    end
end


function seguir()
    S2=0
    S3=0
    leitura()


    G=0.1
    I=0.05
    e=sp-vp
    cp=G*e
    tf=clock()-t
    ci=(I*(e+e0)/2*(tf-t0))+ci
    c=cp+ci
    e0=e
    sim.setJointTargetVelocity(motorLeft,v-c) -- velocidade do motor esquerdo
    sim.setJointTargetVelocity(motorRight,v+c) -- velocidade do motor direito
    t0=tf
end



function esquerda()
    --print("Esquerda")
    local S0=S[4]
    leitura()
    if ((S[4]==1) and (S0==0)) then S2=1 end
    if ((S[2]==1) and(S2==1)) then
        y=0
        op={0.0,0.0,0.0,0.0} -- zero o vetor
    end
    sim.setJointTargetVelocity(motorLeft,-1) -- velocidade do motor esquerdo
    sim.setJointTargetVelocity(motorRight,1) -- velocidade do motor direito
end


function direita()
    --print("Direita")
    local S0=S[5]
    leitura()
    if ((S[5]==1) and (S0==0)) then S3=1 end
    if ((S[2]==1) and(S3==1)) then
        y=0
        op={0.0,0.0,0.0,0.0} -- zero o vetor
    end
    sim.setJointTargetVelocity(motorLeft,1) -- velocidade do motor esquerdo
    sim.setJointTargetVelocity(motorRight,-1) -- velocidade do motor direito
end



function parar()
    print("parar")
    sim.setJointTargetVelocity(motorLeft,0) -- velocidade do motor esquerdo
    sim.setJointTargetVelocity(motorRight,0) -- velocidade do motor direito
end


function escolher()



    local k = 1000000
    local point1 = op[1]*(1/buf_esc[1])
    local point2 = (op[1]*(1/buf_esc[1])) + (op[2]*(1/buf_esc[2]))
    local range = (op[1]*(1/buf_esc[1])) + (op[2]*(1/buf_esc[2])) + (op[3]*(1/buf_esc[3]))

    local r = math.random(0, range*k)
    r = r/k
    -- print(point1, point2, range)
    -- print(r)

    if ( (op[1]==1) and (r < point1) ) then
        print('escolhido esquerda')
        buf_esc[1]=buf_esc[1]+1
        saveAction(-1)
        y=3
    end

    if ( (op[2]==1) and (r >= point1) and (r < point2) ) then
        print('escolhido frente')
        buf_esc[2]=buf_esc[2]+1
        saveAction(0)
        y=0
    end

    if ( (op[3]==1) and (r >= point2) and (r < range) ) then
        print('escolhido direita')
        buf_esc[3]=buf_esc[3]+1
        saveAction(1)
        y=4
    end

end

function seguir_mapa()
local move = map[index]
    index=index+1

    if (move==0) then
        y=0
    end
    if (move==1) then
        y=4
    end
    if (move==-1) then
        y=3
    end

end


function sysCall_cleanup()

end


function sysCall_actuation()
    if (y==0) then
        -- print('Seguir')
        leitura()
        seguir()
    end

    --print(robot_pos_t)

    if (voltando==true) then
        local pos_dif_x = math.abs(robot_pos_t[1]-robot_pos0[1])
        local pos_dif_y = math.abs(robot_pos_t[2]-robot_pos0[2])

        if ((pos_dif_x < 0.2) and (pos_dif_y < 0.2)) then
            print('faz sentido')
            parar()
        end

    end


    --print(robot_pos_t)
    

    if ((S[8]==0) and (y==0)) then
        print('virar')
        y=3
    end


    if (((S[4]==1) or (S[5]==1)) and (y==0)) then
        --print('Intersecao')
        y=1
        op[1]= S[4]
        op[3]= S[5]
    end
    -- Posiciona o eixo do robo em cima da intersecao para facilitar o giro
    if (((S[6]==0) and (S[7]==0)) and (y==1)) then
        seguir()
        --print('Checando')
        if (S[4]==1) then op[1]=S[4] end
        if (S[5]==1) then op[3]=S[5] end
    end
    -- Para na intersec?o e Verifica se tem seguimento a frente ou se ? o target
    if (((S[6]==1) or (S[7]==1)) and (y==1)) then
        --print('Eixo intersecao')
        
        if (voltando==false) then

            y=2
            op[2]=S[2]

            -- Verifica target
            if ((S[2]==1) and (S[4]==1) and (S[5]==1)) then 
                --op[4]=1.0
                print('chegou no final')
                y=3
                index=1
                invertMap()
                voltando=true
                do return end
            end
            parar()

        else

            y=5

        end



    end
    -- Se tiver opcao esquerda escolhe
    if (y==2) then
        escolher()
    end

    if (y==3) then
        esquerda()
    end

    if (y==4) then
        direita()
    end

    if (y==5) then
        seguir_mapa()
    end



    robot_pos=sim.getObjectPosition(robot,-1)

    -- robot_pos=sim.getObjectPosition(robot,-1)
    -- print(robot_pos[1], robot_pos[2], robot_pos[3]) -- Mostra os valores de x y z do robo
end