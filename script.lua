function sysCall_init()
    robot =sim.getObjectHandle("Pioneer_p3dx") -- Define o robo na variavel robot
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor") -- Motor esquerda
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")-- Motor direita
    sensores={-1,-1,-1,-1,-1,-1,-1} -- cria um vetor para os sensores de visao
    sensores[1]=sim.getObjectHandle("Vision_sensor_S1") -- associa sensor 01
    sensores[2]=sim.getObjectHandle("Vision_sensor_S2") -- associa sensor 02
    sensores[3]=sim.getObjectHandle("Vision_sensor_S3") -- associa sensor 03
    sensores[4]=sim.getObjectHandle("Vision_sensor_SE") -- associa sensor 04
    sensores[5]=sim.getObjectHandle("Vision_sensor_SD") -- associa sensor 05
    sensores[6]=sim.getObjectHandle("Vision_sensor_RE") -- associa sensor 04
    sensores[7]=sim.getObjectHandle("Vision_sensor_RD") -- associa sensor 05
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
    
    buf_time=0
end

function saveAction(action)
    map[index] = action
    index=index+1
    print('acao salva')
end

function invertMap()
    print(map)
    local i
    local len = table.getn(map) -- 13
    local invertedMap = {}

    for i=1,len do 
        invertedMap[i] = (map[(len+1)-i])*-1
    end
    map = invertedMap
    print(map)
    
end


function leitura()
    local num=0
    local den=0
    S={0.0,0.0,0.0,0.0,0.0,0.0,0.0} -- cria vetor de 7 posicoes
    for i=1,7,1 do
        result,data=sim.readVisionSensor(sensores[i]) --data[11] e a intensidade do sensor
        if (result>=0) then
            if (data[11]<0.5) then -- compara a intensidade do sensor
                S[i]=1.0 -- sensor na linha recebe 1, e fora da linha recebe 0 na declara??o
            end
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
    clock=os.clock
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
    --print("parar")
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
    print(point1, point2, range)
    print(r)

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


function sysCall_cleanup()

end


function sysCall_actuation()

    print(map)

    if (y==0) then
        -- print('Seguir')
        leitura()
        seguir()
    end

    if ((S[1]==0) and (S[2]==0) and (S[3]==0) and (S[4]==0) and (S[5]==0) and (y==0)) then
        saveAction(-2)
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
        y=2
        op[2]=S[2]
        -- Verifica target
        if ((S[2]==1) and (S[4]==1) and (S[5]==1)) then 
            op[4]=1.0 
            y=0
            print('chegou no final')
        end
        parar()

    end
    -- Se tiver opcao esquerda escolhe
    if (y==2) then
        
        escolher()

    end

    if (y==3) then
        --print('executando esquerda')
        esquerda()
        
    end

    if (y==4) then
        --print('executando direita')
        direita()
    end


    robot_pos=sim.getObjectPosition(robot,-1)

    -- robot_pos=sim.getObjectPosition(robot,-1)
    -- print(robot_pos[1], robot_pos[2], robot_pos[3]) -- Mostra os valores de x y z do robo
end