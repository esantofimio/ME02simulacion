# Simulación Modelos Estocásticos

<ul>
<li>Elmar Santofimio Suarez</li>
<li>Carlos Augusto Gutierrez Silva</li>
<li>German David Guerrero</li>
</ul>


<h3>
  Codigo fuente de la red Ad-Hoc
</h3>
    fichero: ./adhoc_wifi_Opengym/linear_mesh_2/sim.cc
    
    #include "ns3/core-module.h"
    #include "ns3/applications-module.h"
    #include "ns3/opengym-module.h"
    #include "ns3/mobility-module.h"
    #include "ns3/wifi-module.h"
    #include "ns3/internet-module.h"
    #include "ns3/spectrum-module.h"
    #include "ns3/stats-module.h"
    #include "ns3/flow-monitor-module.h"
    #include "ns3/traffic-control-module.h"
    #include "ns3/node-list.h"

    #include "mygym.h"

    using namespace ns3;

    NS_LOG_COMPONENT_DEFINE ("OpenGym");

    int
    main (int argc, char *argv[])
    {
      // Parameters of the environment
      uint32_t simSeed = 1;
      double simulationTime = 10; //seconds
      double envStepTime = 0.1; //seconds, ns3gym env step time interval
      uint32_t openGymPort = 5555;
      uint32_t testArg = 0;

      bool eventBasedEnv = true;

      //Parameters of the scenario
      uint32_t nodeNum = 20;
      double distance = 10.0;
      bool noErrors = false;
      std::string errorModelType = "ns3::NistErrorRateModel";
      bool enableFading = true;
      uint32_t pktPerSec = 1000;
      uint32_t payloadSize = 1500;
      bool enabledMinstrel = false;

      // define datarates
      std::vector<std::string> dataRates;
      dataRates.push_back("OfdmRate1_5MbpsBW5MHz");
      dataRates.push_back("OfdmRate2_25MbpsBW5MHz");
      dataRates.push_back("OfdmRate3MbpsBW5MHz");
      dataRates.push_back("OfdmRate4_5MbpsBW5MHz");
      dataRates.push_back("OfdmRate6MbpsBW5MHz");
      dataRates.push_back("OfdmRate9MbpsBW5MHz");
      dataRates.push_back("OfdmRate12MbpsBW5MHz");
      dataRates.push_back("OfdmRate13_5MbpsBW5MHz");
      uint32_t dataRateId = 1;


      CommandLine cmd;
      // required parameters for OpenGym interface
      cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
      cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", simSeed);
      // optional parameters
      cmd.AddValue ("eventBasedEnv", "Whether steps should be event or time based. Default: true", eventBasedEnv);
      cmd.AddValue ("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
      cmd.AddValue ("nodeNum", "Number of nodes. Default: 5", nodeNum);
      cmd.AddValue ("distance", "Inter node distance. Default: 10m", distance);
      cmd.AddValue ("testArg", "Extra simulation argument. Default: 0", testArg);
      cmd.Parse (argc, argv);

      NS_LOG_UNCOND("Ns3Env parameters:");
      NS_LOG_UNCOND("--simulationTime: " << simulationTime);
      NS_LOG_UNCOND("--openGymPort: " << openGymPort);
      NS_LOG_UNCOND("--envStepTime: " << envStepTime);
      NS_LOG_UNCOND("--seed: " << simSeed);
      NS_LOG_UNCOND("--distance: " << distance);
      NS_LOG_UNCOND("--testArg: " << testArg);

      if (noErrors){
        errorModelType = "ns3::NoErrorRateModel";
      }

      RngSeedManager::SetSeed (1);
      RngSeedManager::SetRun (simSeed);

      // Configuration of the scenario
      // Create Nodes
      NodeContainer nodes;
      nodes.Create (nodeNum);

      // WiFi device
      WifiHelper wifi;
      wifi.SetStandard (WIFI_PHY_STANDARD_80211_5MHZ);

      // Channel
      SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();
      Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();

      spectrumPhy.SetChannel (spectrumChannel);
      spectrumPhy.SetErrorRateModel (errorModelType);
      spectrumPhy.Set ("Frequency", UintegerValue (5200));
      spectrumPhy.Set ("ChannelWidth", UintegerValue (5));

      Config::SetDefault ("ns3::WifiPhy::Frequency", UintegerValue (5200));
      Config::SetDefault ("ns3::WifiPhy::ChannelWidth", UintegerValue (5));

      // Channel
      Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel> ();
      Ptr<NakagamiPropagationLossModel> fadingModel = CreateObject<NakagamiPropagationLossModel> ();
      if (enableFading) {
        lossModel->SetNext (fadingModel);
      }
      spectrumChannel->AddPropagationLossModel (lossModel);
      Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
      spectrumChannel->SetPropagationDelayModel (delayModel);

      // Add MAC and set DataRate
      WifiMacHelper wifiMac;

      if (enabledMinstrel) {
        wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager");
      } else {
        std::string dataRateStr = dataRates.at(dataRateId);
        NS_LOG_UNCOND("dataRateStr: " << dataRateStr);
        wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (dataRateStr),
                                      "ControlMode", StringValue (dataRateStr));
      }

      // Set it to adhoc mode
      wifiMac.SetType ("ns3::AdhocWifiMac",
                       "QosSupported", BooleanValue (false));

      // Install wifi device
      NetDeviceContainer devices = wifi.Install (spectrumPhy, wifiMac, nodes);

      // Mobility model
      MobilityHelper mobility;
      mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                     "MinX", DoubleValue (0.0),
                                     "MinY", DoubleValue (0.0),
                                     "DeltaX", DoubleValue (distance),
                                     "DeltaY", DoubleValue (distance),
                                     "GridWidth", UintegerValue (5),  // will create linear topology
                                     "LayoutType", StringValue ("RowFirst"));
      mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue (Rectangle (-500, 500, -500, 500)),
                                 "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=5.0] "),
                                 "Direction", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"));

      mobility.Install (nodes);

      // IP stack and routing
      InternetStackHelper internet;
      internet.Install (nodes);

      // Assign IP addresses to devices
      Ipv4AddressHelper ipv4;
      NS_LOG_INFO ("Assign IP Addresses");
      ipv4.SetBase ("10.1.1.0", "255.255.255.0");
      Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

      //Configure static multihop routing
      for (uint32_t i = 0; i < nodes.GetN()-1; i++){
        Ptr<Node> src = nodes.Get(i);
        Ptr<Node> nextHop = nodes.Get(i+1);
        Ptr<Ipv4> destIpv4 = nextHop->GetObject<Ipv4> ();
        Ipv4InterfaceAddress dest_ipv4_int_addr = destIpv4->GetAddress (1, 0);
        Ipv4Address dest_ip_addr = dest_ipv4_int_addr.GetLocal ();

        Ptr<Ipv4StaticRouting>  staticRouting = Ipv4RoutingHelper::GetRouting <Ipv4StaticRouting> (src->GetObject<Ipv4> ()->GetRoutingProtocol ());
        staticRouting->RemoveRoute(1);
        staticRouting->SetDefaultRoute(dest_ip_addr, 1, 0);
      }

      // Traffic
      // Create a BulkSendApplication and install it on node 0
      Ptr<UniformRandomVariable> startTimeRng = CreateObject<UniformRandomVariable> ();
      startTimeRng->SetAttribute ("Min", DoubleValue (0.0));
      startTimeRng->SetAttribute ("Max", DoubleValue (1.0));

      uint16_t port = 1000;
      uint32_t srcNodeId = 0;
      uint32_t destNodeId = nodes.GetN() - 1;
      Ptr<Node> srcNode = nodes.Get(srcNodeId);
      Ptr<Node> dstNode = nodes.Get(destNodeId);

      Ptr<Ipv4> destIpv4 = dstNode->GetObject<Ipv4> ();
      Ipv4InterfaceAddress dest_ipv4_int_addr = destIpv4->GetAddress (1, 0);
      Ipv4Address dest_ip_addr = dest_ipv4_int_addr.GetLocal ();

      InetSocketAddress destAddress (dest_ip_addr, port);
      destAddress.SetTos (0x70); //AC_BE
      UdpClientHelper source (destAddress);
      source.SetAttribute ("MaxPackets", UintegerValue (pktPerSec * simulationTime));
      source.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      Time interPacketInterval = Seconds (1.0/pktPerSec);
      source.SetAttribute ("Interval", TimeValue (interPacketInterval)); //packets/s

      ApplicationContainer sourceApps = source.Install (srcNode);
      sourceApps.Start (Seconds (0.0));
      sourceApps.Stop (Seconds (simulationTime));

      // Create a packet sink to receive these packets
      UdpServerHelper sink (port);
      ApplicationContainer sinkApps = sink.Install (dstNode);
      sinkApps.Start (Seconds (0.0));
      sinkApps.Stop (Seconds (simulationTime));

      // Print node positions
      NS_LOG_UNCOND ("Node Positions:");
      for (uint32_t i = 0; i < nodes.GetN(); i++)
      {
        Ptr<Node> node = nodes.Get(i);
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
        NS_LOG_UNCOND ("---Node ID: " << node->GetId() << " Positions: " << mobility->GetPosition());
      }

      // OpenGym Env
      Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface> (openGymPort);
      Ptr<MyGymEnv> myGymEnv;
      if (eventBasedEnv)
      {
        myGymEnv = CreateObject<MyGymEnv> ();
      } else {
        myGymEnv = CreateObject<MyGymEnv> (Seconds(envStepTime));
      }
      myGymEnv->SetOpenGymInterface(openGymInterface);

      // connect OpenGym entity to event source
      Ptr<UdpServer> udpServer = DynamicCast<UdpServer>(sinkApps.Get(0));
      if (eventBasedEnv)
      {
        udpServer->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&MyGymEnv::NotifyPktRxEvent, myGymEnv, dstNode));
      } else {
        udpServer->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&MyGymEnv::CountRxPkts, myGymEnv, dstNode));
      }
      myGymEnv->setUdpServer(udpServer);

      NS_LOG_UNCOND ("Simulation start");
      Simulator::Stop (Seconds (simulationTime));
      Simulator::Run ();
      NS_LOG_UNCOND ("Simulation stop");

      myGymEnv->NotifySimulationEnd();
      Simulator::Destroy ();

    }

<h3>
  Descripción del codigo:
</h3>

Para el modelo de movilidad de los nodos se escogió la clase ns3::RandomWalk2dMobilityModel, ya que nuestra red se planteo en 2 dimensiones, este modelo permite asignar valores aleatorios tanto a la dirección de los nodos como a su velocidad, además varia estos valores cada vez que los nodos recorran una cierta distancia especifica (3mts). Esto permite que cada vez que se realice la simulación el comportamiento de los nodos sea diferente, además el valor mínimo de la velocidad de los nodos se dejo en 0m/s, esto con el fin de simular también los nodos que eventualmente se quedan quietos.


<h3>
  Codigo fuente del agente dqn-agent-v2.py
</h3>
    fichero: ./adhoc_wifi_Opengym/linear_mesh_2/dqn-agent-v2.py 
    
    import scipy.io as io
    import gym
    import tensorflow as tf
    import tensorflow.contrib.slim as slim
    import numpy as np
    from tensorflow import keras
    from ns3gym import ns3env


    class DqnAgent(object):
        """docstring for DqnAgent"""
        def __init__(self, inNum, outNum):
            super(DqnAgent, self).__init__()
            self.model = keras.Sequential()
            self.model.add(keras.layers.Dense(inNum, input_shape=(inNum,), activation='relu'))
            self.model.add(keras.layers.Dense(outNum, activation='softmax'))
            self.model.compile(optimizer=tf.train.AdamOptimizer(0.001),
                            loss='categorical_crossentropy',
                            metrics=['accuracy'])

        def get_action(self, state):
            return np.argmax(self.model.predict(state)[0])

        def predict(self, next_state):
            return self.model.predict(next_state)[0]

        def fit(self, state, target, action):
            target_f = self.model.predict(state)
            target_f[0][action] = target 
            self.model.fit(state, target_f, epochs=1, verbose=0)


    # Environment initialization
    port = 5555
    simTime = 10 # seconds
    startSim = True
    stepTime = 0.05 # seconds
    seed = 132
    simArgs = {"--simTime": simTime,
            "--testArg": 123,
            "--nodeNum": 5,
            "--distance": 500}
    debug = False

    env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)
    #env = gym.make('ns3-v0')


    ob_space = env.observation_space
    ac_space = env.action_space
    print("Observation space: ", ob_space)
    print("Action space: ", ac_space, ac_space.dtype)
    s_size = ob_space[0].shape[0]
    a_size = ac_space.shape[0]

    inputQueues = 1
    cwSize = 100

    agents = []
    for i in range (a_size - 1):
        agents.append(DqnAgent(inputQueues, cwSize))

    total_episodes = 10
    max_env_steps = 100
    env._max_episode_steps = max_env_steps

    epsilon = 1.0               # exploration rate
    epsilon_min = 0.01
    epsilon_decay = 0.999

    time_history = []
    rew_history = []

    stepIdx = 0

    state = env.reset()[0]
    state = np.reshape(state, [1, s_size])
    rewardsum = 0
    for time in range(max_env_steps):
        stepIdx += 1
        
        # Choose action
        actions = []
        if np.random.rand(1) < epsilon:
            for agent in agents:
                actions.append(np.random.randint(cwSize))
        else:
            for agent in agents:
                actions.append(agent.get_action(state[:,0]-state[:,1]))

        actions.append(100)
        print("---action: ", actions)
        # Step
        next_state, reward, done, info = env.step(actions)

        print("Step: ", stepIdx)
        print("---next state, reward, done, info: ", next_state, reward, done, info)
        next_state = next_state[0]
        if done:
            print("Done")
            break

        next_state = np.reshape(next_state, [1, s_size])
        
        targets = []

        # Train
        for agent in agents:
            targets.append(reward)

        if not done:
            for i in range(len(agents)):
                targets[i] = reward + 0.95 * np.amax(agents[i].predict(next_state[:,0]-next_state[:,1]))

        for i in range(len(agents)):
            agents[i].fit(state[:,i]-state[:, i + 1], targets[i], actions[i])

        state = next_state
        rewardsum += reward
        if epsilon > epsilon_min: epsilon *= epsilon_decay

    print('Total Reward:', rewardsum)

<h3>
  Descripción del codigo:
</h3>
La técnica de aprendizaje por refuerzo esta dado por, un agente que toma deciciones en un ambiente dado de forma discreta, de tal manera que induce a maximizar una nocíón de recompensa asociado a los estimulos y las decisiones tomandas, en el agente que se encuentra en la parte superior se implemento el algoritmo DQN el cual tiene las siguientes peculiaridades


Se crea una red neuronal principal donde todos los nodos de la capa esten fuertemente conectados, para cada capa se utiliza una funcion de activacion diferente en la primera la función relu y en la segunda la funcion softmax, estas dos creadas con la ayuda del framework keras.
Para la función del gradiente descendiente se implementa el algorimo ADAM el cual usa a su vez la idea del algoritmo (MGD) para gestionar el aprendizaje cuando se hallan minimos locales y no los minimos globales 

Se incluyen las funciones de fit, predict y get action que corresponden al entrenamiento de los datos de entrenamiento, a la asignacion de labels de acuerdo al estado recibido y las acciones que se deben realizar con lo que aprende el agente

Se definen los hiperparametros como el epsilon, epsilon min y epsilon decay. El epsilon decay me dara los pasos que debe dar el gradiente para llegar a minimo global, el epsilon es un parametro de exploracion donde tendrá entre sus labores revisar el minimo error que puede cometer 


<h3>
  Codigo fuente del agente
</h3>
    fichero: ./adhoc_wifi_Opengym/linear_mesh_2/my_random2.py 
    
    import argparse
    from ns3gym import ns3env

    __author__ = "Piotr Gawlowicz"
    __copyright__ = "Copyright (c) 2018, Technische Universität Berlin"
    __version__ = "0.1.0"
    __email__ = "gawlowicz@tkn.tu-berlin.de"


    parser = argparse.ArgumentParser(description='Start simulation script on/off')
    parser.add_argument('--start',
                        type=int,
                        default=0,
                        help='Start simulation script 0/1, Default: 0')
    parser.add_argument('--iterations',
                        type=int,
                        default=1,
                        help='Number of iterations, Default: 1')
    args = parser.parse_args()
    startSim = bool(args.start)
    iterationNum = int(args.iterations)

    port = 5555
    simTime = 10 # seconds
    stepTime = 0.01  # seconds
    seed = 0
    simArgs = {"--simTime": simTime,
            "--testArg": 123,
            "--nodeNum": 5,
            "--distance": 500}
    debug = False

    env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)
    env.reset()

    ob_space = env.observation_space
    ac_space = env.action_space
    print("Observation space: ", ob_space,  ob_space.dtype)
    print("Action space: ", ac_space, ac_space.dtype)

    stepIdx = 0
    currIt = 0

    rewardsum = 0
    reward = 0
    obs = env.reset()

    while True:
        stepIdx += 1

        action = env.action_space.sample()
        action = action * 1 + 1
        print("---action: ", action)

        obs, reward, done, info = env.step(action)
        print("Step: ", stepIdx)
        print("---obs, reward, done, info: ", obs, reward, done, info)
        rewardsum += reward

        if done:
            print('Done')
            break

    print('Total Reward:', rewardsum)
        
<h3>
  Descripción del codigo:
</h3>

EL agente my_random2 selecciona aleatoriamente una posibilidad del espcio de acciones. Al final el agente termina su ejecución imprimiento el total de recompensa obtenida.

<h3>
  Codigo fuente del ambiente
</h3>
    fichero: ./adhoc_wifi_Opengym/linear_mesh_2/mygym.cc
    
    #include "mygym.h"
    #include "ns3/object.h"
    #include "ns3/core-module.h"
    #include "ns3/wifi-module.h"
    #include "ns3/node-list.h"
    #include "ns3/log.h"
    #include <sstream>
    #include <iostream>

    namespace ns3 {

    NS_LOG_COMPONENT_DEFINE ("MyGymEnv");

    NS_OBJECT_ENSURE_REGISTERED (MyGymEnv);

    MyGymEnv::MyGymEnv ()
    {
      NS_LOG_FUNCTION (this);
      m_currentNode = 0;
      m_rxPktNum = 0;
    }

    MyGymEnv::MyGymEnv (Time stepTime)
    {
      NS_LOG_FUNCTION (this);
      m_currentNode = 0;
      m_rxPktNum = 0;
      m_interval = stepTime;

      Simulator::Schedule (Seconds(0.0), &MyGymEnv::ScheduleNextStateRead, this);
    }

    void
    MyGymEnv::ScheduleNextStateRead ()
    {
      NS_LOG_FUNCTION (this);
      Simulator::Schedule (m_interval, &MyGymEnv::ScheduleNextStateRead, this);
      Notify();
    }

    MyGymEnv::~MyGymEnv ()
    {
      NS_LOG_FUNCTION (this);
    }

    TypeId
    MyGymEnv::GetTypeId (void)
    {
      static TypeId tid = TypeId ("MyGymEnv")
        .SetParent<OpenGymEnv> ()
        .SetGroupName ("OpenGym")
        .AddConstructor<MyGymEnv> ()
      ;
      return tid;
    }

    void
    MyGymEnv::DoDispose ()
    {
      NS_LOG_FUNCTION (this);
    }


    /**
     * Función para obtener el ambiente de acción. Para ese modelo, las acciones representan 
     * un valor fijo para congestion window (cw) de cada nodo. El espacio entonces sera la 
     * nueva cw de cada nodo. Por ende sera un Box de tamaño nodenum entre 0 (el mínimo valor
     * de la cw) y 100 (el máximo valor de la cw)
     * 
     */ 
    Ptr<OpenGymSpace>
    MyGymEnv::GetActionSpace()
    {
      NS_LOG_FUNCTION (this);
      uint32_t nodeNum = NodeList::GetNNodes ();
      float low = 0.0;
      float high = 100.0;
      std::vector<uint32_t> shape = {nodeNum,};
      std::string dtype = TypeNameGet<uint32_t> ();
      Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
      NS_LOG_UNCOND ("GetActionSpace: " << space);
      return space;
    }


    /**
     * 
     * El espacio de observación esta definido por una pareja que contiene el número de paquetes
     * encolados de cada nodo, y el total de paquetes perdidos tras la última acción. Por ende
     * este espacio será una tupla que contiene una box, y un elemento discreto.
     * 
     */ 
    Ptr<OpenGymSpace>
    MyGymEnv::GetObservationSpace()
    {
      NS_LOG_FUNCTION (this);
      uint32_t nodeNum = NodeList::GetNNodes ();
      float low = 0.0;
      float high = 100.0;
      std::vector<uint32_t> shape = {nodeNum,};
      std::string dtype = TypeNameGet<uint32_t> ();
      Ptr<OpenGymDiscreteSpace> discrete = CreateObject<OpenGymDiscreteSpace> (nodeNum);
      Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);

      Ptr<OpenGymTupleSpace> space = CreateObject<OpenGymTupleSpace> ();
      space->Add(box);
      space->Add(discrete);
      NS_LOG_UNCOND ("GetObservationSpace: " << space);
      return space;
    }

    bool
    MyGymEnv::GetGameOver()
    {
      NS_LOG_FUNCTION (this);
      bool isGameOver = false;
      NS_LOG_UNCOND ("MyGetGameOver: " << isGameOver);
      return isGameOver;
    }

    Ptr<WifiMacQueue>
    MyGymEnv::GetQueue(Ptr<Node> node)
    {
      Ptr<NetDevice> dev = node->GetDevice (0);
      Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
      Ptr<WifiMac> wifi_mac = wifi_dev->GetMac ();
      Ptr<RegularWifiMac> rmac = DynamicCast<RegularWifiMac> (wifi_mac);
      PointerValue ptr;
      rmac->GetAttribute ("Txop", ptr);
      Ptr<Txop> txop = ptr.Get<Txop> ();
      Ptr<WifiMacQueue> queue = txop->GetWifiMacQueue ();
      return queue;
    }

    Ptr<OpenGymDataContainer>
    MyGymEnv::GetObservation()
    {

      NS_LOG_FUNCTION (this);
      uint32_t nodeNum = NodeList::GetNNodes ();
      std::vector<uint32_t> shape = {nodeNum,};

      // Instanciación del contenedor de los paquetes perdidos tras la última acción
      static uint32_t lastLosses = 0;
      Ptr<OpenGymDiscreteContainer> discrete = CreateObject<OpenGymDiscreteContainer>(nodeNum);
      uint32_t losses = udpServer->GetLost ();
      discrete->SetValue(losses - lastLosses);
      lastLosses = losses;
      // Busqueda de todos los paquetes encolados de cada nodo.
      Ptr<OpenGymBoxContainer<uint32_t> > box = CreateObject<OpenGymBoxContainer<uint32_t> >(shape);

      for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) {
        Ptr<Node> node = *i;
        Ptr<WifiMacQueue> queue = GetQueue (node);
        uint32_t value = queue->GetNPackets();
        box->AddValue(value);
      }

      Ptr<OpenGymTupleContainer> data = CreateObject<OpenGymTupleContainer> ();
      data->Add(box);
      data->Add(discrete);

      NS_LOG_UNCOND ("MyGetObservation: " << data);
      return data;
    }

    /**
     * La recompensa de cada nodo será la cantidad de paquetes recibidos totales tras la
     * última acción
     * 
     */
    float
    MyGymEnv::GetReward()
    {
      NS_LOG_FUNCTION (this);
      static float lastValue = 0.0;
      float reward = m_rxPktNum - lastValue;
      lastValue = m_rxPktNum;
      NS_LOG_UNCOND ("MyGetReward: " << reward);
      return reward;
    }

    std::string
    MyGymEnv::GetExtraInfo()
    {
      NS_LOG_FUNCTION (this);
      std::string myInfo = "currentNodeId";
      myInfo += "=";
      if (m_currentNode) {
        myInfo += std::to_string(m_currentNode->GetId());
      }
      NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
      return myInfo;
    }

    bool
    MyGymEnv::SetCw(Ptr<Node> node, uint32_t cwMinValue, uint32_t cwMaxValue)
    {
      Ptr<NetDevice> dev = node->GetDevice (0);
      Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
      Ptr<WifiMac> wifi_mac = wifi_dev->GetMac ();
      Ptr<RegularWifiMac> rmac = DynamicCast<RegularWifiMac> (wifi_mac);
      PointerValue ptr;
      rmac->GetAttribute ("Txop", ptr);
      Ptr<Txop> txop = ptr.Get<Txop> ();

      // if both set to the same value then we have uniform backoff?
      if (cwMinValue != 0) {
        NS_LOG_DEBUG ("Set CW min: " << cwMinValue);
        txop->SetMinCw(cwMinValue);
      }

      if (cwMaxValue != 0) {
        NS_LOG_DEBUG ("Set CW max: " << cwMaxValue);
        txop->SetMaxCw(cwMaxValue);
      }
      return true;
    }

    bool
    MyGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
    {
      NS_LOG_FUNCTION (this);
      NS_LOG_UNCOND ("MyExecuteActions: " << action);
      Ptr<OpenGymBoxContainer<uint32_t> > box = DynamicCast<OpenGymBoxContainer<uint32_t> >(action);
      std::vector<uint32_t> actionVector = box->GetData();

      uint32_t nodeNum = NodeList::GetNNodes ();
      for (uint32_t i=0; i<nodeNum; i++)
      {
        Ptr<Node> node = NodeList::GetNode(i);
        uint32_t cwSize = actionVector.at(i);
        SetCw(node, cwSize, cwSize);
      }

      return true;
    }

    void
    MyGymEnv::NotifyPktRxEvent(Ptr<MyGymEnv> entity, Ptr<Node> node, Ptr<const Packet> packet)
    {
      NS_LOG_DEBUG ("Client received a packet of " << packet->GetSize () << " bytes");
      entity->m_currentNode = node;
      entity->m_rxPktNum++;

      NS_LOG_UNCOND ("Node with ID " << entity->m_currentNode->GetId() << " received " << entity->m_rxPktNum << " packets");

      entity->Notify();
    }

    void
    MyGymEnv::CountRxPkts(Ptr<MyGymEnv> entity, Ptr<Node> node, Ptr<const Packet> packet)
    {
      NS_LOG_DEBUG ("Client received a packet of " << packet->GetSize () << " bytes");
      entity->m_currentNode = node;
      entity->m_rxPktNum++;
    }

    void 
    MyGymEnv::setUdpServer(Ptr<UdpServer> theUdpServer){
      udpServer = theUdpServer;
    }


    } // ns3 namespace
    
<h3>
  Codigo fuente de la libreria adicional
</h3>
    fichero: ./adhoc_wifi_Opengym/linear_mesh_2/mygym.h
    
    #ifndef MY_GYM_ENTITY_H
    #define MY_GYM_ENTITY_H

    #include "ns3/stats-module.h"
    #include "ns3/opengym-module.h"
    #include "ns3/udp-server.h"

    namespace ns3 {

    class Node;
    class WifiMacQueue;
    class Packet;

    class MyGymEnv : public OpenGymEnv
    {
    public:
      MyGymEnv ();
      MyGymEnv (Time stepTime);
      virtual ~MyGymEnv ();
      static TypeId GetTypeId (void);
      virtual void DoDispose ();

      Ptr<OpenGymSpace> GetActionSpace();
      Ptr<OpenGymSpace> GetObservationSpace();
      bool GetGameOver();
      Ptr<OpenGymDataContainer> GetObservation();
      float GetReward();
      std::string GetExtraInfo();
      bool ExecuteActions(Ptr<OpenGymDataContainer> action);
      void setUdpServer(Ptr<UdpServer> udpServer);
      // the function has to be static to work with MakeBoundCallback
      // that is why we pass pointer to MyGymEnv instance to be able to store the context (node, etc)
      static void NotifyPktRxEvent(Ptr<MyGymEnv> entity, Ptr<Node> node, Ptr<const Packet> packet);
      static void CountRxPkts(Ptr<MyGymEnv> entity, Ptr<Node> node, Ptr<const Packet> packet);

    private:
      void ScheduleNextStateRead();
      Ptr<WifiMacQueue> GetQueue(Ptr<Node> node);
      bool SetCw(Ptr<Node> node, uint32_t cwMinValue=0, uint32_t cwMaxValue=0);

      Time m_interval = Seconds(0.1);
      Ptr<Node> m_currentNode;
      uint64_t m_rxPktNum;
      Ptr<UdpServer> udpServer;

    };

    }


    #endif // MY_GYM_ENTITY_H
<h3>
  Descripción del codigo:
</h3>

Este archivo se encarga de establecer la comunicación con el agente. Define su espacio de acción y de observación, le brinda las obsrvaciones en cada tiempo, ejecuta las acciones que el agente envia y calcula la recompensa y el fin de la simulación. 

El espacio de accion del agente se define sobre la Congestion Window (CW) de cada agente. El espacio de observación del agente se define por la tupla que contiene cantidad de paquetes que el agente desea enviar, y la cantidad de paquetes perdidos desde la últma acción. 

