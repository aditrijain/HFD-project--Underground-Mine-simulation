#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include <iostream>
#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MineSafetySimulation");

// Generate random CO value
double GenerateCOValue()
{
    static std::default_random_engine gen;
    static std::uniform_real_distribution<double> dist(20.0, 120.0);
    return dist(gen);
}

// Function to send alert packet
void SendAlert(Ptr<Socket> socket, Ipv4Address dstAddr, uint16_t dstPort, uint32_t sensorId, uint32_t workerId)
{
    std::ostringstream msg;
    msg << "ALERT: High CO detected by Sensor " << sensorId << " for Worker " << workerId;
    std::string data = msg.str();

    Ptr<Packet> packet = Create<Packet>((const uint8_t*)data.c_str(), data.size());
    socket->SendTo(packet, 0, InetSocketAddress(dstAddr, dstPort));

    NS_LOG_UNCOND("    >>> ALERT SENT from SensorNode(" << sensorId << ") to WorkerNode(" << workerId << ")");
}

// Periodic CO sensing and alert
void SensorReadAndAlert(Ptr<Node> sensorNode, Ptr<Socket> sendSocket, NodeContainer workers, Ipv4InterfaceContainer allIfs, uint32_t sensorCount)
{
    Ptr<MobilityModel> sensorMob = sensorNode->GetObject<MobilityModel>();
    double coValue = GenerateCOValue();
    double threshold = 70.0;

    NS_LOG_UNCOND("[t=" << Simulator::Now().GetSeconds() << "s] SensorNode(" << sensorNode->GetId()
                    << ") CO=" << coValue << " ppm (threshold=" << threshold << " ppm)");

    if (coValue > threshold)
    {
        for (uint32_t i = 0; i < workers.GetN(); ++i)
        {
            Ptr<Node> worker = workers.Get(i);
            Ptr<MobilityModel> workerMob = worker->GetObject<MobilityModel>();
            double dist = sensorMob->GetDistanceFrom(workerMob);

            if (dist < 10.0) // proximity radius
            {
                Ipv4Address workerAddr = allIfs.GetAddress(sensorCount + i);
                SendAlert(sendSocket, workerAddr, 8080, sensorNode->GetId(), worker->GetId());
            }
        }
    }

    Simulator::Schedule(Seconds(1.0), &SensorReadAndAlert, sensorNode, sendSocket, workers, allIfs, sensorCount);
}

int main(int argc, char* argv[])
{
    double simTime = 30.0;
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation duration", simTime);
    cmd.Parse(argc, argv);

    NodeContainer sensors;
    sensors.Create(3);

    NodeContainer workers;
    workers.Create(5);

    NodeContainer allNodes;
    allNodes.Add(sensors);
    allNodes.Add(workers);

    // Wi-Fi setup (ad-hoc)
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(10.0));
    phy.Set("TxPowerEnd", DoubleValue(10.0));
    phy.Set("RxGain", DoubleValue(0));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, allNodes);

    // Internet stack
    InternetStackHelper stack;
    stack.Install(allNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Mobility: fixed sensors
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(10.0, 20.0, 0.0));
    positionAlloc->Add(Vector(30.0, 40.0, 0.0));
    positionAlloc->Add(Vector(60.0, 10.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensors);

    // Mobility: random-walking workers
    MobilityHelper mobWorkers;
    mobWorkers.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                    "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=80.0]"),
                                    "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=60.0]"));
    mobWorkers.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(0, 100, 0, 100)),
                                "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=3.0]"),
                                "Distance", DoubleValue(5.0));
    mobWorkers.Install(workers);

    // Create sockets: workers receive on port 8080
    uint16_t port = 8080;
    for (uint32_t i = 0; i < workers.GetN(); ++i)
    {
        Ptr<Socket> recvSocket = Socket::CreateSocket(workers.Get(i), UdpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), port);
        recvSocket->Bind(local);
    }

    // For each sensor, create a send socket and schedule periodic sensing
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        Ptr<Socket> sendSocket = Socket::CreateSocket(sensors.Get(i), UdpSocketFactory::GetTypeId());
        Simulator::Schedule(Seconds(1.0 + i), &SensorReadAndAlert, sensors.Get(i), sendSocket, workers, interfaces, sensors.GetN());
    }

    // ------------------- 🟢 NetAnim + UDP packet metadata tracing --------------------
    AnimationInterface anim("mine-3.xml");

    anim.EnablePacketMetadata(true); // crucial: records UDP/IPv4 packets in XML
    anim.EnableIpv4RouteTracking("routes.xml", Seconds(0), Seconds(simTime), Seconds(0.25));

    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        anim.UpdateNodeColor(sensors.Get(i)->GetId(), 255, 0, 0);  // red sensors
        anim.UpdateNodeDescription(sensors.Get(i)->GetId(), "Sensor-" + std::to_string(i));
    }
    for (uint32_t i = 0; i < workers.GetN(); ++i)
    {
        anim.UpdateNodeColor(workers.Get(i)->GetId(), 0, 255, 0);  // green workers
        anim.UpdateNodeDescription(workers.Get(i)->GetId(), "Worker-" + std::to_string(i));
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    NS_LOG_UNCOND("Simulation finished. Open mine-3.xml in NetAnim, select 'UDP' to visualize alerts.");
    return 0;
}

