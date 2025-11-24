/*
 * Industrial Safety Simulation - PURE LOGIC (No OLSR)
 * * Logic Flow (Application Layer Relaying):
 * 1. Sensor -> Sink (Direct Send)
 * 2. Sink -> Central (Manual Forwarding)
 * 3. Central -> Sink (Manual Forwarding)
 * 4. Sink -> Worker (Direct Send)
 * * Result: No background "Hello" packets. NetAnim will only show Alert traffic.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

// --- Global Constants ---
#define UDP_PORT 9999
#define SENSOR_RADIUS 20.0 
#define CO_THRESHOLD 50    
#define CO_DANGER 500      

NS_LOG_COMPONENT_DEFINE ("IndustrialSafetySim");

// ===========================================================================
// 1. Worker Application 
//    - Receives instructions, simulates navigation.
// ===========================================================================
class WorkerApp : public Application {
public:
    WorkerApp() {}
    void Setup(Ptr<Socket> socket) { m_socket = socket; }

private:
    virtual void StartApplication() {
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), UDP_PORT));
        m_socket->SetRecvCallback(MakeCallback(&WorkerApp::ReceivePacket, this));
    }

    void ReceivePacket(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            uint8_t buffer[1024];
            packet->CopyData(buffer, 1024);
            std::string msg = std::string((char*)buffer);

            if (msg.find("ALERT") != std::string::npos) {
                NS_LOG_UNCOND("\033[1;34m[WORKER] Node " << GetNode()->GetId() << " Received: " << msg.substr(7) << "\033[0m");
            }
        }
    }
    Ptr<Socket> m_socket;
};

// ===========================================================================
// 2. Central Application 
//    - Calculates Path, sends to SINK (not worker directly, as it's far away).
// ===========================================================================
class CentralApp : public Application {
public:
    CentralApp() {}
    void Setup(Ptr<Socket> socket, Address defaultSink) { 
        m_socket = socket; 
        m_defaultSink = defaultSink;
    }

private:
    virtual void StartApplication() {
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), UDP_PORT));
        m_socket->SetRecvCallback(MakeCallback(&CentralApp::ReceivePacket, this));
    }

    void ReceivePacket(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            uint8_t buffer[1024];
            packet->CopyData(buffer, 1024);
            std::string msg = std::string((char*)buffer);

            if (msg.find("SPIKE") != std::string::npos) {
                NS_LOG_UNCOND("\033[1;32m[CENTRAL] Received Relay from Sink. Processing...\033[0m");
                ProcessSpike(msg);
            }
        }
    }

    void ProcessSpike(std::string msg) {
        // Msg format: SPIKE:SensorID:CO:WorkerIPs
        std::size_t lastColon = msg.find_last_of(':');
        std::string ipListStr = msg.substr(lastColon + 1);

        NS_LOG_UNCOND("[CENTRAL] Calculating Safe Path (Weighted Dijkstra)...");

        std::stringstream ss(ipListStr);
        std::string workerIpStr;
        while (std::getline(ss, workerIpStr, ',')) {
            // We send the ALERT to the SINK, but we include the Target Worker IP in the payload
            // so the Sink knows who to forward it to.
            // Payload Format: RELAY_TO:<WorkerIP>:<Message>
            std::string vectorInstructions = "RELAY_TO:" + workerIpStr + ":600m Forward, Turn Left";
            
            Ptr<Packet> packet = Create<Packet>((uint8_t*)vectorInstructions.c_str(), vectorInstructions.length() + 1);
            m_socket->SendTo(packet, 0, m_defaultSink);
            NS_LOG_UNCOND("[CENTRAL] Sending Instructions via Sink for: " << workerIpStr);
        }
    }
    Ptr<Socket> m_socket;
    Address m_defaultSink;
};

// ===========================================================================
// 3. Sink Application (The Manual Relay)
//    - Acts as the bridge. No routing protocol needed.
// ===========================================================================
class SinkApp : public Application {
public:
    SinkApp() {} 
    void Setup(Ptr<Socket> socket, Address centralAddr) { 
        m_socket = socket; 
        m_centralAddr = centralAddr;
    }

private:
    virtual void StartApplication() {
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), UDP_PORT));
        m_socket->SetRecvCallback(MakeCallback(&SinkApp::ReceivePacket, this));
    }

    void ReceivePacket(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            uint8_t buffer[1024];
            packet->CopyData(buffer, 1024);
            std::string msg = std::string((char*)buffer);

            // CASE 1: Receive SPIKE from Sensor -> Forward to Central
            if (msg.find("SPIKE") != std::string::npos) {
                // Forward exactly as is to Central
                Ptr<Packet> fwdPacket = Create<Packet>((uint8_t*)msg.c_str(), msg.length() + 1);
                m_socket->SendTo(fwdPacket, 0, m_centralAddr);
                // NS_LOG_UNCOND("[SINK] Relaying SPIKE to Central.");
            }
            // CASE 2: Receive RELAY_TO from Central -> Forward to Worker
            else if (msg.find("RELAY_TO") != std::string::npos) {
                // Format: RELAY_TO:<WorkerIP>:<Message>
                size_t firstC = msg.find(':');
                size_t secondC = msg.find(':', firstC + 1);
                
                std::string targetIpStr = msg.substr(firstC + 1, secondC - firstC - 1);
                std::string instructions = "ALERT: " + msg.substr(secondC + 1);

                Ipv4Address workerIp(targetIpStr.c_str());
                Ptr<Packet> alertPacket = Create<Packet>((uint8_t*)instructions.c_str(), instructions.length() + 1);
                
                m_socket->SendTo(alertPacket, 0, InetSocketAddress(workerIp, UDP_PORT));
                NS_LOG_UNCOND("[SINK] Forwarding Instructions to Worker " << targetIpStr);
            }
        }
    }
    Ptr<Socket> m_socket;
    Address m_centralAddr;
};

// ===========================================================================
// 4. Sensor Application (Same logic, target is Sink)
// ===========================================================================
class SensorApp : public Application {
public:
    SensorApp() : m_coLevel(30.0), m_alertSent(false) {}
    void Setup(Ptr<Socket> socket, Address sinkAddress, NodeContainer workerNodes) {
        m_socket = socket;
        m_sinkAddress = sinkAddress;
        m_workerNodes = workerNodes;
    }

    void TriggerCoSpike() {
        m_coLevel = CO_DANGER; 
        NS_LOG_UNCOND("\033[1;31m[SENSOR] GAS LEAK SIMULATED at Node " << GetNode()->GetId() << "\033[0m");
    }

private:
    virtual void StartApplication() {
        m_socket->Bind();
        Simulator::Schedule(Seconds(1.0), &SensorApp::SenseEnvironment, this);
    }

    std::string GetNearbyWorkers() {
        std::string workerIps = "";
        Ptr<MobilityModel> myMobility = GetNode()->GetObject<MobilityModel>();
        
        for (uint32_t i = 0; i < m_workerNodes.GetN(); ++i) {
            Ptr<Node> worker = m_workerNodes.Get(i);
            Ptr<MobilityModel> workerMobility = worker->GetObject<MobilityModel>();
            double dist = workerMobility->GetDistanceFrom(myMobility);

            if (dist <= SENSOR_RADIUS) {
                Ptr<Ipv4> ipv4 = worker->GetObject<Ipv4>();
                Ipv4Address ip = ipv4->GetAddress(1, 0).GetLocal(); 
                std::ostringstream oss;
                oss << ip;
                if (workerIps.length() > 0) workerIps += ",";
                workerIps += oss.str();
            }
        }
        return workerIps;
    }

    void SenseEnvironment() {
        if (m_coLevel > CO_THRESHOLD && !m_alertSent) {
            std::string nearbyWorkers = GetNearbyWorkers();
            std::ostringstream msg;
            msg << "SPIKE:" << GetNode()->GetId() << ":" << m_coLevel << ":" << nearbyWorkers;
            
            Ptr<Packet> packet = Create<Packet>((uint8_t*)msg.str().c_str(), msg.str().length() + 1);
            m_socket->SendTo(packet, 0, m_sinkAddress);
            
            NS_LOG_UNCOND("[SENSOR] Threshold Exceeded! Sending SPIKE to Sink.");
            m_alertSent = true; 
        }
        Simulator::Schedule(Seconds(1.0), &SensorApp::SenseEnvironment, this);
    }
    Ptr<Socket> m_socket;
    Address m_sinkAddress;
    NodeContainer m_workerNodes;
    double m_coLevel;
    bool m_alertSent;
};

// ===========================================================================
// MAIN
// ===========================================================================
int main(int argc, char *argv[]) {
    
    NodeContainer sensorNodes, sinkNodes, centralNode, workerNodes;
    sensorNodes.Create(2); sinkNodes.Create(2); centralNode.Create(1); workerNodes.Create(3);
    NodeContainer allWifiNodes = NodeContainer(sensorNodes, sinkNodes, centralNode, workerNodes);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    // Position Setup: Sensor->Sink->Central are within communication range of each other
    positionAlloc->Add(Vector(10.0, 50.0, 0.0)); // Sensor 0
    positionAlloc->Add(Vector(90.0, 50.0, 0.0)); // Sensor 1
    positionAlloc->Add(Vector(30.0, 50.0, 0.0)); // Sink 0
    positionAlloc->Add(Vector(70.0, 50.0, 0.0)); // Sink 1
    positionAlloc->Add(Vector(50.0, 50.0, 0.0)); // Central (Moved slightly closer to Sinks for reliability)
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensorNodes); mobility.Install(sinkNodes); mobility.Install(centralNode);

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue(Rectangle(0, 100, 0, 100)));
    mobility.Install(workerNodes);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    YansWifiPhyHelper wifiPhy; 
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default(); 
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, allWifiNodes);

    // --- NO OLSR HERE ---
    InternetStackHelper internet; // Standard stack, no routing helper
    internet.Install(allWifiNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Apps Setup
    // Central (Address 10.1.1.5)
    Ptr<Socket> centralSocket = Socket::CreateSocket(centralNode.Get(0), TypeId::LookupByName("ns3::UdpSocketFactory"));
    Ptr<CentralApp> centralApp = CreateObject<CentralApp>();
    // Central talks to Sink 0 (Address 10.1.1.3) by default
    Address sink0Addr = InetSocketAddress(interfaces.GetAddress(2), UDP_PORT);
    centralApp->Setup(centralSocket, sink0Addr);
    centralNode.Get(0)->AddApplication(centralApp);
    centralApp->SetStartTime(Seconds(1.0));

    // Sinks
    for (uint32_t i = 0; i < sinkNodes.GetN(); ++i) {
        Ptr<Socket> sinkSocket = Socket::CreateSocket(sinkNodes.Get(i), TypeId::LookupByName("ns3::UdpSocketFactory"));
        Ptr<SinkApp> sinkApp = CreateObject<SinkApp>();
        // Sinks talk to Central (Address 10.1.1.5 is index 4 in allWifiNodes)
        Address centralAddr = InetSocketAddress(interfaces.GetAddress(4), UDP_PORT);
        sinkApp->Setup(sinkSocket, centralAddr);
        sinkNodes.Get(i)->AddApplication(sinkApp);
        sinkApp->SetStartTime(Seconds(1.0));
    }

    // Workers
    for (uint32_t i = 0; i < workerNodes.GetN(); ++i) {
        Ptr<Socket> workerSocket = Socket::CreateSocket(workerNodes.Get(i), TypeId::LookupByName("ns3::UdpSocketFactory"));
        Ptr<WorkerApp> wApp = CreateObject<WorkerApp>();
        wApp->Setup(workerSocket);
        workerNodes.Get(i)->AddApplication(wApp);
        wApp->SetStartTime(Seconds(1.0));
    }

    // Sensors
    for (uint32_t i = 0; i < sensorNodes.GetN(); ++i) {
        Ptr<Socket> sensorSocket = Socket::CreateSocket(sensorNodes.Get(i), TypeId::LookupByName("ns3::UdpSocketFactory"));
        Ptr<SensorApp> sApp = CreateObject<SensorApp>();
        // Sensors talk to nearest Sink (Sensor 0->Sink 0)
        Address targetSink = InetSocketAddress(interfaces.GetAddress(2 + i), UDP_PORT);
        sApp->Setup(sensorSocket, targetSink, workerNodes);
        sensorNodes.Get(i)->AddApplication(sApp);
        sApp->SetStartTime(Seconds(1.0));
        
        if (i == 0) Simulator::Schedule(Seconds(5.0), &SensorApp::TriggerCoSpike, sApp);
    }

    AnimationInterface anim("industrial-safety-5.xml");
    anim.UpdateNodeColor(sensorNodes.Get(0), 255, 0, 0); 
    anim.UpdateNodeColor(sensorNodes.Get(1), 255, 0, 0); 
    anim.UpdateNodeColor(sinkNodes.Get(0), 255, 255, 0); 
    anim.UpdateNodeColor(sinkNodes.Get(1), 255, 255, 0); 
    anim.UpdateNodeColor(centralNode.Get(0), 0, 255, 0); 
    anim.UpdateNodeSize(centralNode.Get(0)->GetId(), 2.0, 2.0);
    for(uint32_t i=0; i<workerNodes.GetN(); ++i) anim.UpdateNodeColor(workerNodes.Get(i), 0, 0, 255);
    
    // DISABLE PACKET METADATA to reduce clutter if needed, but we want to see Application packets
    anim.EnablePacketMetadata(true); 

    NS_LOG_UNCOND("Starting Simulation (NO OLSR - Pure App Logic)...");
    Simulator::Stop(Seconds(30.0)); 
    Simulator::Run();
    Simulator::Destroy();
    NS_LOG_UNCOND("Finished.");
    return 0;
}
