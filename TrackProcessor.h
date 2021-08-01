#pragma once
#include <mutex>
#include <iostream>
#include "Track.h"
#include "SharedARTASData.h"
#include "../Network/SharedNetworkData.h"
#include "../Utils.h"
#include "../Log/LogFunctions.h"

#define PART1_1_MASK 0x00000001
#define PART1_1_SHIFT 0
#define PART1_2_MASK 0x00000002
#define PART1_2_SHIFT 1
#define PART10_1_MASK 0x00000FFC
#define PART10_1_SHIFT 2
#define PART10_2_MASK 0x003FF000
#define PART10_2_SHIFT 12
#define PART10_3_MASK 0xFFC00000
#define PART10_3_SHIFT 22

#define HDG_FIX_FSD 120/341

class TrackProcessor {
public:
    
    static void HandleDisconnect() {
        SharedNetworkData::PilotDisconnectMessagesLock.lock();
        SharedNetworkData::PilotDisconnectMessagesQueue.clear();
        SharedNetworkData::PilotDisconnectMessagesLock.unlock();

        SharedNetworkData::PositionUpdateMessagesLock.lock();
        SharedNetworkData::PositionUpdateMessagesQueue.clear();  
        SharedNetworkData::PositionUpdateMessagesLock.unlock();

        SharedARTASData::Tracks.clear();

    }

    static void HandleNumb2Fsd(std::uint32_t data_to_split, 
    std::uint16_t &heading, std::uint16_t &bank, 
    std::uint16_t &pitch, std::uint16_t &part_1_1, std::uint16_t &part_1_2) {
        part_1_1 =     (data_to_split & PART1_1_MASK) >> PART1_1_SHIFT;
        part_1_2 =     (data_to_split & PART1_2_MASK) >> PART1_2_SHIFT;

        heading = (data_to_split & PART10_1_MASK) >> PART10_1_SHIFT;
        bank = (data_to_split & PART10_2_MASK) >> PART10_2_SHIFT;
        pitch = (data_to_split & PART10_3_MASK) >> PART10_3_SHIFT;
    }

    // This processes incoming Position messages from the network to the internal format
    static void ProcessTrackUpdates() {

        // We read pilot disconnect message
        SharedNetworkData::PilotDisconnectMessagesLock.lock();
        if (!SharedNetworkData::PilotDisconnectMessagesQueue.empty()) {
            for (auto Message : SharedNetworkData::PilotDisconnectMessagesQueue) 
            { 
                vector<string> data = vSkyUtils::split(Message, ':');

                if (data.size() < 2)
                    continue;

                string callsign = data.front();
                callsign.erase(0, 3);

                if (SharedARTASData::Tracks.find(callsign) != SharedARTASData::Tracks.end()) {
                    SharedARTASData::Tracks.erase(SharedARTASData::Tracks.find(callsign));
                    #ifdef FIGURE_SHIT_OUT
                        Logger::Info("Deleting pilot " + callsign);
                    #endif
                }

            }
            
            SharedNetworkData::PilotDisconnectMessagesQueue.clear();
        }
        SharedNetworkData::PilotDisconnectMessagesLock.unlock();


        // We start with positionUpdates
        SharedNetworkData::PositionUpdateMessagesLock.lock();
        if (!SharedNetworkData::PositionUpdateMessagesQueue.empty()) {
            for (auto Message : SharedNetworkData::PositionUpdateMessagesQueue) {

                // std::cout << "Processing Track " << Message.Message << std::endl;

                vector<string> data = vSkyUtils::split(Message.Message, ':');
                if (data.size() < 10)
                    continue;

                try {
                    // The track exists, we update it
                    Track t;
                    if (SharedARTASData::Tracks.find(data.at(1)) != SharedARTASData::Tracks.end()) {
                        t = SharedARTASData::Tracks[data.at(1)];
                        Track::PositionHistoryPackage ph;
                        ph.Position = t.Position;
                        ph.ReceivedAt = t.ReceivedAt;
                        ph.Flightlevel = t.Flightlevel;
                        ph.Altitude = t.Altitude;
                        t.PositionHistory.push_back(ph);
                    } else {
                        t.Callsign = data.at(1);
                    }

                    t.Position = Coordinates(stod(data.at(4)), stod(data.at(5)));
                    t.Squawk = data.at(2);
                    t.Altitude = stoi(data.at(6));
                    t.Flightlevel = t.Altitude + stoi(data.at(9));
                    t.Groundspeed = stoi(data.at(7));
                    t.Transponder = Track::TransponderModes::STDBY;
                    if (data.at(0) == "@N")
                        t.Transponder = Track::TransponderModes::C;
                    if (data.at(0) == "@Y")
                        t.Transponder = Track::TransponderModes::IDENT;
                    
                    if (SharedNetworkData::OpenSkyConnection) {
                        t.Heading = static_cast<int>(std::stoi(data.at(8)));
                    } else {
                        std::uint32_t bullshit_package = static_cast<uint32_t>(std::stoul(data.at(8)));
                        std::uint16_t heading, bank, pitch, part_1_1, part_1_2;

                        HandleNumb2Fsd(bullshit_package, heading, bank, pitch, part_1_1, part_1_2);

                        t.Heading = static_cast<int>(heading)*HDG_FIX_FSD;
                    }

                    t.Heading = t.Heading % 360;
                    if (t.Heading < 0)
                            t.Heading += 360;
                    
                    SharedARTASData::Tracks[data.at(1)] = t;
                } catch(std::exception &exc) {
                    Logger::Error("Error processing target " + Message.Message + "  " + exc.what());
                }
            }
            SharedNetworkData::PositionUpdateMessagesQueue.clear();   
        }
        SharedNetworkData::PositionUpdateMessagesLock.unlock();
    };

};
