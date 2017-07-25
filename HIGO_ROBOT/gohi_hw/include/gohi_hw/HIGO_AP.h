#pragma once

#ifndef HIGO_AP_H_
#define HIGO_AP_H_

#include <fstream>
#include <gohi_hw/transport_serial.h>
#include <hf_link.h>
#include <cstdlib>

class HIGO_AP
{
public:
	 HIGO_AP(std::string url, std::string config_addr);

    bool updateCommand(const Command &command, int count);

    void updateRobot();

    inline RobotAbstract* getRobotAbstract()
    {
        return &my_robot_;
    }

    inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
    {
        return port_->getIOinstace();
    }

    bool reconfig()
    {

    }

    inline bool initialize_ok () const
    {
        return initialize_ok_;
    }

    inline void checkHandshake()
    {
        if (hflink_->getReceiveRenewFlag(SHAKING_HANDS)==1)
        {
            sendCommand(SHAKING_HANDS);
            std::cout<<"send shake hands"<<std::endl;
        }
    }

private:
    boost::shared_ptr<Transport> port_;
    boost::shared_ptr<HFLink> hflink_;
    boost::shared_ptr<boost::asio::deadline_timer> timer_;

    //for reading config file
    std::fstream file_;
    bool initialize_ok_;
    //for updating data
    int hflink_command_set_[LAST_COMMAND_FLAG];
    int hflink_freq_[LAST_COMMAND_FLAG];
    int hflink_count_[LAST_COMMAND_FLAG];
    int hflink_command_set_current_[LAST_COMMAND_FLAG];

    int time_out_;
    bool time_out_flag_;
    boost::mutex wait_mutex_;
    bool ack_ready_;
    void timeoutHandler(const boost::system::error_code &ec);

    inline uint8_t checkUpdate(const Command command_state)
    {
        if (hflink_command_set_current_[command_state] & hflink_->getReceiveRenewFlag(command_state))
        {
            return 1;
        }
        if (hflink_command_set_current_[command_state] == 0 ) return 1;
        return 0;
    }

    inline void sendCommand(const Command command_state)
    {
        std::cout<<"send message  "<<command_state <<std::endl;
        hflink_->masterSendCommand(command_state);
        Buffer data(hflink_->getSerializedData(), hflink_->getSerializedLength() + hflink_->getSerializedData());
        port_->writeBuffer(data);
    }

    // a single object for robot
    RobotAbstract my_robot_;

};



#endif
