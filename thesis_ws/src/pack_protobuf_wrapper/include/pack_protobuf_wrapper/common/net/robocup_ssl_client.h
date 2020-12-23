//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    robocup_ssl_client.h
  \brief   C++ Interface: robocup_ssl_client
  \author  Stefan Zickler, 2009
*/
//========================================================================
#ifndef ROBOCUP_SSL_CLIENT_H
#define ROBOCUP_SSL_CLIENT_H
#include "netraw.h"
#include <string>
#include "pack_protobuf_wrapper/proto/referee.pb.h"
using namespace std;
/**
  @author Author Name
*/

class RoboCupSSLClient {
protected:
    static const int MaxDataGramSize = 65536;
    char * in_buffer;
    Net::UDP mc; // multicast client
    int _port;
    string _net_address;
    string _net_interface;
public:
    RoboCupSSLClient(int port,
                     string net_ref_address,
                     string net_ref_interface = "");

    ~RoboCupSSLClient();
    bool open(bool blocking = false);
    void close();
    bool receive(::google::protobuf::Message & packet);

};

#endif