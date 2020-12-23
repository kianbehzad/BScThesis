#include "pack_protobuf_wrapper/common/net/udpsend.h"

UDPSend::UDPSend(std::string address, int _port) { // : QObject(parent)
    connect = true;
    QString add(address.c_str());
    host.setAddress(add);
    port = _port;
    socket = new QUdpSocket();
    socket->bind(host, port);

}

void UDPSend::setIP(std::string _ip) {
    QString add(_ip.c_str());
    connect = false;
    socket->disconnectFromHost();
    host.clear();
    host.setAddress(add);
    socket->bind(host, port);
    connect = true;
}

void UDPSend::setport(int _port) {
    connect = false;
    port = _port;
    socket->bind(host, port);
    connect = true;
}

void UDPSend::send(std::string buf) {

    QByteArray datagram(buf.c_str(), buf.length());
    if (connect) {
        socket->writeDatagram(datagram, host, port);
    }
//        ROS_INFO("sending byte: %lld", socket->writeDatagram(datagram, host, port));

}

UDPSend::~UDPSend() = default;
