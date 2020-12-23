#ifndef UDPSEND_H
#define UDPSEND_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QString>
#include <QByteArray>


class UDPSend { // : public QObject
    // Q_OBJECT
public:
    explicit UDPSend(std::string address, int _port);
    ~UDPSend();
    void send(std::string buf);
    void setIP(std::string _ip);
    void setport(int _port);

private:
    QUdpSocket* socket;
    QHostAddress host;
    int port;
    bool connect;



};

#endif // UDPSEND_H
