//
// Created by ubuntu on 31.12.20.
//

#include "rclcpp/rclcpp.hpp"
#include "pack_gui/interface/extern_variables.h"
#include "pack_gui/interface/mainwindow/mainwindow.h"

#include <QApplication>
#include <QPushButton>

//declare extern variables
std::vector<std::string> extern_argv;

int main(int argc, char * argv[])
{
    // define extern_argv
    for(int i{}; i < argc; i++)
        extern_argv.push_back(argv[i]);

    //create interface application
    QApplication a{argc, argv};
    MainWindow w{};
    w.show();
    a.exec();

    return 0;
}