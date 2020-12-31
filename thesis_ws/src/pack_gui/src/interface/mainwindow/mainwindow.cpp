//
// Created by ubuntu on 31.12.20.
//

#include "pack_gui/interface/mainwindow/mainwindow.h"
#include <QPushButton>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    // new pointers
    main_layout = new QGridLayout{};
    main_widget = new QWidget{};
    dynamic_reconfigure = new QPushButton{};
    monitor = new QPushButton{};

    // get rid of all the spacings and margins
    main_layout->setContentsMargins(0, 0, 0, 0);
    dynamic_reconfigure->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    monitor->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    // add interface widgets into main_layout
    main_layout->addWidget(dynamic_reconfigure, 0, 0, 1, 1);
    main_layout->addWidget(monitor, 0, 1, 1, 5);
    
    // set main_widget with main_layout
    main_widget->setLayout(main_layout);

    // add main_widget to the mainwindow
    this->setCentralWidget(main_widget);

//    for (int i{}; i < extern_argv.size(); i++)
//        if(extern_argv[i] == "--params-file")
//            test->setText(QString::fromStdString(extern_argv[i+1]));
}

MainWindow::~MainWindow()
{
    delete dynamic_reconfigure;
    delete monitor;
    delete main_layout;
    delete main_widget;

}