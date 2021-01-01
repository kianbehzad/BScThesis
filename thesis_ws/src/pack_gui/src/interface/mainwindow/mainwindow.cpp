//
// Created by ubuntu on 31.12.20.
//

#include "pack_gui/interface/mainwindow/mainwindow.h"
#include <QPushButton>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    // new pointers
    splitter = new QSplitter{};
    dynamic_reconfigure = new DynamicReconfigure{};
    monitor = new QPushButton{};

    // get rid of all the spacings and margins
    dynamic_reconfigure->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    monitor->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    // add interface widgets into main_layout
    splitter->addWidget(dynamic_reconfigure);
    splitter->addWidget(monitor);

    // add main_widget to the mainwindow
    this->setCentralWidget(splitter);

}

MainWindow::~MainWindow()
{
    delete dynamic_reconfigure;
    delete monitor;
    delete splitter;

}