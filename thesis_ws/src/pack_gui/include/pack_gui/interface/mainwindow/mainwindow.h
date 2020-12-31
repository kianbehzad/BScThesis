//
// Created by ubuntu on 31.12.20.
//

#ifndef PACK_GUI_MAINWINDOW_H
#define PACK_GUI_MAINWINDOW_H

#include "pack_gui/interface/extern_variables.h"
#include <QMainWindow>
#include <QGridLayout>
#include <QWidget>
#include <QPushButton>


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = 0);
    ~MainWindow();

private:
    QWidget* main_widget;
    QGridLayout* main_layout;
    QPushButton* dynamic_reconfigure;
    QPushButton* monitor;



};

#endif //PACK_GUI_MAINWINDOW_H
