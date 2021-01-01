//
// Created by ubuntu on 31.12.20.
//

#ifndef PACK_GUI_MAINWINDOW_H
#define PACK_GUI_MAINWINDOW_H

#include "pack_gui/interface/extern_variables.h"
#include "pack_gui/interface/mainwindow/dynamic_reconfigure/dynamic_reconfigure.h"
#include <QMainWindow>
#include <QSplitter>
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
    QSplitter* splitter;
    DynamicReconfigure* dynamic_reconfigure;
    QPushButton* monitor;



};

#endif //PACK_GUI_MAINWINDOW_H
