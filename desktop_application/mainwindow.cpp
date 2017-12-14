#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    step_sizes << "1" << "1/2" << "1/4" << "1/8";

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        ui->serialPortComboBox->addItem(info.portName());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_stepSlider_valueChanged(int value)
{
    ui->stepValueLabel->setText(step_sizes[value]);
}
