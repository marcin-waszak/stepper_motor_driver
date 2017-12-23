#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    step_sizes << "1" << "1/2" << "1/4" << "1/8" << "1/16";

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        ui->serialPortComboBox->addItem(info.portName());

    ui->disconnectButton->setEnabled(false);
    ui->driverGroupBox->setEnabled(false);
    ui->singleRotationGroupBox->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_stepSlider_valueChanged(int value)
{
    ui->stepValueLabel->setText(step_sizes[value]);
    if(ui->loopedCheckBox->isChecked())
        TransmitParameters();
}

bool MainWindow::OpenSerialPort(const QString& port)
{
    serial.setPortName(port);
    serial.setBaudRate(QSerialPort::Baud115200);

    if(!serial.open(QIODevice::ReadWrite))
        return false;

    ui->serialPortComboBox->setEnabled(false);
    ui->connectButton->setEnabled(false);
    ui->disconnectButton->setEnabled(true);
    ui->driverGroupBox->setEnabled(true);

    return true;
}

bool MainWindow::CloseSerialPort()
{
    serial.close();

    ui->serialPortComboBox->setEnabled(true);
    ui->connectButton->setEnabled(true);
    ui->disconnectButton->setEnabled(false);
    ui->driverGroupBox->setEnabled(false);

    return true;
}

void MainWindow::on_connectButton_clicked()
{
    if(!OpenSerialPort(ui->serialPortComboBox->currentText()))
        QMessageBox::critical(this, "Connection error",
            "Cannot connect to the device! Error code: " + QString::number(serial.error()));

    // Do something
}

void MainWindow::on_disconnectButton_clicked()
{
    CloseSerialPort();
}

void MainWindow::on_speedSlider_valueChanged(int value)
{
    ui->speedValueLabel->setText(QString::number(value));
    if(ui->loopedCheckBox->isChecked())
        TransmitParameters();
}

void MainWindow::TransmitParameters()
{
    data_t message = {
        1u << ui->stepSlider->value() | (ui->loopedCheckBox->isChecked() ? 0 : 0x80),
        ui->flipDirectionCheckBox->isChecked(),
        static_cast<uint16_t>(ui->speedSlider->value()),
        ui->stepsLineEdit->text().toUInt() * (1u << ui->stepSlider->value())
    };

    serial.write(reinterpret_cast<char*>(&message),
                 sizeof(message));
}

void MainWindow::TransmitStop()
{
    data_t message = {
        0x80,
        0,
        0,
        0
    };

    serial.write(reinterpret_cast<char*>(&message),
                 sizeof(message));
}

void MainWindow::on_loopedCheckBox_toggled(bool checked)
{
    ui->singleRotationGroupBox->setEnabled(!checked);

    if(checked)
        TransmitParameters();
    else
        TransmitStop();
}

void MainWindow::on_flipDirectionCheckBox_toggled(bool checked)
{
    if(ui->loopedCheckBox->isChecked())
        TransmitParameters();
}

void MainWindow::on_rotateButton_clicked()
{
    TransmitParameters();
}

void MainWindow::on_stopButton_clicked()
{
    TransmitStop();
}
