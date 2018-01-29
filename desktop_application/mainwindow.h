#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStringList>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QTimer>

#include <QDebug>

#define IS_HEADER(in)           (((in) & 0b11100000) == 0b10100000)
#define IS_RESET(in)            ((in) & 1 << 0)
#define IS_SINGLESTEPPING(in)   ((in) & 1 << 1)
#define IS_OVERHEAT(in)         ((in) & 1 << 2)
#define IS_OVERVOLTAGE(in)      ((in) & 1 << 3)
#define IS_UNDERVOLTAGE(in)     ((in) & 1 << 4)

typedef struct
{
    uint8_t header;
    uint8_t mode;
    uint16_t speed;
    uint32_t steps;
} data_t;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_stepSlider_valueChanged(int value);

    void on_connectButton_clicked();

    void on_disconnectButton_clicked();

    void on_speedSlider_valueChanged(int value);

    void on_loopedCheckBox_toggled(bool checked);

    void on_flipDirectionCheckBox_toggled(bool checked);

    void on_rotateButton_clicked();

    void on_stopButton_clicked();

private:
    Ui::MainWindow *ui;
    QStringList step_sizes;
    QSerialPort serial;
    QTimer timer;

    void TransmitParameters();
    void TransmitStop();
    bool OpenSerialPort(const QString& port);
    bool CloseSerialPort();

    void ControlsEnabled(bool enabled);

    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);
    void handleTimeout();
};

#endif // MAINWINDOW_H
