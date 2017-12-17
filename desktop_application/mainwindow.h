#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStringList>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>

#include <QDebug>

typedef struct
{
    uint8_t mode;
    uint8_t direction;
    uint16_t speed;
    uint16_t steps;
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

private:
    Ui::MainWindow *ui;
    QStringList step_sizes;
    QSerialPort serial;

    void TransmitParameters();
    bool OpenSerialPort(const QString& port);
    bool CloseSerialPort();
};

#endif // MAINWINDOW_H
