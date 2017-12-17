#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStringList>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>

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

private:
    Ui::MainWindow *ui;
    QStringList step_sizes;
    QSerialPort serial;

    bool OpenSerialPort(const QString& port);
    bool CloseSerialPort();
};

#endif // MAINWINDOW_H
