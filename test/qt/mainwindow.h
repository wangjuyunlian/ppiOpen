#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class ppiWorker;
class QLocalServer;
class QLocalSocket;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_cbcom_activated(int index);

    void on_local_connect(void);

    void on_pbopen_clicked();

    void on_local_recv(void);

    void on_pbwrite_clicked();

    void on_pbread_clicked();

private:
    void fillPortsInfo();

private:
    Ui::MainWindow *ui;

private:
    ppiWorker *wk;
    QLocalServer *lsvr;
    QLocalSocket *lsk;
};

#endif // MAINWINDOW_H
