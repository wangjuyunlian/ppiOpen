#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ppiWorker.h"

#include <QSerialPortInfo>
#include <QLocalSocket>
#include <QLocalServer>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    wk = new ppiWorker;
    lsvr = new QLocalServer;
    lsk = NULL;

    lsvr->listen("ppi");
    lsvr->setMaxPendingConnections(1);
    connect(lsvr, SIGNAL(newConnection()), this, SLOT(on_local_connect()));

    wk->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::fillPortsInfo()
{
    static const QString blankString = QObject::tr("N/A");
    QString description;
    QStringList list;

    ui->cbcom->clear();

    list << QString(tr("刷新"))
         << QString(tr("点击更新列表"));
    ui->cbcom->addItem(list.first(), list);

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QStringList list;
        description = info.description();

        list << info.portName()
             << (!description.isEmpty() ? description : blankString);

        ui->cbcom->addItem(list.first(), list);
    }

    if (ui->cbcom->count() != 1)
    {
        ui->cbcom->setCurrentIndex(1);
    }
}

void MainWindow::on_cbcom_activated(int index)
{
    if (index == 0)
        fillPortsInfo();
}

void MainWindow::on_local_connect(void)
{
    lsk = lsvr->nextPendingConnection();
    connect(lsk, SIGNAL(readyRead()), this, SLOT(on_local_recv()));
}

void MainWindow::on_pbopen_clicked()
{
    QString msg;

    if (ui->cbcom->currentIndex() < 1)
        return;
    if (!lsk)
        return;

    if (ui->pbopen->text() == "打开")
    {
        msg = "open ";
        msg += ui->cbcom->currentText();
    }
    else
    {
        msg = "close";
    }

    lsk->write(msg.toStdString().c_str());
}

void MainWindow::on_local_recv(void)
{
    QByteArray msg;
    QString str;
    QStringList args;

    msg = lsk->readAll();
    str = msg.toStdString().c_str();
    args = str.split(" ");

    if (args[0] == "open")
    {
        if (args[1] == "ok")
            ui->pbopen->setText("关闭");

        return;
    }

    if (args[0] == "closed")
    {
        ui->pbopen->setText("打开");
        return;
    }

    if (args[0] == "ret")
    {
        msg = msg.remove(0, 4);
        str = msg.toStdString().c_str();
        ui->teret->setPlainText(str);
        return;
    }
}

void MainWindow::on_pbwrite_clicked()
{
    QString str;
    QString msg = "write ";

    str = ui->levalue->text();
    if (str.isEmpty())
        return;
    if (ui->pbopen->text() == "打开")
        return;
    msg += ui->da->text();
    msg += " ";
    msg += ui->memtype->currentText();
    msg += " ";
    msg += ui->dattype->currentText();
    msg += " ";
    msg += ui->byteoff->text();
    msg += " ";
    msg += ui->bitoff->text();
    msg += " ";
    msg += ui->num->text();
    msg += " ";

    msg += str;

    lsk->write(msg.toStdString().c_str());
}

void MainWindow::on_pbread_clicked()
{
    QString msg = "read ";

    if (ui->pbopen->text() == "打开")
        return;

    msg += ui->da->text();
    msg += " ";
    msg += ui->memtype->currentText();
    msg += " ";
    msg += ui->dattype->currentText();
    msg += " ";
    msg += ui->byteoff->text();
    msg += " ";
    msg += ui->bitoff->text();
    msg += " ";
    msg += ui->num->text();

    lsk->write(msg.toStdString().c_str());
}
