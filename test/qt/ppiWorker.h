#ifndef PPIWORKER_H
#define PPIWORKER_H

#include <QObject>
#include <QThread>

class ppiWorker : public QThread
{
    Q_OBJECT

public:
    ppiWorker();
    ~ppiWorker();

private:
    static void _debug(void *obj, const char* fmt, ...);
    void test_multMemRead(void *ctx);
    void test_bitRead(void *ctx);
    void test_errRead(void *ctx);
    QString doread(void *ctx, QStringList &args);
    QString dowrite(void *ctx, QStringList &args);
    void ppi_gettype(QString &smt, QString &sdt, int &mt, int &dt);

private:
    void run();
};

#endif // PPIWORKER_H
