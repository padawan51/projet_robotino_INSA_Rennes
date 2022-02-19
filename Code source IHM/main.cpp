#include "mainwindow.h"
#include "utils.h"

#include <QApplication>
#include <QTranslator>
#include <QLocale>
#include <QLibraryInfo>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString locale = QLocale::system().name().section('_', 0, 0);
    QTranslator translator;
    translator.load(QString("qt_") + locale, QLibraryInfo::location(QLibraryInfo::TranslationsPath));
    app.installTranslator(&translator);

    /*if(processRepeat("ihm_tino2.exe") < 2){
        w.show();

        return app.exec();
    }
    else return 0;*/
    MainWindow w;
    w.show();

    return app.exec();
}
