#ifndef STLPARSER_H
#define STLPARSER_H

#include <QString>
#include <QFile>
#include <QTextStream>
class STLParser
{
public:
    STLParser();
    ~STLParser();

    void open(QString path);
    std::vector<float> getVertices();

private:
    QFile m_stlFile;
    std::vector<float> vertices;
    QTextStream textStream;
    bool binary;

};

#endif // STLPARSER_H

