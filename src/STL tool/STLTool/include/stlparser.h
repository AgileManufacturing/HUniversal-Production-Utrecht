#ifndef STLPARSER_H
#define STLPARSER_H

#include <QString>
#include <QFile>
#include <QTextStream>

/**
 * @brief The STLParser class
 *
 * This function is responsible for parsing STL files
 * and storing the vertice data in a vector.
 */
class STLParser{
public:
    /**
     * @brief STLParser constructor
     */
    STLParser();
    ~STLParser();
    /**
     * @brief open
     *
     * This function opens a file and determines whether it is
     * in binary or ascii format.
     * @param path The path to the STL file.
     */
    void open(QString path);
    /**
     * @brief getVertices
     *
     * @return Returns a vector with the vertices from the openend STL file.
     */
    std::vector<float> getVertices();

private:
    QFile m_stlFile;
    std::vector<float> vertices;
    QTextStream textStream;
    bool binary;

};

#endif // STLPARSER_H

