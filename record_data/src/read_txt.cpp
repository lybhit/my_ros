#include "read_txt.h"

int countLines(const char * filename)
{
    ifstream readFile;
    int n = 0;
    string tmp;
    readFile.open(filename, ios::in);

    if(readFile.fail())
    {
        return 0;
    }
    else{
        while(getline(readFile, tmp, '\n'))
        {
            n++;
        }

        readFile.close();
        return n;
    }
}

string readLine(const char* filename, int line)
{
    int lines, i = 0;
    string tmp;
    fstream file;
    file.open(filename, ios::in);
    lines = countLines(filename);
    if(line < 0)
    {
        return "Error 1: line number is wrong, it cannot be 0 or negtive.";
    }
    if(file.fail())
    {
        return "Error 2: file is not exist.";
    }

    if(line > lines)
    {
        return "Error 3: row number is over file range.";
    }

    while(getline(file, tmp) && i < line - 1)
    {
        i++;
    }
    file.close();
    
    return tmp;
}