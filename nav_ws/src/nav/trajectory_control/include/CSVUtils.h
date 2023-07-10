#pragma once 

#include <cmath>
#include <cstdlib>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


class CSVRow
{
public:

    std::string const& operator[](std::size_t index) const
    {
        return m_data[index];
    }

    std::size_t size() const
    {
        return m_data.size();
    }

    void readNextRow(std::istream& str)
    {
        std::string line;
        std::getline(str, line);

        std::stringstream lineStream(line);
        std::string cell;

        m_data.clear();
        while (std::getline(lineStream, cell, ','))
        {
            m_data.push_back(cell);
        }
    }

private:
    std::vector<std::string> m_data;

};


inline std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}


class CSVIterator
{
public:

    typedef std::input_iterator_tag iterator_category;
    typedef CSVRow value_type;
    typedef std::size_t difference_type;
    typedef CSVRow* pointer;
    typedef CSVRow& reference;

    CSVIterator(std::istream& str) : m_str(str.good() ? &str : NULL)
    {
        ++(*this);
    }

    CSVIterator() : m_str(NULL)
    {
    }

    // Pre Increment
    // CSVIterator& operator++() {if (m_str) { (*m_str) >> m_row;m_str = m_str->good()?m_str:NULL;}return *this;}
    CSVIterator& operator++()
    {
        if (m_str)
        {
            if (!((*m_str) >> m_row))
            {
                ;
                m_str = NULL;
            }
        }
        return *this;
    }
    
    // Post increment
    CSVIterator operator++(int)
    {
        CSVIterator tmp(*this);
        ++(*this);
        return tmp;
    }

    CSVRow const& operator*() const
    {
        return m_row;
    }

    CSVRow const* operator->() const
    {
        return &m_row;
    }

    bool operator==(CSVIterator const& rhs)
    {
        return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
    }

    bool operator!=(CSVIterator const& rhs)
    {
        return !((*this) == rhs);
    }

private:

    std::istream* m_str;
    CSVRow m_row;
};

struct DataSet
{
    std::vector<std::vector<double> > x;
    std::vector<std::vector<double> > y;
};


inline bool readCSVDataFromFile(const std::string& filename, int x_index, int y_index, DataSet& d)
{
    //    std::cout << "index x: " << x_index << std::endl;
    //    std::cout << "index y: " << y_index << std::endl;
    if (y_index <= x_index)
        return false;

    std::ifstream file(filename);
    std::string::size_type sz;
    for (CSVIterator loop(file); loop != CSVIterator(); ++loop)
    {
        if (y_index > (*loop).size())
        {
            std::cout << "Line size: " << (*loop).size() << std::endl;
            return false;
        }
        std::vector<double> x;
        std::vector<double> y;
        for (unsigned int i = 0; i < x_index; i++)
        {
            double v = std::stod((*loop)[i], &sz);
            //std::cout << "input element(" << v << ")\n";
            x.push_back(v);
        }

        for (unsigned int i = x_index; i < y_index; i++)
        {
            double v = std::stod((*loop)[i], &sz);
            //std::cout << "target element(" << v << ")\n";
            y.push_back(v);
        }

        d.x.push_back(x);
        d.y.push_back(y);
    }
    return true;
}

inline std::vector<double> computeMean(std::vector<std::vector<double> > y){
    std::vector<double> mean;
    for(unsigned int k = 0; k < y.at(0).size(); k++){
        mean.push_back(0);
    }
    
    for(unsigned int i = 0; i < y.size(); i++){
        for(unsigned int j = 0; j < y[i].size(); j++){
            double m = mean[j];
            m += y.at(i).at(j);
            mean[j] = m;
        }
    }
    
    for(unsigned int k = 0; k < mean.size(); k++){
        double m = mean[k];
        m = m/y.size();
        mean[k] = m;
    }
    
    return mean;
}

inline std::vector<double> computeStandardDeviation(std::vector<std::vector<double> > y){
    
    std::vector<double> sd;
    for(unsigned int k = 0; k < y.at(0).size(); k++){
        sd.push_back(0);
    }
    
    std::vector<double> mean = computeMean(y);
    
    for(unsigned int i = 0; i < y.size(); i++){
        for(unsigned int j = 0; j < y[i].size(); j++){
            double d = sd[j];
            d += pow(y.at(i).at(j) - mean[j], 2);
            sd[j] = d;
        }
    }
    
    for(unsigned int k = 0; k < sd.size(); k++){
        double d = sd[k];
        d = sqrt(d/y.size());
        sd[k] = d;
    }
    
    
    return sd;
}


