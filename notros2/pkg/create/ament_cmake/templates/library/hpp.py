

def get_source() -> str:
    return f"""\
#pragma once

#include <ostream>

#include <QString>
#include <eigen3/Eigen/Dense>

class SampleClass
{{
private:
    int a_private_member_;
    const QString a_private_qt_string_;
    const Eigen::MatrixXd a_private_eigen3_matrix_;

public:
    SampleClass();

    int get_a_private_member() const;
    void set_a_private_member(int value);
    std::string to_string() const;

    static double sum_of_squares(const double&a, const double& b);
}};

std::ostream& operator<<(std::ostream& os, const SampleClass& sc);
"""