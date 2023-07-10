import argparse


def get_source(args: argparse.Namespace) -> str:
    return f"""\
#include <{args.package_name}/sample_class.hpp>



/**
 * @brief SampleClass::SampleClass the default constructor.
 */
SampleClass::SampleClass():
    a_private_qt_string_("I am a QString"),
    a_private_eigen3_matrix_((Eigen::Matrix2d() << 1,2,3,4).finished())
{{

}}

/**
 * @brief SampleClass::get_a_private_member.
 * @return an int with the value of a_private_member_.
 */
int SampleClass::get_a_private_member() const
{{
    return a_private_member_;
}}

/**
 * @brief SampleClass::set_a_private_member.
 * @param [in] value The new value for a_private_member_.
 */
void SampleClass::set_a_private_member(int value)
{{
    a_private_member_ = value;
}}

/**
 * @brief SampleClass::sum_of_squares.
 * @param [in] a The first number.
 * @param [in] b The second number.
 * @return a*a + 2*a*b + b*b.
 */
double SampleClass::sum_of_squares(const double &a, const double &b)
{{
    return a*a + 2*a*b + b*b;
}}

/**
 * @brief SampleClass::to_string converts a SampleClass to a std::string representation.
 * @return a pretty(-ish) std::string representation of the object.
 */
std::string SampleClass::to_string() const
{{
    std::stringstream ss;
    ss << "Sample_Class:: " << std::endl <<
          "a_private_member_ = "        << std::to_string(a_private_member_) << std::endl <<
          "a_private_qt_string_ = "     << a_private_qt_string_.toStdString() << std::endl <<
          "a_private_eigen3_matrix_ = " << a_private_eigen3_matrix_ << std::endl;
    return ss.str();
}}

/**
 * @brief operator << the stream operator for SampleClass objects.
 * @param [in/out] the std::ostream to be modified.
 * @param [in] sc the SampleClass whose representation is to be streamed.
 * @return the modified os with the added SampleClass string representation.
 * @see SampleClass::to_string().
 */
std::ostream &operator<<(std::ostream &os, const SampleClass &sc)
{{
    return os << sc.to_string();
}}
"""
