#ifndef BIORBD_UTILS_EQUATION_H
#define BIORBD_UTILS_EQUATION_H

#include <vector>
#include <map>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd
{
namespace utils
{

///
/// \brief Strings that are to be interpreted as equation that can be evaluated
///
class BIORBD_API Equation : public biorbd::utils::String
{
public:
    ///
    /// \brief Construct Equation
    ///
    Equation();

    ///
    /// \brief Construct Equation
    /// \param string The equation in a char format
    ///
    Equation(const char *string);

    ///
    /// \brief Construct Equation
    /// \param string The equation in a string format
    ///
    Equation(const biorbd::utils::String &string);

    ///
    /// \brief Construct Equation
    /// \param string The equation in a list of char format
    ///
    Equation(const std::basic_string<char> &string);

    ///
    /// \brief Split equation into smaller parts, down to number or math symbols
    /// \param wholeEq The whole equation to split
    /// \param variables The set of variables in the equation
    /// \return The split equation
    ///
    static std::vector<biorbd::utils::Equation> splitIntoEquation(
        biorbd::utils::Equation wholeEq,
        const std::map<biorbd::utils::Equation, double>& variables);

    ///
    /// \brief Evaluate and return an equation
    /// \param wholeEq The already split equation to evaluate
    /// \return The evaluated equation
    ///
    static double evaluateEquation(
        std::vector<biorbd::utils::Equation> wholeEq);

    ///
    /// \brief Evaluate and return an equation
    /// \param wholeEq The whole equation to evaluate
    /// \return The evaluated equation
    ///
    static double evaluateEquation(
        biorbd::utils::Equation wholeEq);

    ///
    /// \brief Evaluate and return an equation
    /// \param wholeEq The whole equation to evaluate
    /// \param variables The variables in the equation
    /// \return The evaluated equation
    ///
    static double evaluateEquation(
        biorbd::utils::Equation wholeEq,
        const std::map<biorbd::utils::Equation, double>& variables);

    ///
    /// \brief Replace constants in the split equation by a number
    /// \param eq The split equation
    ///
    /// The supported constants are:
    ///
    ///   - pi -- that evaluates to M_PI, that is \f$3.14159265358979323846\f$ on UNIX
    ///
    ///
    static void replaceCste(
        std::vector<biorbd::utils::Equation> &eq);

    ///
    /// \brief Replace the varirables in the equation by their values
    /// \param eq The equation to replace the variables
    /// \param variables The variable set
    ///
    static void replaceVar(
        biorbd::utils::Equation &eq,
        const std::map<biorbd::utils::Equation, double>& variables);

protected:
    ///
    /// \brief Resolve the equation
    /// \param eq The equation to resolve
    /// \param math The mathematical symbol that is being evaluated now
    ///
    static double evaluateEquation(
        std::vector<biorbd::utils::Equation> eq,
        unsigned int math);

    ///
    /// \brief Prepare the mathematical symbols
    /// \return The math symbols in an order that respect the order of operation
    ///
    /// The supported symbols are:
    ///
    ///   - "(" -- Open a parenthese
    ///   - ")" -- Close a parenthese
    ///   - "e" -- value of the format 1e2 (i.e. 1x10^2)
    ///   - "/" -- Division
    ///   - "*" -- Multiplication
    ///   - "+" -- Addition
    ///   - "-" -- Subtraction, or negative number if it starts the equation
    ///
    static std::vector<biorbd::utils::Equation> prepareMathSymbols();
};

}
}

#endif // BIORBD_UTILS_EQUATION_H
