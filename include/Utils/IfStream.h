#ifndef BIORBD_UTILS_IFSTREAM_H
#define BIORBD_UTILS_IFSTREAM_H

#include <map>
#include "biorbdConfig.h"
#include "Utils/Path.h"
#include <iostream>
#include "rbdl/rbdl_math.h"

namespace biorbd
{
namespace utils
{
class Equation;
///
/// \brief Wrapper for the an std::ifstream with increased capacities
///
class BIORBD_API IfStream
{
public:
    ///
    /// \brief Construct IfStream
    ///
    IfStream();

    ///
    /// \brief Construct IfStream
    /// \param path The file path to open
    /// \param mode The open mode of "std::ios_base" base
    ///
    IfStream(
        const biorbd::utils::Path& path,
        std::ios_base::openmode mode );

    ///
    /// \brief Construct IfStream
    /// \param path The file path to open
    /// \param mode The open mode of "std::ios_base" base
    ///
    IfStream(
        const char* path,
        std::ios_base::openmode mode );

    ///
    /// \brief Open the file
    /// \param path The file path to open
    /// \param mode The open mode of "std::ios_base" base
    /// \return True on success
    ///
    bool open(
        const biorbd::utils::Path& path,
        std::ios_base::openmode mode );

    ///
    /// \brief Read a word in the file skipping the word if it is c-like commented
    /// \param text The text read (output)
    /// \return True on success
    ///
    bool read(
        biorbd::utils::String& text);

    ///
    /// \brief Read a word in the file
    /// \param text The text read (output)
    /// \return True on success
    ///
    bool readAWord(
        biorbd::utils::String& text);

    ///
    /// \brief Read an integer in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        int& val);

    ///
    /// \brief Read an boolean in the file
    /// \param val The value read (output)
    /// \return True on success
    ///
    bool read(
        bool& val);

    ///
    /// \brief Read an unsigned integer in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        unsigned int& val);

    ///
    /// \brief Read an double in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        double& val);
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Read an double in the file
    /// \param val The number read (output)
    /// \return True on success
    ///
    bool read(
        RBDLCasadiMath::MX_Xd_SubMatrix val);
#endif

    ///
    /// \brief Read and evaluate an equation
    /// \param result The number read (output)
    /// \param variables The variable set
    /// \return True on success
    ///
    bool read(
        double& result,
        const std::map<biorbd::utils::Equation, double> &variables);
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Read and evaluate an equation
    /// \param result The number read (output)
    /// \param variables The variable set
    /// \return True on success
    ///
    bool read(
        RBDLCasadiMath::MX_Xd_SubMatrix result,
        const std::map<biorbd::utils::Equation, double> &variables);
#endif

    ///
    /// \brief Advance in the file to a specific tag
    /// \param tag The tag to reach
    /// \param text The text that follows a tag
    /// \return True on success
    ///
    bool readSpecificTag(
        const biorbd::utils::String& tag,
        biorbd::utils::String& text);

    ///
    /// \brief Advance in the file to a specific tag
    /// \param tag The tag to reach
    /// \return True on success
    ///
    bool reachSpecificTag(
        const biorbd::utils::String& tag);

    ///
    /// \brief Counts the number of consecutive lines starting with the same tag and then brings it back to the initial position
    /// \param tag The tag to count
    /// \return The number of consecutive lines starting with the same tag
    ///
    int countTagsInAConsecutiveLines(
        const biorbd::utils::String& tag);

    ///
    /// \brief Read a whole line
    /// \param text The text read (output)
    ///
    void getline(
        biorbd::utils::String& text);

    ///
    /// \brief Close the file
    ///
    bool close();

    ///
    /// \brief Return if the file is at the end
    /// \return If the file is at the end
    ///
    bool eof();

protected:
    std::shared_ptr<bool> m_isOpen;///< If file is open

private:
    std::shared_ptr<std::ifstream> m_ifs;///< the ifstream
    std::shared_ptr<biorbd::utils::Path> m_path;///< The path of the file
};

}
}

#endif // BIORBD_UTILS_IFSTREAM_H

