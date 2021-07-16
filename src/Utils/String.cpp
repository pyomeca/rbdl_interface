#define BIORBD_API_EXPORTS
#include "Utils/String.h"

#include <sstream>
#include <map>
#include <algorithm>
#include <vector>
#include "Utils/Error.h"

biorbd::utils::String::String()
    : std::string("")
{

}

biorbd::utils::String::String(
    const char *text)
    : std::string(text)
{

}

biorbd::utils::String::String(
    const String &text)
    : std::string(text)
{

}

biorbd::utils::String::String(
    const std::basic_string<char> &text)
    : std::string(text)
{

}

biorbd::utils::String &biorbd::utils::String::operator=(
    const biorbd::utils::String &other)
{
    if (this==&other) { // check for self-assigment
        return *this;
    }

    this->std::string::operator=(other);
    return *this;
}

biorbd::utils::String biorbd::utils::String::operator+(
    const char *text)
{
    String tp = *this;
    tp.append(text);
    return tp;
}

biorbd::utils::String biorbd::utils::String::operator+(
    double val)
{
    return *this + to_string(val);
}

biorbd::utils::String biorbd::utils::String::operator+(
    unsigned int val)
{
    return *this + std::to_string(val);
}

biorbd::utils::String biorbd::utils::String::operator+(
    int val)
{
    return *this + std::to_string(val);
}

biorbd::utils::String biorbd::utils::String::operator()(
    unsigned int idx) const
{
    biorbd::utils::Error::check(idx<this->length(),
                                "Index for string out of range");
    char out[2];
    out[0] = (*this)[idx];
    out[1] = '\0';
    return out;
}

biorbd::utils::String biorbd::utils::String::operator()(
    unsigned int startIdx,
    unsigned int lastIdx) const
{
    biorbd::utils::Error::check((startIdx<this->length()
                                 || lastIdx<this->length()), "Index for string out of range");
    biorbd::utils::Error::check(lastIdx>startIdx,
                                "Second argument should be higher than first!");
    char *out = static_cast<char*>(malloc(lastIdx-startIdx+2*sizeof(char)));
    for (unsigned int k=0; k<lastIdx-startIdx+1; ++k) {
        out[k] = (*this)[startIdx+k];
    }
    out[lastIdx-startIdx+1] = '\0';
    biorbd::utils::String Out(out);
    free(out);
    return Out;
}

biorbd::utils::String::~String()
{

}

biorbd::utils::String biorbd::utils::String::tolower(const biorbd::utils::String
        &str)
{
    biorbd::utils::String new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::tolower);
    return new_str;
}

biorbd::utils::String biorbd::utils::String::tolower() const
{
    return tolower(*this);
}

biorbd::utils::String biorbd::utils::String::toupper(const biorbd::utils::String
        &str)
{
    biorbd::utils::String new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::toupper);
    return new_str;
}

biorbd::utils::String biorbd::utils::String::toupper() const
{
    return toupper(*this);
}

biorbd::utils::String biorbd::utils::String::to_string(
    double val)
{
    std::ostringstream out;
    out.precision(20);
    out << std::fixed << val;
    return removeTrailing(out.str(), "0");
}

biorbd::utils::String biorbd::utils::String::to_string(
    float val)
{
    std::ostringstream out;
    out.precision(20);
    out << std::fixed << val;
    return removeTrailing(out.str(), "0");
}

biorbd::utils::String biorbd::utils::String::removeTrailing(
    const biorbd::utils::String &origin,
    const biorbd::utils::String &trailTag)
{
    biorbd::utils::Error::check(trailTag.length() == 1,
                                "Tag should be of length 1");
    biorbd::utils::String out(origin);

    while(out.length() > 0 && out.back() == trailTag[0]) {
        out.pop_back();
    }
    return out;
}

std::ostream &operator<<(std::ostream &os, const biorbd::utils::String &a)
{
    os << a.c_str();
    return os;
}
