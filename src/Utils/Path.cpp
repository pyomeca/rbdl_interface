#define BIORBD_API_EXPORTS
#include "Utils/Path.h"

#include <fstream>
#include <regex>

#include "Utils/String.h"
#include "Utils/Error.h"

#ifdef _WIN32
    #include <direct.h>
    #include <Windows.h>
    #undef max
#else
    #include <sys/stat.h>
    #include <unistd.h>
#endif

biorbd::utils::Path::Path() :
    m_originalPath(std::make_shared<biorbd::utils::String>()),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{

}

biorbd::utils::Path::Path(
    const char *path) :
    m_originalPath(std::make_shared<biorbd::utils::String>(path)),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{
    parseFileName(*m_originalPath, *m_folder, *m_filename, *m_extension);
    setIsFolderAbsolute();
}

biorbd::utils::Path::Path(
    const biorbd::utils::String &path) :
    m_originalPath(std::make_shared<biorbd::utils::String>(path)),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{
    parseFileName(*m_originalPath, *m_folder, *m_filename, *m_extension);
    setIsFolderAbsolute();
}

biorbd::utils::Path::Path(
    const std::basic_string<char> &path) :
    m_originalPath(std::make_shared<biorbd::utils::String>(path)),
    m_folder(std::make_shared<biorbd::utils::String>()),
    m_isFolderAbsolute(std::make_shared<bool>()),
    m_filename(std::make_shared<biorbd::utils::String>()),
    m_extension(std::make_shared<biorbd::utils::String>())
{
    parseFileName(*m_originalPath, *m_folder, *m_filename, *m_extension);
    setIsFolderAbsolute();
}

biorbd::utils::Path biorbd::utils::Path::DeepCopy() const
{
    biorbd::utils::Path copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::Path::DeepCopy(
    const Path &other)
{
    *m_originalPath = *other.m_originalPath;
    *m_folder = *other.m_folder;
    *m_isFolderAbsolute = *other.m_isFolderAbsolute;
    *m_filename = *other.m_filename;
    *m_extension = *other.m_extension;
}

bool biorbd::utils::Path::isFileExist() const
{
    return isFileExist(absolutePath());
}
bool biorbd::utils::Path::isFileExist(
    const Path& path)
{
    return isFileExist(path.absolutePath());
}
bool biorbd::utils::Path::isFileExist(
    const biorbd::utils::String& path)
{
    if (FILE *file = fopen(
#ifdef _WIN32
                         toWindowsFormat(path).c_str(),
#else
                         path.c_str(),
#endif
                         "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

bool biorbd::utils::Path::isFileReadable() const
{
    std::ifstream fichier(
#ifdef _WIN32
        toWindowsFormat(absolutePath()).c_str()
#else
        absolutePath().c_str()
#endif
    );
    bool isOpen(fichier.is_open());
    fichier.close();
    return isOpen;
}

bool biorbd::utils::Path::isFolderExist() const
{
    return isFolderExist(*this);
}

bool biorbd::utils::Path::isFolderExist(
    const Path &path)
{
    return isFolderExist(path.folder());
}

bool biorbd::utils::Path::isFolderExist(
    const biorbd::utils::String & path)
{
#ifdef _WIN32
    if (GetFileAttributesA(toWindowsFormat(path).c_str())
            == INVALID_FILE_ATTRIBUTES) {
        return false;    // Le path est invalide
    } else
        // Si on est ici, c'est que quelque chose existe,
        // s'assurer que ça ne soit pas un fichier
        if (isFileExist(path)) {
            return false;
        } else {
            return true;
        }
#else
    struct stat statbuf;
    if (stat(path.c_str(), &statbuf) != -1) {
        return true;
    } else {
        return false;
    }
#endif

}

void biorbd::utils::Path::parseFileName(
    const biorbd::utils::String &path,
    biorbd::utils::String &folder,
    biorbd::utils::String &filename,
    biorbd::utils::String &extension)
{
    biorbd::utils::String pathSep(toUnixFormat(path));

    size_t sepPos(pathSep.rfind("/"));

    // Stocker le folder
    if (sepPos == std::string::npos)
        // If no separator is found, then there is no separator
        // and therefore the path is ./
    {
        folder = "";
    } else {
        folder = pathSep.substr(0, sepPos) + "/";
    }

    if (folder.find("./") == 0) { // Remove the leading ./ if necessary
        folder = folder.substr(2);
    }

    // Stocker l'extension
    size_t ext(pathSep.rfind("."));
    if (ext != SIZE_MAX) {
        extension = pathSep.substr(ext+1);
    } else {
        extension = "";
    }

    // Stocker le nom de fichier
    filename = pathSep.substr(sepPos+1, ext- sepPos-1);
}

biorbd::utils::String biorbd::utils::Path::relativePath()  const
{
    return relativePath(*this, currentDir());
}

biorbd::utils::String biorbd::utils::Path::relativePath(
    const biorbd::utils::String& relativeTo) const
{
    return relativePath(*this, relativeTo);
}

biorbd::utils::String biorbd::utils::Path::relativePath(
    const biorbd::utils::Path &path,
    const biorbd::utils::String &relativeTo)
{
    biorbd::utils::String me(path.absolutePath());
    biorbd::utils::String currentDir(relativeTo);

    biorbd::utils::String meFirstPart("");
    biorbd::utils::String currentDirFirstPart("");

    // Set the separator to the 0 position
    size_t sepMe = std::string::npos;
    size_t sepCurrentDir = std::string::npos;
    do {
        // cut according to previous separator
        me = me.substr(sepMe+1);
        currentDir = currentDir.substr(sepCurrentDir+1);

        // Find the next separator
        sepMe = me.find("/"); // UNIX formalism
        sepCurrentDir = currentDir.find("/");

        // Separate first and last part
        meFirstPart = me.substr(0, sepMe);
        currentDirFirstPart = currentDir.substr(0, sepCurrentDir);

        // While the first part are equal,
        // we still can advance to find de closest relative part
    } while(!meFirstPart.compare(currentDirFirstPart));

    biorbd::utils::String outPath;
    while (currentDir.compare("")) {
        // Tant que currentDir n'est pas vide, reculer
        // Trouver le prochain séparateur
        sepCurrentDir = currentDir.find("/");

        // Séparer la première et la dernière partie
        if (sepCurrentDir != std::string::npos) {
            // -1 Si on est au dernier dossier
            // et que celui-ci ne se termine pas par "/"
            currentDirFirstPart = currentDir.substr(0, sepCurrentDir);
            currentDir = currentDir.substr(sepCurrentDir+1);
        } else {
            currentDir = "";
        }

        outPath += "../";
        // Tant que les premières parties sont égales,
        // continuer à avancer dans le path
    };
    if (!outPath.compare("") && me.find("../") != 0) {
        outPath += "./";
    }

    outPath += me;

    return outPath;
}

biorbd::utils::String biorbd::utils::Path::absoluteFolder(
    const biorbd::utils::Path &path)
{
    if (*path.m_isFolderAbsolute) {
        return path.folder();
    }

    biorbd::utils::String base;
#ifdef _WIN32
    biorbd::utils::String current(currentDir());
    std::smatch matches;

    if (std::regex_search(current, matches, std::regex("^([A-Z]):[\\/].*$"))) {
        base = matches[1].str() + ":/";
    } else {
        biorbd::utils::Error::raise("I could not find the current drive to estimate the path");
    }
#else
    base = "/";
#endif
    return base + relativePath(path, base);
}

biorbd::utils::String biorbd::utils::Path::absoluteFolder() const
{
    if (*m_isFolderAbsolute) {
        return *m_folder;
    } else {
        return currentDir() + *m_folder;
    }
}

biorbd::utils::String biorbd::utils::Path::absolutePath() const
{
    if (m_filename->compare("")) {
        if (m_extension->compare("")) {
            return absoluteFolder() + *m_filename + "." + *m_extension;
        } else {
            return absoluteFolder() + *m_filename;
        }
    } else {
        return absoluteFolder();
    }
}

biorbd::utils::String biorbd::utils::Path::toUnixFormat(
    const biorbd::utils::String& path)
{
    biorbd::utils::String pathOut(path);

    // Depending on the string origin, "\\" is either the character "\"
    // escaped or the character "\" written twice. Test for both
    size_t pos(pathOut.rfind("\\\\"));
    while (pos != std::string::npos) {
        pathOut.replace(pos, 2, "/");
        pos = pathOut.rfind("\\\\");
    }

    // However, this next hunk can create false positive each time a
    // legitimate escape character is used (should not happen in a path?)
    pos = pathOut.rfind("\\");
    while (pos != std::string::npos) {
        pathOut.replace(pos, 1, "/");
        pos = pathOut.rfind("\\");
    }
    return pathOut;
}

biorbd::utils::String biorbd::utils::Path::toWindowsFormat(
    const biorbd::utils::String &path)
{
    biorbd::utils::String pathOut(path);
    size_t pos(pathOut.rfind("/"));
    while (pos != std::string::npos) {
        pathOut.replace(pos, 1, "\\\\");
        pos = pathOut.rfind("/");
    }
    return pathOut;
}

const biorbd::utils::String &biorbd::utils::Path::originalPath() const
{
    return *m_originalPath;
}

const biorbd::utils::String &biorbd::utils::Path::folder() const
{
    return *m_folder;
}

void biorbd::utils::Path::setFilename(
    const biorbd::utils::String& name)
{
    *m_filename = name;
}

const biorbd::utils::String& biorbd::utils::Path::filename() const
{
    return *m_filename;
}

void biorbd::utils::Path::setExtension(
    const biorbd::utils::String &ext)
{
    *m_extension = ext;
}

const biorbd::utils::String& biorbd::utils::Path::extension() const
{
    return *m_extension;
}

void biorbd::utils::Path::setIsFolderAbsolute()
{
    biorbd::utils::String base;
#ifdef _WIN32
    biorbd::utils::String current(*m_folder);
    std::smatch matches;

    if (std::regex_search(current, matches, std::regex("^([A-Z]):[\\/].*$"))) {
        *m_isFolderAbsolute = true;
    } else {
        *m_isFolderAbsolute = false;
    }
#else
    base = "/";
    size_t pos(m_folder->find(base.c_str()));
    if (pos == 0) {
        *m_isFolderAbsolute = true;
    } else {
        *m_isFolderAbsolute = false;
    }
#endif
}

biorbd::utils::String biorbd::utils::Path::currentDir()
{
    char buff[FILENAME_MAX];
#ifdef _WIN32
    biorbd::utils::Error::check(_getcwd(buff, FILENAME_MAX),
                                "Could not find the current directory");
#else
    biorbd::utils::Error::check(getcwd(buff, FILENAME_MAX),
                                "Could not find the current directory");
#endif
    return toUnixFormat(buff) + "/";
}

void biorbd::utils::Path::createFolder() const
{
    const biorbd::utils::String& tp(folder());
    biorbd::utils::String tp2(tp);

    size_t sep = std::string::npos;
    size_t sepTrack = 0;
    do {
        // Découper en fonction de la position précédente du séparateur
        tp2 = tp2.substr(sep+1);

        // Trouver le prochain séparateur
        sep = tp2.find("/"); // Path are stored in UNIX formalism
        if (sep != std::string::npos) {
            sepTrack += sep + 1 ;

            // Séparer la première et la dernière partie
            if (!isFolderExist(
                        static_cast<biorbd::utils::String>(
                            tp.substr(0, sepTrack)))) {
#ifdef _WIN32
                _mkdir(toWindowsFormat(tp.substr(0, sepTrack)).c_str());
#else
                mkdir(tp.substr(0, sepTrack).c_str(), 0777);
#endif
            }
        }
    } while (sep != std::string::npos);
}
