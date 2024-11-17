#include "tinyply.h"
#include <string>
#include <fstream>

namespace glUtil {
    class PLYLoader {
    public:
        void load(std::string path){
            std::ifstream file(path, std::ios::binary); //????: maybe need to check whehter it's binary
            
            tinyply::PlyFile tinyFile;
            
        }
    };
}
